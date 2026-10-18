#include "render/renderer.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <limits>
#include <string>
#include <vector>

namespace fsim {

namespace {

// The sun: where the light comes from (and the shadows point away from).
const glm::vec3 kSun = glm::normalize(glm::vec3(0.4F, 1.0F, 0.3F));
constexpr int kShadowSize = 2048; // the shadow map's pixels, each way

// Every box is the unit cube (or cylinder), from -0.5 to 0.5, drawn many
// times at once (instancing): each copy reads its own placement, size,
// colour and flags from the instance buffer, one record per box.
constexpr const char* kBoxVertex = R"(#version 330 core
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec4 aPlace0;  // the box's placement (rotation and position):
layout(location = 3) in vec4 aPlace1;  // a mat4 takes four attributes, one per column
layout(location = 4) in vec4 aPlace2;
layout(location = 5) in vec4 aPlace3;
layout(location = 6) in vec3 aSize;
layout(location = 7) in vec4 aColor;   // rgb and alpha
layout(location = 8) in vec4 aParams;  // stripe spacing, stripe offset, flags, -
uniform mat4 uViewProjection;
uniform mat4 uLightSpace;              // world -> the shadow map's view
out vec3 vNormal;
out vec3 vLocal;                       // the position on the box, in metres from its centre, in its own axes
out vec3 vLocalNormal;
out vec4 vLight;                       // the position as the sun sees it
flat out vec3 vSize;
flat out vec4 vColor;
flat out vec4 vParams;
void main() {
    mat4 place = mat4(aPlace0, aPlace1, aPlace2, aPlace3);
    vec4 world = place * vec4(aPosition * aSize, 1.0);
    vNormal = mat3(place) * aNormal;
    vLocal = aPosition * aSize;
    vLocalNormal = aNormal;
    vLight = uLightSpace * world;
    vSize = aSize;
    vColor = aColor;
    vParams = aParams;
    gl_Position = uViewProjection * world;
}
)";

// How much sun reaches a point: 1 lit, 0 in shadow. The shadow map holds,
// for each spot the sun sees, the distance to the nearest surface; a point
// further away than that is behind something. Nine samples around it (PCF)
// make the edge soft instead of jagged; each sample is itself filtered
// between four texels by the hardware (sampler2DShadow, LINEAR).
constexpr const char* kShadowFunction = R"(
uniform sampler2DShadow uShadowMap;
uniform float uShadows; // 0: shadows off
float sunlight(vec4 light, float slope) {
    if (uShadows < 0.5) return 1.0;
    vec3 p = light.xyz / light.w * 0.5 + 0.5;
    if (p.x < 0.0 || p.x > 1.0 || p.y < 0.0 || p.y > 1.0 || p.z > 1.0) return 1.0; // outside the map: lit
    // A little bias, more on surfaces the sun grazes, so a surface doesn't
    // shadow itself ("shadow acne").
    float bias = mix(0.0015, 0.0004, slope);
    vec2 texel = 1.0 / vec2(textureSize(uShadowMap, 0));
    float lit = 0.0;
    for (int x = -1; x <= 1; ++x)
        for (int y = -1; y <= 1; ++y)
            lit += texture(uShadowMap, vec3(p.xy + vec2(x, y) * texel, p.z - bias));
    return lit / 9.0;
}
)";

constexpr const char* kBoxFragmentStart = R"(#version 330 core
in vec3 vNormal;
in vec3 vLocal;
in vec3 vLocalNormal;
in vec4 vLight;
flat in vec3 vSize;
flat in vec4 vColor;
flat in vec4 vParams;
out vec4 fragColor;
const vec3 kSun = normalize(vec3(0.4, 1.0, 0.3));
)";

constexpr const char* kBoxFragmentMain = R"(
void main() {
    int flags = int(vParams.z + 0.5);
    bool unlit = (flags & 1) != 0;
    bool highlight = (flags & 2) != 0;
    bool round = (flags & 4) != 0;
    if (unlit) {
        fragColor = vColor;
        return;
    }
    vec3 color = vColor.rgb;
    // A moving belt: light stripes across its top, moving with the surface.
    if (vParams.x > 0.0 && vLocalNormal.y > 0.5) {
        float s = fract((vLocal.x + 0.5 * vSize.x - vParams.y) / vParams.x);
        if (s < 0.06) color = mix(color, vec3(0.75), 0.6);
    }
    // Darker edges, so boxes of the same colour stay readable: the distance
    // to the nearest edge of this face, in metres. A cylinder's only edges
    // are its rims.
    vec3 toEdge = 0.5 * vSize - abs(vLocal);
    vec3 n = abs(vLocalNormal);
    float edge = n.x > 0.5 ? min(toEdge.y, toEdge.z) : n.y > 0.5 ? min(toEdge.x, toEdge.z) : min(toEdge.x, toEdge.y);
    if (round) edge = n.y > 0.5 ? 0.5 * vSize.x - length(vLocal.xz) : toEdge.y;
    color *= mix(0.55, 1.0, smoothstep(0.0, 0.006, edge));
    // Light: the sun where it isn't shadowed, plus a sky light that's
    // brighter from above.
    vec3 normal = normalize(vNormal);
    float facing = dot(normal, kSun);
    float sun = max(facing, 0.0) * (facing > 0.0 ? sunlight(vLight, facing) : 1.0);
    float sky = 0.35 + 0.25 * normal.y;
    color *= sky + 0.75 * sun;
    if (highlight) color = mix(color, vec3(0.35, 0.65, 1.0), 0.45); // a blue tint
    fragColor = vec4(color, vColor.a);
}
)";

// The shadow pass: the same boxes from the sun, only their depth.
constexpr const char* kDepthVertex = R"(#version 330 core
layout(location = 0) in vec3 aPosition;
layout(location = 2) in vec4 aPlace0;
layout(location = 3) in vec4 aPlace1;
layout(location = 4) in vec4 aPlace2;
layout(location = 5) in vec4 aPlace3;
layout(location = 6) in vec3 aSize;
layout(location = 8) in vec4 aParams;
uniform mat4 uLightSpace;
flat out int vFlags;
void main() {
    vFlags = int(aParams.z + 0.5);
    gl_Position = uLightSpace * mat4(aPlace0, aPlace1, aPlace2, aPlace3) * vec4(aPosition * aSize, 1.0);
}
)";

constexpr const char* kDepthFragment = R"(#version 330 core
flat in int vFlags;
void main() {
    if ((vFlags & 1) != 0) discard; // lights (LEDs, beams) cast no shadow
}
)";

// The floor: a grid of lines every metre and every 10 cm, fading with
// distance, and the line's shadows on it.
constexpr const char* kFloorVertex = R"(#version 330 core
layout(location = 0) in vec3 aPosition;
uniform mat4 uViewProjection;
uniform mat4 uLightSpace;
out vec3 vWorld;
out vec4 vLight;
void main() {
    vWorld = aPosition;
    vLight = uLightSpace * vec4(aPosition, 1.0);
    gl_Position = uViewProjection * vec4(aPosition, 1.0);
}
)";

constexpr const char* kFloorFragmentStart = R"(#version 330 core
in vec3 vWorld;
in vec4 vLight;
uniform vec3 uCamera;
out vec4 fragColor;
)";

constexpr const char* kFloorFragmentMain = R"(
// How strongly a point is on a grid line, `spacing` metres apart; fwidth
// keeps the lines one pixel wide at any distance.
float line(vec2 p, float spacing) {
    vec2 g = abs(fract(p / spacing - 0.5) - 0.5) / fwidth(p / spacing);
    return 1.0 - min(min(g.x, g.y), 1.0);
}

void main() {
    vec3 color = vec3(0.30, 0.31, 0.33);
    float fade = 1.0 - smoothstep(8.0, 30.0, length(vWorld.xz - uCamera.xz));
    color = mix(color, vec3(0.36, 0.37, 0.40), line(vWorld.xz, 0.1) * 0.5 * fade);
    color = mix(color, vec3(0.50, 0.52, 0.55), line(vWorld.xz, 1.0) * fade);
    color *= mix(0.6, 1.0, sunlight(vLight, 0.9));
    fragColor = vec4(color, 1.0);
}
)";

std::string join(std::initializer_list<const char*> parts) {
    std::string text;
    for (const char* part : parts) {
        text += part;
    }
    return text;
}

// 6 faces x 2 triangles x 3 corners, each with a position and a normal.
std::array<float, 216> cubeVertices() {
    std::array<float, 216> data{};
    std::size_t i = 0;
    // For each face: its normal axis and sign; the two other axes span it.
    for (int axis = 0; axis < 3; ++axis) {
        for (const float sign : {-1.0F, 1.0F}) {
            const int u = (axis + 1) % 3;
            const int v = (axis + 2) % 3;
            // Two triangles, wound counter-clockwise seen from outside.
            const float corners[6][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, -1}, {1, 1}, {-1, 1}};
            for (int c = 0; c < 6; ++c) {
                const int k = sign > 0 ? c : 5 - c; // the far side is seen from the other way
                float position[3];
                position[axis] = 0.5F * sign;
                position[u] = 0.5F * corners[k][0];
                position[v] = 0.5F * corners[k][1];
                float normal[3] = {0.0F, 0.0F, 0.0F};
                normal[axis] = sign;
                for (float p : position)
                    data[i++] = p;
                for (float n : normal)
                    data[i++] = n;
            }
        }
    }
    return data;
}

// A cylinder of diameter 1 and height 1 around the y axis, centred on the
// origin like the cube: `kCylinderSegments` flat sides (with normals
// pointing straight out, so the light makes it look round) and two lids.
constexpr int kCylinderSegments = 32;

std::vector<float> cylinderVertices() {
    std::vector<float> data;
    const auto add = [&](glm::vec3 p, glm::vec3 n) { data.insert(data.end(), {p.x, p.y, p.z, n.x, n.y, n.z}); };
    const auto rim = [](int i) {
        const float a = 2.0F * 3.14159265F * static_cast<float>(i) / kCylinderSegments;
        return glm::vec3(0.5F * std::cos(a), 0.0F, -0.5F * std::sin(a)); // counter-clockwise seen from above
    };
    const glm::vec3 up{0.0F, 0.5F, 0.0F};
    for (int i = 0; i < kCylinderSegments; ++i) {
        const auto a = rim(i);
        const auto b = rim(i + 1);
        const auto na = glm::normalize(a);
        const auto nb = glm::normalize(b);
        // The side: two triangles, wound counter-clockwise seen from outside.
        add(a - up, na);
        add(b - up, nb);
        add(b + up, nb);
        add(a - up, na);
        add(b + up, nb);
        add(a + up, na);
        // The lids.
        add(up, {0, 1, 0});
        add(a + up, {0, 1, 0});
        add(b + up, {0, 1, 0});
        add(-up, {0, -1, 0});
        add(b - up, {0, -1, 0});
        add(a - up, {0, -1, 0});
    }
    return data;
}

// One box's record in the instance buffer, as the vertex shader reads it.
struct Instance {
    float place[16]; // rotation and position, column by column
    float size[3];
    float color[4];  // with alpha
    float params[4]; // stripe spacing, stripe offset, flags, unused
};
static_assert(sizeof(Instance) == 27 * sizeof(float), "tightly packed, as the attribute pointers assume");

constexpr float kUnlit = 1.0F;
constexpr float kHighlight = 2.0F;
constexpr float kRound = 4.0F;

Instance instanceOf(const DrawBox& box) {
    Instance instance{};
    const auto place = glm::translate(glm::mat4(1.0F), box.center) * glm::mat4_cast(box.rotation);
    std::copy_n(glm::value_ptr(place), 16, instance.place);
    std::copy_n(glm::value_ptr(box.size), 3, instance.size);
    const glm::vec4 color(box.color, box.alpha);
    std::copy_n(glm::value_ptr(color), 4, instance.color);
    instance.params[0] = box.stripeSpacing;
    instance.params[1] = box.stripeOffset;
    instance.params[2] =
        (box.unlit ? kUnlit : 0.0F) + (box.highlight ? kHighlight : 0.0F) + (box.round ? kRound : 0.0F);
    return instance;
}

// A mesh's vertex array: its vertices (position, normal) from its own
// buffer, and the per-box attributes from the instance buffer, advancing
// once per copy drawn (divisor 1) instead of once per vertex.
GLuint makeArray(GLuint& buffer, const float* data, std::size_t bytes, GLuint instances) {
    GLuint array = 0;
    glGenVertexArrays(1, &array);
    glGenBuffers(1, &buffer);
    glBindVertexArray(array);
    glBindBuffer(GL_ARRAY_BUFFER, buffer);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(bytes), data, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, instances);
    for (GLuint location = 2; location <= 8; ++location) {
        glEnableVertexAttribArray(location);
        glVertexAttribDivisor(location, 1);
    }
    glBindVertexArray(0);
    return array;
}

// Points the per-box attributes of the bound vertex array at the records
// from `first` on: each group of boxes (solid cubes, see-through cylinders...)
// is one stretch of the instance buffer.
void pointAtInstances(std::size_t first) {
    const auto stride = static_cast<GLsizei>(sizeof(Instance));
    const auto at = [&](std::size_t floats) {
        return reinterpret_cast<void*>(first * sizeof(Instance) + floats * sizeof(float));
    };
    for (GLuint column = 0; column < 4; ++column) {
        glVertexAttribPointer(2 + column, 4, GL_FLOAT, GL_FALSE, stride, at(column * 4));
    }
    glVertexAttribPointer(6, 3, GL_FLOAT, GL_FALSE, stride, at(16));
    glVertexAttribPointer(7, 4, GL_FLOAT, GL_FALSE, stride, at(19));
    glVertexAttribPointer(8, 4, GL_FLOAT, GL_FALSE, stride, at(23));
}

} // namespace

Renderer::Renderer()
    : boxShader_(kBoxVertex, join({kBoxFragmentStart, kShadowFunction, kBoxFragmentMain})),
      floorShader_(kFloorVertex, join({kFloorFragmentStart, kShadowFunction, kFloorFragmentMain})),
      depthShader_(kDepthVertex, kDepthFragment) {
    glGenBuffers(1, &instanceBuffer_);
    const auto cube = cubeVertices();
    cubeArray_ = makeArray(cubeBuffer_, cube.data(), sizeof cube, instanceBuffer_);
    const auto cylinder = cylinderVertices();
    cylinderVertices_ = static_cast<GLsizei>(cylinder.size() / 6);
    cylinderArray_ = makeArray(cylinderBuffer_, cylinder.data(), cylinder.size() * sizeof(float), instanceBuffer_);

    constexpr float h = 60.0F; // the floor reaches 60 m in every direction
    const float floor[] = {-h, 0, -h, h, 0, h, h, 0, -h, -h, 0, -h, -h, 0, h, h, 0, h};
    glGenVertexArrays(1, &floorArray_);
    glGenBuffers(1, &floorBuffer_);
    glBindVertexArray(floorArray_);
    glBindBuffer(GL_ARRAY_BUFFER, floorBuffer_);
    glBufferData(GL_ARRAY_BUFFER, sizeof floor, floor, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    // The shadow map: a depth texture the sun's view is drawn into. Compare
    // mode makes a lookup answer "is this point nearer than what's stored?"
    // (0 or 1, filtered between texels), which is what sampler2DShadow reads.
    glGenTextures(1, &shadowMap_);
    glBindTexture(GL_TEXTURE_2D, shadowMap_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, kShadowSize, kShadowSize, 0, GL_DEPTH_COMPONENT, GL_FLOAT,
                 nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
    glGenFramebuffers(1, &shadowFramebuffer_);
    glBindFramebuffer(GL_FRAMEBUFFER, shadowFramebuffer_);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, shadowMap_, 0);
    glDrawBuffer(GL_NONE); // depth only: no colour to write
    glReadBuffer(GL_NONE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);

    glGenQueries(2, timer_);
}

Renderer::~Renderer() {
    glDeleteQueries(2, timer_);
    glDeleteFramebuffers(1, &shadowFramebuffer_);
    glDeleteTextures(1, &shadowMap_);
    glDeleteBuffers(1, &instanceBuffer_);
    glDeleteBuffers(1, &cubeBuffer_);
    glDeleteVertexArrays(1, &cubeArray_);
    glDeleteBuffers(1, &cylinderBuffer_);
    glDeleteVertexArrays(1, &cylinderArray_);
    glDeleteBuffers(1, &floorBuffer_);
    glDeleteVertexArrays(1, &floorArray_);
}

void Renderer::draw(const std::vector<DrawBox>& boxes, const OrbitCamera& camera, int width, int height) {
    const auto started = std::chrono::steady_clock::now();
    // The GPU's time for the frame before last: its query is done by now.
    const GLuint measuring = timer_[frame_ % 2];
    const GLuint finished = timer_[(frame_ + 1) % 2];
    if (frame_ > 0) {
        GLuint64 nanoseconds = 0;
        glGetQueryObjectui64v(finished, GL_QUERY_RESULT, &nanoseconds);
        stats_.gpuMilliseconds = static_cast<double>(nanoseconds) / 1e6;
    }
    glBeginQuery(GL_TIME_ELAPSED, measuring);
    stats_.drawCalls = 0;
    stats_.boxes = boxes.size();

    // The boxes as instance records, in four groups: solid cubes, solid
    // cylinders, then the see-through ones (drawn last, over the rest).
    std::vector<Instance> instances;
    instances.reserve(boxes.size());
    std::array<std::size_t, 5> start{}; // where each group starts, and the end
    const auto group = [](const DrawBox& box) { return (box.alpha < 1.0F ? 2 : 0) + (box.round ? 1 : 0); };
    for (int g = 0; g < 4; ++g) {
        start[static_cast<std::size_t>(g)] = instances.size();
        for (const auto& box : boxes) {
            if (group(box) == g) {
                instances.push_back(instanceOf(box));
            }
        }
    }
    start[4] = instances.size();
    glBindBuffer(GL_ARRAY_BUFFER, instanceBuffer_);
    // A new buffer every frame ("orphaning"): the GPU may still be drawing
    // from last frame's, and this way neither waits for the other.
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(instances.size() * sizeof(Instance)), instances.data(),
                 GL_STREAM_DRAW);
    const auto drawGroup = [&](int g) {
        const auto first = start[static_cast<std::size_t>(g)];
        const auto count = static_cast<GLsizei>(start[static_cast<std::size_t>(g) + 1] - first);
        if (count == 0) {
            return;
        }
        const bool round = g % 2 == 1;
        glBindVertexArray(round ? cylinderArray_ : cubeArray_);
        glBindBuffer(GL_ARRAY_BUFFER, instanceBuffer_);
        pointAtInstances(first);
        glDrawArraysInstanced(GL_TRIANGLES, 0, round ? cylinderVertices_ : 36, count);
        ++stats_.drawCalls;
    };

    // The sun's view: an orthographic box around the line (its boxes' extent,
    // with a margin), looking along the sunlight.
    glm::vec3 low(std::numeric_limits<float>::max());
    glm::vec3 high(-std::numeric_limits<float>::max());
    for (const auto& box : boxes) {
        const float reach = 0.5F * glm::length(box.size);
        low = glm::min(low, box.center - reach);
        high = glm::max(high, box.center + reach);
    }
    if (boxes.empty()) {
        low = glm::vec3(-1.0F);
        high = glm::vec3(1.0F);
    }
    low.y = 0.0F; // down to the floor, which receives the shadows
    const glm::vec3 middle = (low + high) * 0.5F;
    const float radius = std::max(1.0F, 0.5F * glm::length(high - low)) + 0.5F;
    const auto lightView = glm::lookAt(middle + kSun * radius * 2.0F, middle, glm::vec3(0.0F, 1.0F, 0.0F));
    const auto lightProjection = glm::ortho(-radius, radius, -radius, radius, 0.1F, radius * 4.0F);
    const auto lightSpace = lightProjection * lightView;

    if (shadows) {
        glBindFramebuffer(GL_FRAMEBUFFER, shadowFramebuffer_);
        glViewport(0, 0, kShadowSize, kShadowSize);
        glClear(GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);
        // Pushing the stored depths a little away from the sun is the other
        // half of fighting shadow acne (with the bias in the shader).
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(2.0F, 4.0F);
        depthShader_.use();
        depthShader_.set("uLightSpace", lightSpace);
        drawGroup(0);
        drawGroup(1);
        glDisable(GL_POLYGON_OFFSET_FILL);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    glViewport(0, 0, width, height);
    glClearColor(0.55F, 0.62F, 0.70F, 1.0F); // a pale sky
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, shadowMap_);

    const float aspect = height > 0 ? static_cast<float>(width) / static_cast<float>(height) : 1.0F;
    const auto viewProjection = camera.projection(aspect) * camera.view();

    floorShader_.use();
    floorShader_.set("uViewProjection", viewProjection);
    floorShader_.set("uLightSpace", lightSpace);
    floorShader_.set("uCamera", camera.position());
    floorShader_.set("uShadowMap", 0);
    floorShader_.set("uShadows", shadows ? 1.0F : 0.0F);
    glBindVertexArray(floorArray_);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    ++stats_.drawCalls;

    boxShader_.use();
    boxShader_.set("uViewProjection", viewProjection);
    boxShader_.set("uLightSpace", lightSpace);
    boxShader_.set("uShadowMap", 0);
    boxShader_.set("uShadows", shadows ? 1.0F : 0.0F);
    drawGroup(0);
    drawGroup(1);
    // See-through boxes, blended over what's behind them, without hiding
    // each other (no depth writes).
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    drawGroup(2);
    drawGroup(3);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glBindVertexArray(0);
    glBindTexture(GL_TEXTURE_2D, 0);
    glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
    glEndQuery(GL_TIME_ELAPSED);
    ++frame_;
    stats_.cpuMilliseconds =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
}

} // namespace fsim
