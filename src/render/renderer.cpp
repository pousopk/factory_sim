#include "render/renderer.hpp"
#include <array>
#include <glm/gtc/matrix_transform.hpp>

namespace fsim {

namespace {

// Every box is this cube (from -0.5 to 0.5 on each axis), scaled, turned and
// moved into place by its model matrix.
constexpr const char* kBoxVertex = R"(#version 330 core
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;
uniform mat4 uModel;     // place, turn, then the box's size
uniform mat4 uRotation;  // just the turn, for the normals
uniform mat4 uViewProjection;
uniform vec3 uSize;
out vec3 vNormal;
out vec3 vLocal;         // the position on the box, in metres from its centre, in its own axes
out vec3 vLocalNormal;
void main() {
    vNormal = mat3(uRotation) * aNormal;
    vLocal = aPosition * uSize;
    vLocalNormal = aNormal;
    gl_Position = uViewProjection * uModel * vec4(aPosition, 1.0);
}
)";

constexpr const char* kBoxFragment = R"(#version 330 core
in vec3 vNormal;
in vec3 vLocal;
in vec3 vLocalNormal;
uniform vec3 uColor;
uniform vec3 uSize;
uniform float uStripeSpacing; // 0: no stripes
uniform float uStripeOffset;
uniform float uAlpha;
uniform float uUnlit;         // 1: a light source, not shaded
uniform float uHighlight;     // 1: part of the selection
out vec4 fragColor;

const vec3 kSun = normalize(vec3(0.4, 1.0, 0.3));

void main() {
    if (uUnlit > 0.5) {
        fragColor = vec4(uColor, uAlpha);
        return;
    }
    vec3 color = uColor;
    // A moving belt: light stripes across its top, moving with the surface.
    if (uStripeSpacing > 0.0 && vLocalNormal.y > 0.5) {
        float s = fract((vLocal.x + 0.5 * uSize.x - uStripeOffset) / uStripeSpacing);
        if (s < 0.06) color = mix(color, vec3(0.75), 0.6);
    }
    // Darker edges, so boxes of the same colour stay readable: the distance
    // to the nearest edge of this face, in metres.
    vec3 toEdge = 0.5 * uSize - abs(vLocal);
    vec3 n = abs(vLocalNormal);
    float edge = n.x > 0.5 ? min(toEdge.y, toEdge.z) : n.y > 0.5 ? min(toEdge.x, toEdge.z) : min(toEdge.x, toEdge.y);
    color *= mix(0.55, 1.0, smoothstep(0.0, 0.006, edge));
    // Light: a sun, plus a sky light that's brighter from above.
    vec3 normal = normalize(vNormal);
    float sun = max(dot(normal, kSun), 0.0);
    float sky = 0.35 + 0.25 * normal.y;
    color *= sky + 0.75 * sun;
    if (uHighlight > 0.5) color = mix(color, vec3(0.35, 0.65, 1.0), 0.45); // a blue tint
    fragColor = vec4(color, uAlpha);
}
)";

// The floor: a grid of lines every metre and every 10 cm, fading with distance.
constexpr const char* kFloorVertex = R"(#version 330 core
layout(location = 0) in vec3 aPosition;
uniform mat4 uViewProjection;
out vec3 vWorld;
void main() {
    vWorld = aPosition;
    gl_Position = uViewProjection * vec4(aPosition, 1.0);
}
)";

constexpr const char* kFloorFragment = R"(#version 330 core
in vec3 vWorld;
uniform vec3 uCamera;
out vec4 fragColor;

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
    fragColor = vec4(color, 1.0);
}
)";

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

} // namespace

Renderer::Renderer() : boxShader_(kBoxVertex, kBoxFragment), floorShader_(kFloorVertex, kFloorFragment) {
    const auto cube = cubeVertices();
    glGenVertexArrays(1, &cubeArray_);
    glGenBuffers(1, &cubeBuffer_);
    glBindVertexArray(cubeArray_);
    glBindBuffer(GL_ARRAY_BUFFER, cubeBuffer_);
    glBufferData(GL_ARRAY_BUFFER, sizeof cube, cube.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

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
}

Renderer::~Renderer() {
    glDeleteBuffers(1, &cubeBuffer_);
    glDeleteVertexArrays(1, &cubeArray_);
    glDeleteBuffers(1, &floorBuffer_);
    glDeleteVertexArrays(1, &floorArray_);
}

void Renderer::draw(const std::vector<DrawBox>& boxes, const OrbitCamera& camera, int width, int height) {
    glViewport(0, 0, width, height);
    glClearColor(0.55F, 0.62F, 0.70F, 1.0F); // a pale sky
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    const float aspect = height > 0 ? static_cast<float>(width) / static_cast<float>(height) : 1.0F;
    const auto viewProjection = camera.projection(aspect) * camera.view();

    floorShader_.use();
    floorShader_.set("uViewProjection", viewProjection);
    floorShader_.set("uCamera", camera.position());
    glBindVertexArray(floorArray_);
    glDrawArrays(GL_TRIANGLES, 0, 6);

    boxShader_.use();
    boxShader_.set("uViewProjection", viewProjection);
    glBindVertexArray(cubeArray_);
    // Solid boxes first; see-through ones after, blended over what's behind
    // them, without hiding each other (no depth writes).
    const auto drawBox = [&](const DrawBox& box) {
        const auto rotation = glm::mat4_cast(box.rotation);
        const auto model =
            glm::translate(glm::mat4(1.0F), box.center) * rotation * glm::scale(glm::mat4(1.0F), box.size);
        boxShader_.set("uModel", model);
        boxShader_.set("uRotation", rotation);
        boxShader_.set("uSize", box.size);
        boxShader_.set("uColor", box.color);
        boxShader_.set("uStripeSpacing", box.stripeSpacing);
        boxShader_.set("uStripeOffset", box.stripeOffset);
        boxShader_.set("uAlpha", box.alpha);
        boxShader_.set("uUnlit", box.unlit ? 1.0F : 0.0F);
        boxShader_.set("uHighlight", box.highlight ? 1.0F : 0.0F);
        glDrawArrays(GL_TRIANGLES, 0, 36);
    };
    for (const auto& box : boxes) {
        if (box.alpha >= 1.0F) {
            drawBox(box);
        }
    }
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    for (const auto& box : boxes) {
        if (box.alpha < 1.0F) {
            drawBox(box);
        }
    }
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glBindVertexArray(0);
    glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
}

} // namespace fsim
