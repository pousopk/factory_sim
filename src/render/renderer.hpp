#pragma once
#include "render/gl.hpp"
#include "render/orbit_camera.hpp"
#include "render/scene.hpp"
#include "render/shader.hpp"
#include "sim/camera.hpp"
#include <map>
#include <vector>

// Draws the scene with OpenGL 3.3: a floor with a metre grid, and boxes
// lit by a sun (with shadows) and a soft sky light. All the boxes of a kind
// are drawn in one call (instancing), from a buffer of their placements
// filled once per frame. It needs a current OpenGL context, so it's created
// after the window.
namespace fsim {

// An image drawn offscreen, for an ImGui window to show: what a camera sees.
// It owns its OpenGL objects (textures and framebuffers), so it moves but
// doesn't copy, like Shader.
class Picture {
public:
    Picture() = default;
    ~Picture();
    Picture(Picture&& other) noexcept;
    Picture& operator=(Picture&& other) noexcept;
    Picture(const Picture&) = delete;
    Picture& operator=(const Picture&) = delete;

    GLuint texture() const { return image_; } // the finished image (RGBA, 8 bits)
    int width() const { return width_; }
    int height() const { return height_; }

private:
    friend class Renderer;
    void resize(int width, int height); // (re)creates the targets at that size
    void release();
    int width_ = 0;
    int height_ = 0;
    GLuint frame_ = 0; // one render: colour texture and depth
    GLuint frameColor_ = 0;
    GLuint frameDepth_ = 0;
    GLuint sum_ = 0; // the renders averaged, in floats
    GLuint sumColor_ = 0;
    GLuint finished_ = 0;
    GLuint image_ = 0;
};

// A line-scan camera's lines, uploaded as a texture to show.
class LineTexture {
public:
    LineTexture() = default;
    ~LineTexture();
    LineTexture(const LineTexture&) = delete;
    LineTexture& operator=(const LineTexture&) = delete;
    void upload(const LineImage& image); // oldest line first, at the top
    GLuint texture() const { return texture_; }
    int width() const { return width_; }
    int height() const { return height_; }

private:
    GLuint texture_ = 0;
    int width_ = 0;
    int height_ = 0;
};

class Renderer {
public:
    Renderer();
    ~Renderer();
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    // Draws into the current framebuffer of `width` x `height` pixels.
    // In the window's framebuffer, the rectangle at (x, y) (from its bottom
    // left, as OpenGL counts) of `width` x `height` pixels.
    void draw(const std::vector<DrawBox>& boxes, const OrbitCamera& camera, int x, int y, int width, int height);

    // Draws what a camera sees into `picture` (sized `width` x `height`): the
    // scene once for each of `moments` (the boxes as they were at moments
    // during the exposure), averaged, which blurs what moved, then noise of
    // `noise` grey levels (standard deviation) added, from `seed`.
    void drawPicture(Picture& picture, const std::vector<std::vector<DrawBox>>& moments,
                     const glm::mat4& viewProjection, glm::vec3 eye, int width, int height, float noise, unsigned seed);

    // What the last frames cost: the CPU's time to hand the frame to OpenGL,
    // the GPU's time to draw it (measured with a timer query, read a frame
    // later so it never waits), and how many draw calls it took.
    struct Stats {
        double cpuMilliseconds = 0.0;
        double gpuMilliseconds = 0.0;
        int drawCalls = 0;
        std::size_t boxes = 0;
    };
    const Stats& stats() const { return stats_; }

    bool shadows = true; // the sun's shadows (a depth pass from the sun, then a lookup per pixel)

private:
    // The scene into `framebuffer` (0: the window), with its shadow pass.
    void render(const std::vector<DrawBox>& boxes, const glm::mat4& viewProjection, glm::vec3 eye, GLuint framebuffer,
                int width, int height, int x = 0, int y = 0);

    // An item shape's surface on the GPU, drawn instanced like the cube.
    struct GpuMesh {
        GLuint array = 0;
        GLuint buffer = 0;
        GLsizei vertices = 0;
    };
    GpuMesh upload(const ItemGeometry& geometry);
    // Each item shape's mesh, uploaded the first time it's drawn, by its id
    // (a few dozen at most: shapes are shared, sizes rounded to millimetres).
    std::map<std::uint64_t, GpuMesh> meshes_;

    Stats stats_;
    Shader accumulate_;        // adds one render, weighted, to the sum
    Shader finish_;            // the sum, plus noise, into the image
    GLuint emptyArray_ = 0;    // for the full-screen triangle (its corners come from gl_VertexID)
    GLuint timer_[2] = {0, 0}; // two queries: one being measured, one being read
    int frame_ = 0;
    Shader boxShader_;
    Shader floorShader_;
    Shader depthShader_;        // the shadow pass: depth only, from the sun
    GLuint instanceBuffer_ = 0; // one record per box, rewritten every frame
    GLuint shadowMap_ = 0;      // depth texture
    GLuint shadowFramebuffer_ = 0;
    GLuint cubeArray_ = 0; // vertex array: a unit cube with normals
    GLuint cubeBuffer_ = 0;
    GLuint cylinderArray_ = 0; // a cylinder, for tins
    GLuint cylinderBuffer_ = 0;
    GLsizei cylinderVertices_ = 0;
    GLuint floorArray_ = 0; // vertex array: one big square at y = 0
    GLuint floorBuffer_ = 0;
};

} // namespace fsim
