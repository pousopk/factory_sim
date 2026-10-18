#pragma once
#include "render/gl.hpp"
#include "render/orbit_camera.hpp"
#include "render/scene.hpp"
#include "render/shader.hpp"
#include <vector>

// Draws the scene with OpenGL 3.3: a floor with a metre grid, and boxes
// lit by a sun (with shadows) and a soft sky light. All the boxes of a kind
// are drawn in one call (instancing), from a buffer of their placements
// filled once per frame. It needs a current OpenGL context, so it's created
// after the window.
namespace fsim {

class Renderer {
public:
    Renderer();
    ~Renderer();
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    // Draws into the current framebuffer of `width` x `height` pixels.
    void draw(const std::vector<DrawBox>& boxes, const OrbitCamera& camera, int width, int height);

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
    Stats stats_;
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
