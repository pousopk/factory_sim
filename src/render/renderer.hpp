#pragma once
#include "render/gl.hpp"
#include "render/orbit_camera.hpp"
#include "render/scene.hpp"
#include "render/shader.hpp"
#include <vector>

// Draws the scene with OpenGL 3.3: a floor with a metre grid, and boxes
// lit by a sun and a soft sky light. It needs a current OpenGL context, so
// it's created after the window.
namespace fsim {

class Renderer {
public:
    Renderer();
    ~Renderer();
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    // Draws into the current framebuffer of `width` x `height` pixels.
    void draw(const std::vector<DrawBox>& boxes, const OrbitCamera& camera, int width, int height);

private:
    Shader boxShader_;
    Shader floorShader_;
    GLuint cubeArray_ = 0; // vertex array: a unit cube with normals
    GLuint cubeBuffer_ = 0;
    GLuint floorArray_ = 0; // vertex array: one big square at y = 0
    GLuint floorBuffer_ = 0;
};

} // namespace fsim
