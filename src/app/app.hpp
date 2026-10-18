#pragma once
#include "render/orbit_camera.hpp"
#include "render/renderer.hpp"
#include "sim/world.hpp"
#include <memory>

// The application: the simulated world, the camera, and the panels to
// change them. It draws one frame at a time into the current window (main
// owns the window and the ImGui setup).
//
// The world runs in simulated time, in 1 ms steps; each frame advances it
// by the real time since the last frame, times the time scale (0 when
// paused). So 0.1x shows a 2 m/s belt moving 0.2 mm per real millisecond,
// slow enough to watch, with exactly the same physics.
namespace fsim {

class App {
public:
    App(); // needs a current OpenGL context (for the renderer)

    // One frame: advances the world by `realSeconds` of real time (scaled),
    // handles the mouse, draws the scene and the panels.
    void frame(double realSeconds, int width, int height);

    World& world() { return *world_; }

private:
    void handleMouse(int height);
    void drawPanels();
    void resetScene();

    std::unique_ptr<World> world_;
    OrbitCamera camera_;
    Renderer renderer_;
    bool paused_ = false;
    float timeScale_ = 1.0F;
    double frameSeconds_ = 0.0; // the last frame's real duration, for the panel
    int stepsLastFrame_ = 0;
};

} // namespace fsim
