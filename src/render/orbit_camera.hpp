#pragma once
#include <glm/glm.hpp>

// A camera that orbits around a point, as in CAD programs: drag to turn
// around the point, drag with the other button to move the point, scroll
// to come closer or go further. It's only maths (no OpenGL, no window), so
// it's tested on its own.
namespace fsim {

class OrbitCamera {
public:
    glm::vec3 target{1.5F, 0.8F, 0.0F}; // the point it looks at and turns around
    float yaw = 0.6F;                   // radians around the vertical, 0 = looking along -Z from +Z
    float pitch = 0.45F;                // radians above the horizontal
    float distance = 4.5F;

    // Mouse movements in pixels; a drag across the whole window height turns
    // it by about 180 degrees, whatever the window's size.
    void rotate(float dx, float dy, float windowHeight);
    // Moves the target sideways and up/down in the view, so the point under
    // the mouse follows it.
    void pan(float dx, float dy, float windowHeight);
    // Scroll steps: each one brings it 10 % closer (or further).
    void zoom(float steps);

    glm::vec3 position() const; // where the camera is
    glm::mat4 view() const;
    glm::mat4 projection(float aspect) const;

    // The ray from the camera through a pixel (x right, y down, in a window
    // of width x height): its origin and unit direction. For clicking things.
    struct Ray {
        glm::vec3 origin;
        glm::vec3 direction;
    };
    Ray rayThrough(float x, float y, float width, float height) const;

    static constexpr float kFieldOfView = 0.8F; // vertical, radians (about 46 degrees)
    static constexpr float kMinDistance = 0.3F;
    static constexpr float kMaxDistance = 60.0F;
};

} // namespace fsim
