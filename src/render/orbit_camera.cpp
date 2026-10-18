#include "render/orbit_camera.hpp"
#include <algorithm>
#include <cmath>
#include <glm/gtc/matrix_transform.hpp>
#include <numbers>

namespace fsim {

namespace {

constexpr float kMaxPitch = 1.5F; // just under 90 degrees: looking straight down flips the view

} // namespace

void OrbitCamera::rotate(float dx, float dy, float windowHeight) {
    const float perPixel = std::numbers::pi_v<float> / std::max(windowHeight, 1.0F);
    yaw -= dx * perPixel;
    pitch = std::clamp(pitch + dy * perPixel, -kMaxPitch, kMaxPitch);
}

void OrbitCamera::pan(float dx, float dy, float windowHeight) {
    // At the target's distance, the window's height covers this many metres.
    const float visibleHeight = 2.0F * distance * std::tan(kFieldOfView / 2.0F);
    const float perPixel = visibleHeight / std::max(windowHeight, 1.0F);
    const auto forward = glm::normalize(target - position());
    const auto right = glm::normalize(glm::cross(forward, glm::vec3(0.0F, 1.0F, 0.0F)));
    const auto up = glm::cross(right, forward);
    target += (-right * dx + up * dy) * perPixel;
}

void OrbitCamera::zoom(float steps) {
    distance = std::clamp(distance * std::pow(0.9F, steps), kMinDistance, kMaxDistance);
}

glm::vec3 OrbitCamera::position() const {
    const glm::vec3 offset{std::sin(yaw) * std::cos(pitch), std::sin(pitch), std::cos(yaw) * std::cos(pitch)};
    return target + offset * distance;
}

glm::mat4 OrbitCamera::view() const {
    return glm::lookAt(position(), target, glm::vec3(0.0F, 1.0F, 0.0F));
}

glm::mat4 OrbitCamera::projection(float aspect) const {
    return glm::perspective(kFieldOfView, aspect, 0.02F, 200.0F);
}

OrbitCamera::Ray OrbitCamera::rayThrough(float x, float y, float width, float height) const {
    // The pixel in normalized device coordinates (-1..1, y up), then back
    // through the projection and view to two points: on the near plane and
    // on the far plane. The ray goes from one through the other.
    const glm::vec2 ndc{2.0F * x / std::max(width, 1.0F) - 1.0F, 1.0F - 2.0F * y / std::max(height, 1.0F)};
    const auto inverse = glm::inverse(projection(width / std::max(height, 1.0F)) * view());
    auto nearPoint = inverse * glm::vec4(ndc, -1.0F, 1.0F);
    auto farPoint = inverse * glm::vec4(ndc, 1.0F, 1.0F);
    nearPoint /= nearPoint.w;
    farPoint /= farPoint.w;
    return Ray{glm::vec3(nearPoint), glm::normalize(glm::vec3(farPoint - nearPoint))};
}

} // namespace fsim
