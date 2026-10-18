#include "render/orbit_camera.hpp"
#include <doctest/doctest.h>
#include <glm/gtc/matrix_transform.hpp>

using namespace fsim;

TEST_CASE("The camera stays at its distance from the target, looking at it") {
    OrbitCamera camera;
    camera.rotate(150.0F, -80.0F, 900.0F);
    CHECK(glm::length(camera.position() - camera.target) == doctest::Approx(camera.distance));
    // The target is straight ahead: in the middle of the view.
    const auto seen = camera.projection(1.5F) * camera.view() * glm::vec4(camera.target, 1.0F);
    CHECK(seen.x / seen.w == doctest::Approx(0.0F).epsilon(0.0001));
    CHECK(seen.y / seen.w == doctest::Approx(0.0F).epsilon(0.0001));
}

TEST_CASE("Turning: a drag across the window's height is half a turn, and the pitch is limited") {
    OrbitCamera camera;
    const float yaw = camera.yaw;
    camera.rotate(-900.0F, 0.0F, 900.0F);
    CHECK(camera.yaw - yaw == doctest::Approx(3.14159F));
    camera.rotate(0.0F, 100000.0F, 900.0F); // far past straight up
    CHECK(camera.pitch < 1.571F);
    CHECK(camera.position().y > camera.target.y); // looking down, not flipped over
}

TEST_CASE("Zooming brings it closer, within limits") {
    OrbitCamera camera;
    camera.distance = 10.0F;
    camera.zoom(1.0F);
    CHECK(camera.distance == doctest::Approx(9.0F));
    camera.zoom(1000.0F);
    CHECK(camera.distance == doctest::Approx(OrbitCamera::kMinDistance));
}

TEST_CASE("Panning moves the target in the view's plane, not towards it") {
    OrbitCamera camera;
    const auto before = camera.target;
    const auto forward = glm::normalize(camera.target - camera.position());
    camera.pan(120.0F, -60.0F, 900.0F);
    const auto moved = camera.target - before;
    CHECK(glm::length(moved) > 0.0F);
    CHECK(glm::dot(moved, forward) == doctest::Approx(0.0F).epsilon(0.0001));
}

TEST_CASE("The ray through the middle of the window points at the target") {
    OrbitCamera camera;
    const auto ray = camera.rayThrough(700.0F, 450.0F, 1400.0F, 900.0F);
    const auto toTarget = glm::normalize(camera.target - ray.origin);
    CHECK(glm::dot(ray.direction, toTarget) == doctest::Approx(1.0F).epsilon(1e-5));
    // A pixel to the right gives a ray that goes to the right of the target.
    const auto right = camera.rayThrough(900.0F, 450.0F, 1400.0F, 900.0F);
    const auto side = glm::cross(toTarget, glm::vec3(0.0F, 1.0F, 0.0F));
    CHECK(glm::dot(right.direction, side) > 0.0F);
}
