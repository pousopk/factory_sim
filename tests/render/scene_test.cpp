#include "render/scene.hpp"
#include <algorithm>
#include <doctest/doctest.h>
#include <numbers>

using namespace fsim;

// The scene is plain geometry, so it's checked like the simulation: where
// each part of a conveyor ends up.

namespace {

// The lowest and highest y a box reaches (all the boxes here are upright).
float bottom(const DrawBox& box) {
    return box.center.y - box.size.y / 2.0F;
}
float top(const DrawBox& box) {
    return box.center.y + box.size.y / 2.0F;
}

} // namespace

TEST_CASE("A conveyor is a belt, two side frames and four legs") {
    World world;
    ConveyorSpec spec;
    spec.length = 3.0F;
    spec.beltHeight = 0.8F;
    world.addConveyor(spec);
    const auto parts = conveyorBoxes(world.conveyors().front());
    REQUIRE(parts.size() == 7);

    const auto& belt = parts[0];
    CHECK(top(belt) == doctest::Approx(0.8F)); // the belt surface is where the physics puts it
    CHECK(belt.size.x == doctest::Approx(3.0F));
    CHECK(belt.center.x == doctest::Approx(1.5F));
    CHECK(belt.stripeSpacing > 0.0F);

    for (std::size_t leg = 3; leg < 7; ++leg) {
        CAPTURE(leg);
        CHECK(bottom(parts[leg]) == doctest::Approx(0.0F));          // standing on the floor
        CHECK(top(parts[leg]) == doctest::Approx(bottom(parts[1]))); // up to the frame
    }
    CHECK(top(parts[1]) > top(belt)); // the frames stand a little above the belt, as guides
}

TEST_CASE("The stripes move with the belt surface") {
    World world;
    ConveyorSpec spec;
    spec.targetSpeed = 0.5F;
    spec.acceleration = 100.0F;
    world.addConveyor(spec);
    world.run(0.1);
    const auto before = conveyorBoxes(world.conveyors().front())[0].stripeOffset;
    world.run(0.1); // 5 cm further
    const auto after = conveyorBoxes(world.conveyors().front())[0].stripeOffset;
    CHECK(after - before == doctest::Approx(0.05F).epsilon(0.01));
    CHECK(after < kStripeSpacing); // kept within one spacing, so it never loses precision
}

TEST_CASE("A turned conveyor's parts turn with it") {
    World world;
    ConveyorSpec spec;
    spec.length = 2.0F;
    spec.heading = std::numbers::pi_v<float> / 2.0F; // towards -Z
    world.addConveyor(spec);
    const auto belt = conveyorBoxes(world.conveyors().front())[0];
    CHECK(belt.center.x == doctest::Approx(0.0F).epsilon(0.001));
    CHECK(belt.center.z == doctest::Approx(-1.0F));
    // The box's own x axis (its length) points along the travel.
    const auto alongLength = belt.rotation * glm::vec3(1.0F, 0.0F, 0.0F);
    CHECK(alongLength.z == doctest::Approx(-1.0F));
}

TEST_CASE("The scene has every conveyor part and every item") {
    World world;
    world.addConveyor(ConveyorSpec{});
    world.spawnItem({0.5F, 1.0F, 0.0F}, {0.2F, 0.15F, 0.2F}, {1, 0, 0});
    const auto boxes = sceneBoxes(world);
    REQUIRE(boxes.size() == 8);
    CHECK(boxes.back().size == glm::vec3(0.2F, 0.15F, 0.2F));
    CHECK(boxes.back().color == glm::vec3(1, 0, 0));
}
