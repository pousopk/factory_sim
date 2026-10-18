#include "render/scene.hpp"
#include "sim/scene.hpp"
#include <algorithm>
#include <cmath>
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
    REQUIRE(boxes.size() == 8); // 7 conveyor parts and the item
    CHECK(boxes.back().size == glm::vec3(0.2F, 0.15F, 0.2F));
    CHECK(boxes.back().color == glm::vec3(1, 0, 0));
}

TEST_CASE("A photocell: housing, reflector, a beam at its height, and an LED lit by the output") {
    World world;
    ConveyorSpec spec;
    spec.beltHeight = 0.8F;
    world.addConveyor(spec);
    PhotocellSpec cell;
    cell.along = 1.0F;
    cell.height = 0.05F;
    world.addPhotocell(cell);
    auto& photocell = world.photocells().front();
    auto parts = photocellBoxes(photocell, world.conveyors().front());
    REQUIRE(parts.size() == 4);
    const auto& beam = parts[2];
    CHECK(beam.unlit);
    CHECK(beam.alpha < 1.0F);
    CHECK(beam.center.y == doctest::Approx(0.85F));
    CHECK(beam.center.x == doctest::Approx(1.0F));
    CHECK(beam.size.z > spec.width); // across the whole belt
    const auto ledOff = parts[3].color;
    photocell.output.set(0.1, true);
    parts = photocellBoxes(photocell, world.conveyors().front());
    CHECK(parts[3].color != ledOff);
}

TEST_CASE("A ray hits a box where it enters it, turned boxes included") {
    DrawBox box;
    box.center = {2.0F, 0.0F, 0.0F};
    box.size = {1.0F, 1.0F, 1.0F};
    const auto hit = hitDistance(box, {0.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F});
    REQUIRE(hit.has_value());
    CHECK(*hit == doctest::Approx(1.5F));
    CHECK_FALSE(hitDistance(box, {0.0F, 0.0F, 0.0F}, {-1.0F, 0.0F, 0.0F}).has_value()); // behind
    CHECK_FALSE(hitDistance(box, {0.0F, 2.0F, 0.0F}, {1.0F, 0.0F, 0.0F}).has_value());  // above
    // Turned 45 degrees around Y, its corner points at the ray: it's hit sooner.
    box.rotation = glm::angleAxis(std::numbers::pi_v<float> / 4.0F, glm::vec3(0.0F, 1.0F, 0.0F));
    CHECK(*hitDistance(box, {0.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F}) == doctest::Approx(2.0F - std::sqrt(0.5F)));
}

TEST_CASE("Clicking picks the nearest selectable thing; its parts are highlighted") {
    World world;
    world.addConveyor(ConveyorSpec{});
    PhotocellSpec cell;
    cell.along = 1.0F;
    world.addPhotocell(cell);
    world.addSpawner(SpawnerSpec{});
    const Pick photocell{Pick::Kind::Photocell, 0};
    const auto boxes = sceneBoxes(world, 1.0, photocell);
    // Straight down onto the belt, away from the photocell: the conveyor.
    CHECK(pickAt(boxes, {2.0F, 3.0F, 0.0F}, {0.0F, -1.0F, 0.0F}) == Pick{Pick::Kind::Conveyor, 0});
    // Down onto the photocell's housing, at the side of the belt.
    const float housingZ = -(0.2F + kFrameWidth + kHousingSize / 2.0F);
    CHECK(pickAt(boxes, {1.0F, 3.0F, housingZ}, {0.0F, -1.0F, 0.0F}) == photocell);
    // Down onto the spawner's gate, above the belt at its start.
    CHECK(pickAt(boxes, {0.2F, 3.0F, 0.0F}, {0.0F, -1.0F, 0.0F}) == Pick{Pick::Kind::Spawner, 0});
    CHECK_FALSE(pickAt(boxes, {9.0F, 3.0F, 9.0F}, {0.0F, -1.0F, 0.0F}).has_value()); // nothing there
    const auto highlighted = std::count_if(boxes.begin(), boxes.end(), [](const DrawBox& b) { return b.highlight; });
    CHECK(highlighted == 4); // the photocell's four parts
}

TEST_CASE("Every named device has a label just above it") {
    auto scene = sensorsDemo();
    addPusher(scene, 0);
    const auto world = buildWorld(scene);
    const auto labels = sceneLabels(*world);
    const auto find = [&](const std::string& text) {
        return std::find_if(labels.begin(), labels.end(), [&](const Label& l) { return l.text == text; });
    };
    for (const char* name : {"C1", "BI1", "BK1", "BL1", "BC1", "P1"}) {
        CAPTURE(name);
        CHECK(find(name) != labels.end());
    }
    // Above the belt, near the device along it.
    const auto laser = find("BL1");
    REQUIRE(laser != labels.end());
    CHECK(laser->pick == Pick{Pick::Kind::Laser, 0});
    CHECK(laser->position.y > scene.conveyors[0].spec.beltHeight + scene.lasers[0].mountHeight);
    CHECK(laser->position.x == doctest::Approx(scene.lasers[0].along));
    const auto conveyor = find("C1");
    CHECK(conveyor->position.x == doctest::Approx(scene.conveyors[0].spec.length / 2.0F));
}

TEST_CASE("An area camera's view: its field of view fills its picture, along the travel") {
    auto world = std::make_unique<World>(1);
    ConveyorSpec belt;
    belt.length = 4.0F;
    belt.heading = 0.7F; // any direction
    world->addConveyor(belt);
    CameraSpec spec;
    spec.along = 2.0F;
    spec.fieldOfView = 0.3F;
    spec.pixels = 1280;
    spec.pixelsAcross = 1024;
    spec.workingDistance = 0.5F;
    const auto& c = world->conveyors()[0];
    const auto view = cameraView(spec, c);
    const auto onPicture = [&](glm::vec3 p) {
        const auto clip = view.projection * view.view * glm::vec4(p, 1.0F);
        return glm::vec2(clip) / clip.w;
    };
    const auto middle = onPicture(c.beltPoint(2.0F));
    CHECK(middle.x == doctest::Approx(0.0F).epsilon(1e-4));
    CHECK(middle.y == doctest::Approx(0.0F).epsilon(1e-4));
    // The ends of the field of view, along the travel, at the picture's left and right.
    CHECK(onPicture(c.beltPoint(2.15F)).x == doctest::Approx(1.0F).epsilon(1e-3));
    CHECK(onPicture(c.beltPoint(1.85F)).x == doctest::Approx(-1.0F).epsilon(1e-3));
    // Across: 0.3 x 1024 / 1280 = 0.24 m, to the top and bottom.
    const auto left = leftOf(c.direction());
    CHECK(std::abs(onPicture(c.beltPoint(2.0F) + left * 0.12F).y) == doctest::Approx(1.0F).epsilon(1e-3));
    // Drawn at most 640 wide, the camera's shape kept.
    CHECK(view.width == 640);
    CHECK(view.height == 512);
}
