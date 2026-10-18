#include "sim/scene.hpp"
#include <doctest/doctest.h>
#include <numbers>

using namespace fsim;

// Pneumatic actuators, bins, guides and transfers: the timing on its own,
// then real boxes on real belts.

namespace {

ConveyorSpec belt(float length = 4.0F) {
    ConveyorSpec spec;
    spec.length = length;
    spec.targetSpeed = 0.5F;
    spec.acceleration = 1000.0F;
    return spec;
}

ItemId boxAt(World& world, std::size_t conveyor, float along) {
    const auto& c = world.conveyors()[conveyor];
    return world.spawnItem(c.beltPoint(along) + glm::vec3(0.0F, 0.076F, 0.0F), {0.2F, 0.15F, 0.2F}, {1, 1, 1},
                           c.spec.heading);
}

std::optional<ItemView> find(const World& world, ItemId id) {
    for (const auto& item : world.items()) {
        if (item.id == id) {
            return item;
        }
    }
    return std::nullopt;
}

// Runs the world in steps, like the app does.
void steps(World& world, int count) {
    for (int i = 0; i < count; ++i) {
        world.step();
    }
}

} // namespace

TEST_CASE("A stroke: dead time, then steady travel, end sensors with exact times") {
    Stroke stroke(StrokeSpec{0.015, 0.1, 0.2});
    CHECK(stroke.retracted().value());
    stroke.command(0.0, true);
    double t = 0.0;
    for (int i = 0; i < 200; ++i, t += 0.001) {
        stroke.update(t, t + 0.001);
    }
    CHECK(stroke.position() == 1.0);
    // It left the retracted end after the dead time, and arrived 100 ms later.
    CHECK(stroke.retracted().edges().back().time == doctest::Approx(0.015));
    CHECK(stroke.extended().edges().back().time == doctest::Approx(0.115));

    stroke.command(0.2, false);
    for (int i = 0; i < 300; ++i, t += 0.001) {
        stroke.update(t, t + 0.001);
    }
    CHECK(stroke.position() == 0.0);
    CHECK(stroke.extended().edges().back().time == doctest::Approx(0.215));  // left extended after the dead time
    CHECK(stroke.retracted().edges().back().time == doctest::Approx(0.415)); // retracting takes 200 ms
}

TEST_CASE("A stroke reversed halfway turns back from where it is") {
    Stroke stroke(StrokeSpec{0.0, 0.1, 0.1});
    stroke.command(0.0, true);
    double t = 0.0;
    for (int i = 0; i < 50; ++i, t += 0.001) {
        stroke.update(t, t + 0.001);
    }
    CHECK(stroke.position() == doctest::Approx(0.5));
    CHECK_FALSE(stroke.extended().value());
    stroke.command(t, false);
    for (int i = 0; i < 50; ++i, t += 0.001) {
        stroke.update(t, t + 0.001);
    }
    CHECK(stroke.position() == doctest::Approx(0.0));
    CHECK(stroke.retracted().value());
    CHECK(stroke.extended().edges().empty()); // it never got there
}

TEST_CASE("A pusher pushes a box off the belt into the bin, which counts it") {
    World world;
    world.addConveyor(belt());
    PusherSpec pusher;
    pusher.along = 1.0F;
    pusher.side = Side::Left;
    world.addPusher(pusher);
    BinSpec bin;
    bin.along = 1.0F;
    bin.side = Side::Right; // across the belt from the pusher
    world.addBin(bin);
    world.conveyors()[0].spec.targetSpeed = 0.0F; // a still belt: the box waits in front of the paddle
    const auto id = boxAt(world, 0, 1.0F);
    steps(world, 200);
    world.pushers()[0].stroke.command(world.time(), true);
    steps(world, 1500);
    CHECK(world.bins()[0].count == 1);
    const auto box = find(world, id);
    REQUIRE(box.has_value());
    CHECK(box->position.y < 0.5F); // on the floor of the bin, not on the belt
    CHECK(world.pushers()[0].stroke.extended().value());
}

TEST_CASE("A diverter swung out guides boxes off the far side; swung back, they pass") {
    for (const bool active : {false, true}) {
        CAPTURE(active);
        World world;
        world.addConveyor(belt());
        DiverterSpec diverter;
        diverter.along = 1.0F;
        diverter.side = Side::Left;
        world.addDiverter(diverter);
        if (active) {
            world.diverters()[0].stroke.command(0.0, true);
        }
        steps(world, 400); // swung out (or not) before the box comes
        const auto id = boxAt(world, 0, 0.3F);
        steps(world, active ? 4000 : 8000); // (at 0.5 m/s, 8 s to reach the end)
        const auto box = find(world, id);
        if (active) {
            // Off the right side of the belt (towards +Z for a belt along +X), and down.
            REQUIRE(box.has_value());
            CHECK(box->position.z > 0.25F);
            CHECK(box->position.y < 0.5F);
        } else {
            // Straight past, off the end.
            REQUIRE(box.has_value());
            CHECK(box->position.x > 3.5F);
        }
    }
}

TEST_CASE("A stopper holds boxes back while the belt slides under them, and lets them go") {
    World world;
    world.addConveyor(belt());
    StopperSpec stopper;
    stopper.along = 2.0F;
    world.addStopper(stopper);
    world.stoppers()[0].stroke.command(0.0, true);
    const auto id = boxAt(world, 0, 0.5F);
    steps(world, 5000);
    auto box = find(world, id);
    REQUIRE(box.has_value());
    CHECK(box->position.x + 0.1F == doctest::Approx(2.0F - 0.01F).epsilon(0.01)); // its front against the blade
    CHECK(std::abs(box->velocity.x) < 0.01F);                                     // standing still on a moving belt
    world.stoppers()[0].stroke.command(world.time(), false);
    steps(world, 1000);
    box = find(world, id);
    REQUIRE(box.has_value());
    CHECK(box->position.x > 2.2F); // on its way again
}

TEST_CASE("Straight on: a box crosses the gap onto the next conveyor") {
    auto scene = Scene{};
    PlacedConveyor first;
    first.spec = belt(2.0F);
    scene.conveyors.push_back(first);
    addConveyorAfter(scene, 0, Turn::Straight);
    auto world = buildWorld(scene);
    const auto id = boxAt(*world, 0, 0.5F);
    world->run(5.0);
    const auto box = find(*world, id);
    REQUIRE(box.has_value());
    CHECK(box->position.x > 2.5F);                                   // on the second conveyor
    CHECK(box->position.y == doctest::Approx(0.875F).epsilon(0.01)); // still on a belt
}

TEST_CASE("A turned conveyor gets a guide on its far side, and the box turns onto it") {
    auto scene = Scene{};
    PlacedConveyor first;
    first.spec = belt(2.0F);
    scene.conveyors.push_back(first);
    const auto turned = addConveyorAfter(scene, 0, Turn::Left);
    CHECK(scene.conveyors[turned].spec.guideRight); // the side away from where boxes come from
    CHECK_FALSE(scene.conveyors[turned].spec.guideLeft);
    auto world = buildWorld(scene);
    const auto id = boxAt(*world, 0, 1.0F);
    world->run(5.0); // onto the turned belt after about 2 s; off its end after about 6
    const auto box = find(*world, id);
    REQUIRE(box.has_value());
    CHECK(box->position.y > 0.8F);  // still on a belt
    CHECK(box->position.z < -0.5F); // carried along the turned conveyor, towards -Z
}
