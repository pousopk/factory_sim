#include "sim/world.hpp"
#include <cmath>
#include <doctest/doctest.h>
#include <numbers>

using namespace fsim;

// These tests run the simulation without any graphics. The physics is
// deterministic (single-threaded, fixed 1 ms steps), so results are exact
// from run to run; the tolerances are about physics, not about luck.

namespace {

ConveyorSpec belt(float speed) {
    ConveyorSpec spec;
    spec.length = 4.0F;
    spec.targetSpeed = speed;
    spec.acceleration = 100.0F; // effectively instant, unless a test is about ramps
    return spec;
}

ItemId boxOn(World& world, std::size_t conveyor, float along) {
    const auto& c = world.conveyors()[conveyor];
    return world.spawnItem(c.beltPoint(along) + glm::vec3(0.0F, 0.076F, 0.0F), {0.2F, 0.15F, 0.2F}, {1, 1, 1},
                           c.spec.heading);
}

ItemView view(const World& world, ItemId id) {
    for (const auto& item : world.items()) {
        if (item.id == id)
            return item;
    }
    FAIL("no item " << id);
    return {};
}

} // namespace

TEST_CASE("A box on a moving belt is carried at the belt's speed") {
    World world;
    world.addConveyor(belt(0.5F));
    const auto id = boxOn(world, 0, 0.5F);
    const auto start = view(world, id).position;

    world.run(1.0);
    const auto after = view(world, id);
    CHECK(after.velocity.x == doctest::Approx(0.5).epsilon(0.02));
    // It needs a moment to catch up with the belt (friction), so a little less than 0.5 m in 1 s.
    CHECK(after.position.x - start.x == doctest::Approx(0.5).epsilon(0.05));
    CHECK(std::abs(after.position.z - start.z) < 0.001F);            // straight along the belt
    CHECK(after.position.y == doctest::Approx(0.875).epsilon(0.01)); // still sitting on it
}

TEST_CASE("The heading turns the direction of travel") {
    World world;
    auto spec = belt(0.5F);
    spec.heading = std::numbers::pi_v<float> / 2.0F; // 90 degrees: towards -Z
    world.addConveyor(spec);
    const auto id = boxOn(world, 0, 0.5F);
    world.run(1.0);
    const auto v = view(world, id).velocity;
    CHECK(v.z == doctest::Approx(-0.5).epsilon(0.02));
    CHECK(std::abs(v.x) < 0.01F);
}

TEST_CASE("A box on a stopped belt stays where it is") {
    World world;
    world.addConveyor(belt(0.0F));
    const auto id = boxOn(world, 0, 1.0F);
    world.run(0.2); // let it settle onto the belt
    const auto settled = view(world, id).position;
    world.run(1.0);
    CHECK(glm::length(view(world, id).position - settled) < 0.001F);
}

TEST_CASE("Belt speed ramps at the configured acceleration, like a VFD") {
    World world;
    auto spec = belt(1.0F);
    spec.acceleration = 2.0F; // 0 -> 1 m/s in 0.5 s
    world.addConveyor(spec);
    world.run(0.25);
    CHECK(world.conveyors()[0].speed == doctest::Approx(0.5).epsilon(0.01));
    world.run(0.5);
    CHECK(world.conveyors()[0].speed == doctest::Approx(1.0));
    world.conveyors()[0].spec.targetSpeed = 0.0F; // and ramps down again
    world.run(0.25);
    CHECK(world.conveyors()[0].speed == doctest::Approx(0.5).epsilon(0.01));
}

TEST_CASE("The belt counts how far its surface has travelled") {
    World world;
    world.addConveyor(belt(2.0F));
    world.run(1.5);
    CHECK(world.conveyors()[0].travelled == doctest::Approx(3.0).epsilon(0.01));
}

TEST_CASE("A box falls off the end, lands on the floor, and is cleared away") {
    World world;
    auto spec = belt(1.0F);
    spec.length = 1.0F;
    world.addConveyor(spec);
    const auto id = boxOn(world, 0, 0.7F);
    world.run(1.5);
    const auto fallen = view(world, id);
    CHECK(fallen.position.y < 0.2F); // on the floor
    CHECK(fallen.position.x > 1.0F); // past the end
    world.run(4.0);
    CHECK(world.itemCount() == 0);
}

TEST_CASE("advance() runs exact 1 ms steps, carrying the remainder") {
    World world;
    CHECK(world.advance(0.0105) == 10); // 10.5 ms: 10 steps, 0.5 ms left over
    CHECK(world.time() == doctest::Approx(0.010));
    CHECK(world.advance(0.0005) == 1); // the 0.5 ms left + 0.5 ms = one more step
    CHECK(world.stepCount() == 11);
}

TEST_CASE("advance() never runs more than maxSteps, and drops the backlog") {
    World world;
    CHECK(world.advance(10.0, 100) == 100);
    CHECK(world.advance(0.0) == 0); // the remaining 9.9 s were dropped, not queued
}

TEST_CASE("run() simulates exactly the time asked for") {
    World world;
    world.run(1.5);
    CHECK(world.stepCount() == 1500);
}

TEST_CASE("A spawner puts a box on the belt every interval") {
    World world;
    world.addConveyor(belt(0.5F));
    SpawnerSpec spawner;
    spawner.interval = 0.5;
    world.addSpawner(spawner);
    world.run(2.0);
    // Steps start at 0 ms, 1 ms, ... 1999 ms: the spawn due at 2.0 s belongs
    // to the step that starts at 2.0 s, which hasn't run yet.
    CHECK(world.itemCount() == 3); // at 0.5, 1.0 and 1.5 s
    world.step();
    CHECK(world.itemCount() == 4); // and now at 2.0 s
}

TEST_CASE("Boxes queue up behind each other without passing through") {
    World world;
    auto spec = belt(0.5F);
    world.addConveyor(spec);
    const auto front = boxOn(world, 0, 1.0F);
    const auto back = boxOn(world, 0, 0.6F);
    world.run(1.0);
    const auto gap = view(world, front).position.x - view(world, back).position.x;
    CHECK(gap == doctest::Approx(0.4).epsilon(0.05)); // same speed: the gap stays
}

TEST_CASE("Clearing removes every item, and the world goes on") {
    World world;
    world.addConveyor(belt(0.5F));
    boxOn(world, 0, 0.5F);
    boxOn(world, 0, 1.5F);
    world.clearItems();
    CHECK(world.itemCount() == 0);
    const auto id = boxOn(world, 0, 0.5F);
    world.run(0.5);
    CHECK(view(world, id).velocity.x > 0.4F);
}
