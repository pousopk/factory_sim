#include "sim/world.hpp"
#include <cmath>
#include <doctest/doctest.h>
#include <numbers>

using namespace fsim;

// Photocells and encoders: when they switch, exactly, and what they count.

namespace {

ConveyorSpec belt(float speed) {
    ConveyorSpec spec;
    spec.length = 4.0F;
    spec.targetSpeed = speed;
    spec.acceleration = 1000.0F; // instant
    return spec;
}

ItemId boxAt(World& world, float along, glm::vec3 size = {0.2F, 0.15F, 0.2F}) {
    const auto& c = world.conveyors()[0];
    return world.spawnItem(c.beltPoint(along) + glm::vec3(0.0F, size.y / 2.0F + 0.001F, 0.0F), size, {1, 1, 1});
}

// The box's front edge, metres from the belt's start.
float front(const World& world, ItemId id) {
    for (const auto& item : world.items()) {
        if (item.id == id) {
            return item.position.x + item.size.x / 2.0F;
        }
    }
    FAIL("no item");
    return 0.0F;
}

} // namespace

TEST_CASE("Beam coverage: clipped to the beam, overlaps counted once") {
    CHECK(beamCoverage(1.0F, 0.01F, {}) == 0.0F);
    CHECK(beamCoverage(1.0F, 0.01F, {{0.5F, 0.998F}}) == doctest::Approx(0.3F)); // covers 0.995..0.998
    CHECK(beamCoverage(1.0F, 0.01F, {{0.0F, 2.0F}}) == doctest::Approx(1.0F));
    CHECK(beamCoverage(1.0F, 0.01F, {{0.99F, 1.0F}, {0.999F, 1.002F}}) == doctest::Approx(0.7F));
}

TEST_CASE("The photocell switches when the front edge covers enough of the beam, exact to a fraction of a step") {
    World world;
    world.addConveyor(belt(0.5F));
    PhotocellSpec spec;
    spec.along = 1.5F;
    spec.beamDiameter = 0.01F;
    spec.responseTime = 0.0;
    world.addPhotocell(spec);
    const auto id = boxAt(world, 0.9F);
    world.run(0.5); // the box catches up with the belt

    // From here it moves at 0.5 m/s: its front reaches the "on" point
    // (60 % of the beam covered: 1 mm past the beam's centre) at a time we
    // can work out from where it is now.
    const float onPoint = 1.5F - 0.005F + 0.6F * 0.01F;
    const double expected = world.time() + (onPoint - front(world, id)) / 0.5;
    world.run(2.0);
    const auto& photocell = world.photocells()[0];
    REQUIRE(photocell.triggers.size() == 1);
    CHECK(std::abs(photocell.triggers.front().beamTime - expected) < 0.00005); // within 50 µs, though steps are 1 ms
    CHECK(photocell.triggers.front().item == id);
}

TEST_CASE("The output follows the beam after the response time; the error grows with it") {
    for (const double response : {0.0, 0.002, 0.010}) {
        CAPTURE(response);
        World world;
        world.addConveyor(belt(1.0F));
        PhotocellSpec spec;
        spec.responseTime = response;
        world.addPhotocell(spec);
        boxAt(world, 0.5F);
        world.run(2.0);
        const auto& trigger = world.photocells()[0].triggers.front();
        CHECK(trigger.outputTime - trigger.beamTime == doctest::Approx(response));
        // At 1 m/s: 1 mm of travel per ms of response time, plus where the
        // "on" point sits (60 % into an 8 mm beam: 0.8 mm past the centre).
        const double expectedError = 0.0008 + 1.0 * response;
        CHECK(std::abs(trigger.positionError - expectedError) < 0.0002);
        CHECK(world.photocells()[0].output.edges().front().time == doctest::Approx(trigger.outputTime));
    }
}

TEST_CASE("Hysteresis: on at 60 % covered, off below 40 %; the output goes off after the box") {
    World world;
    world.addConveyor(belt(0.5F));
    PhotocellSpec spec;
    spec.responseTime = 0.0;
    world.addPhotocell(spec);
    boxAt(world, 0.9F, {0.2F, 0.15F, 0.2F});
    world.run(3.0);
    const auto& edges = world.photocells()[0].output.edges();
    REQUIRE(edges.size() == 2);
    CHECK(edges[0].value);
    CHECK_FALSE(edges[1].value);
    // On when the front edge covers 60 % of the beam (0.1 d past its centre),
    // off when the rear edge leaves less than 40 % covered: also 0.1 d past
    // the centre. So the pulse lasts exactly the box's length at its speed:
    // the hysteresis shifts it, without shortening it.
    CHECK(edges[1].time - edges[0].time == doctest::Approx(0.2 / 0.5).epsilon(0.001));
}

TEST_CASE("Objects too small, or below the beam, aren't seen") {
    World world;
    world.addConveyor(belt(0.5F));
    PhotocellSpec spec;
    spec.beamDiameter = 0.01F;
    spec.height = 0.05F;
    world.addPhotocell(spec);
    boxAt(world, 0.6F, {0.005F, 0.15F, 0.2F}); // 5 mm long: covers at most 50 % of the beam
    boxAt(world, 0.2F, {0.2F, 0.03F, 0.2F});   // 3 cm tall: the beam passes above it
    world.run(4.0);
    CHECK(world.photocells()[0].triggers.empty());
}

TEST_CASE("The encoder counts pulses per metre of belt") {
    ConveyorSpec spec = belt(0.5F);
    spec.encoder.pulsesPerRevolution = 1000;
    spec.encoder.rollerDiameter = 0.1F;
    CHECK(spec.encoder.pulsesPerMetre() == doctest::Approx(1000.0 / (std::numbers::pi * 0.1)));
    World world;
    world.addConveyor(spec);
    world.run(2.0); // 1 m of belt
    CHECK(world.conveyors()[0].encoderCount() == static_cast<std::int64_t>(std::floor(spec.encoder.pulsesPerMetre())));
}

TEST_CASE("A trigger records the encoder count at the output time, not at the step") {
    World world;
    auto spec = belt(1.0F);
    world.addConveyor(spec);
    PhotocellSpec cell;
    cell.responseTime = 0.0005; // half a step
    world.addPhotocell(cell);
    boxAt(world, 0.5F);
    world.run(2.0);
    const auto& trigger = world.photocells()[0].triggers.front();
    // At 1 m/s the belt has travelled outputTime metres (it started at full speed).
    const auto expected = static_cast<std::int64_t>(std::floor(trigger.outputTime * spec.encoder.pulsesPerMetre()));
    CHECK(std::abs(trigger.encoderCount - expected) <= 1);
}
