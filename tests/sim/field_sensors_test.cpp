#include "sim/scene.hpp"
#include <cmath>
#include <doctest/doctest.h>

using namespace fsim;

// Proximity switches, laser distance sensors and light curtains.

namespace {

// A belt 0.1 m wide at `speed`, reaching it at once, with nothing else on it.
std::unique_ptr<World> lane(float speed) {
    auto world = std::make_unique<World>();
    ConveyorSpec spec;
    spec.length = 4.0F;
    spec.width = 0.10F;
    spec.targetSpeed = speed;
    spec.acceleration = 1000.0F;
    world->addConveyor(spec);
    return world;
}

// An item on the belt, already moving with it (as from an infeed): its
// position is then exactly the belt's travel.
ItemId put(World& world, float along, glm::vec3 size, Shape shape) {
    const auto& c = world.conveyors()[0];
    return world.spawnItem(c.beltPoint(along) + glm::vec3(0.0F, size.y / 2.0F + 0.001F, 0.0F), size, {1, 1, 1}, 0.0F,
                           false, shape, c.direction() * c.spec.targetSpeed);
}

const glm::vec3 kTin{0.073F, 0.11F, 0.073F};
const glm::vec3 kBox{0.12F, 0.09F, 0.09F};

ProximitySpec proximity(bool inductive) {
    ProximitySpec p;
    p.along = 1.0F;
    p.inductive = inductive;
    p.faceDiameter = 0.030F;
    p.range = 0.015F; // tins pass 13.5 mm from the face, boxes 5 mm
    p.height = 0.04F;
    return p;
}

// Whether the switch came on while the item passed it.
bool sees(const ProximitySpec& spec, glm::vec3 size, Shape shape) {
    auto world = lane(0.5F);
    world->setFieldSensors({spec}, {}, {});
    put(*world, 0.6F, size, shape);
    bool on = false;
    for (int i = 0; i < 1500 && !on; ++i) {
        world->step();
        on = world->proximities()[0].detector.output.value();
    }
    return on;
}

} // namespace

TEST_CASE("An inductive switch sees tins, not cardboard; a capacitive one sees both") {
    CHECK(sees(proximity(true), kTin, Shape::Tin));
    CHECK_FALSE(sees(proximity(true), kBox, Shape::Box));
    CHECK(sees(proximity(false), kTin, Shape::Tin));
    CHECK(sees(proximity(false), kBox, Shape::Box)); // 5 mm away: within 40 % of 15 mm
    auto far = proximity(false);
    far.range = 0.010F; // cardboard at 4 mm: the box, 5 mm away, is out of reach
    CHECK_FALSE(sees(far, kBox, Shape::Box));
    CHECK(sees(far, kTin, Shape::Tin) == false); // and the tin, 13.5 mm away, is beyond 10 mm
}

TEST_CASE("A proximity switch above the belt sees the tops of items within its range") {
    auto spec = proximity(true);
    spec.mount = Mount::Above;
    spec.height = 0.12F; // 10 mm over a tin
    CHECK(sees(spec, kTin, Shape::Tin));
    spec.height = 0.13F; // 20 mm: too far
    CHECK_FALSE(sees(spec, kTin, Shape::Tin));
}

TEST_CASE("A proximity switch switches at its face, with exact times, like a photocell") {
    auto world = lane(0.5F);
    world->setFieldSensors({proximity(true)}, {}, {});
    put(*world, 0.6F, kTin, Shape::Tin);
    world->run(1.5);
    const auto& edges = world->proximities()[0].detector.output.edges();
    REQUIRE(edges.size() == 2);
    // On when the tin's front covers half the 30 mm face: front at 1.0 m, from
    // 0.6365 m at 0.5 m/s, plus the 1 ms response.
    CHECK(edges[0].time == doctest::Approx((1.0 - 0.6365) / 0.5 + 0.001).epsilon(0.002));
    // On for the tin's length plus the face's diameter between the thresholds.
    CHECK(edges[1].time - edges[0].time == doctest::Approx((0.073 + 0.030 * (0.7 - 0.5)) / 0.5).epsilon(0.02));
}

TEST_CASE("A laser measures the distance to what's under it, as 0..27648 over its range") {
    auto world = lane(0.4F);
    LaserSpec laser;
    laser.along = 1.0F;
    laser.mountHeight = 0.5F;
    laser.rangeMin = 0.25F;
    laser.rangeMax = 0.5F;
    laser.noise = 0.0F;
    laser.output = "%IW8";
    world->setFieldSensors({}, {laser}, {});
    world->run(0.01);
    CHECK(world->lasers()[0].distance == doctest::Approx(0.5)); // the belt
    CHECK(world->lasers()[0].value == 27648);
    CHECK(laserValue(laser, 0.375F) == 13824); // halfway
    CHECK(laserValue(laser, 0.2F) == kAnalogOverflow);
    CHECK(laserValue(laser, -1.0F) == kAnalogOverflow);
    put(*world, 0.6F, kBox, Shape::Box);
    // The box arrives under the spot after (1.0 - 0.66) / 0.4 = 0.85 s.
    world->run(1.0);
    const auto& l = world->lasers()[0];
    CHECK(l.distance == doctest::Approx(0.5 - 0.09).epsilon(0.002)); // its top: 410 mm away
    world->run(0.02);
    CHECK(world->io().value(*parseAddress("%IW8")) == laserValue(laser, l.distance));
}

TEST_CASE("A laser's noise and resolution: steps of the resolution, spread by the noise") {
    auto world = lane(0.0F);
    LaserSpec laser;
    laser.mountHeight = 0.5F;
    laser.resolution = 0.0005F;
    laser.noise = 0.0002F;
    world->setFieldSensors({}, {laser}, {});
    world->run(2.0); // 1000 measurements, 2 ms apart
    const auto& samples = world->lasers()[0].samples;
    REQUIRE(samples.size() > 900);
    double sum = 0.0;
    double squares = 0.0;
    for (const auto& [time, distance] : samples) {
        // On the 0.5 mm grid.
        CHECK(std::abs(distance / 0.0005 - std::round(distance / 0.0005)) < 1e-3);
        sum += distance;
        squares += distance * distance;
    }
    const double n = static_cast<double>(samples.size());
    const double mean = sum / n;
    CHECK(mean == doctest::Approx(0.5).epsilon(0.0002));
    // The noise is less than one step, so the readings sit on 0.5 m and its
    // neighbours: some spread, but less than the noise plus a step.
    const double deviation = std::sqrt(squares / n - mean * mean);
    CHECK(deviation > 0.0001);
    CHECK(deviation < 0.0005);
}

TEST_CASE("A light curtain counts the beams an item breaks, once per scan, and says when it's clear") {
    auto world = lane(0.4F);
    LightCurtainSpec curtain;
    curtain.along = 1.0F;
    curtain.beams = 16;
    curtain.pitch = 0.01F;
    curtain.lowest = 0.01F;
    curtain.clearInput = "%IX1.0";
    curtain.countInput = "%IW8";
    world->setFieldSensors({}, {}, {curtain});
    world->run(0.02);
    CHECK(world->curtains()[0].clear.value());
    put(*world, 0.6F, kBox, Shape::Box); // 90 mm high: beams at 10..90 mm
    world->run(1.0);
    const auto& c = world->curtains()[0];
    CHECK_FALSE(c.clear.value());
    CHECK(c.blocked == 9);
    world->run(0.02);
    CHECK(world->io().value(*parseAddress("%IW8")) == 9U);
    CHECK_FALSE(world->io().bit(*parseAddress("%IX1.0")));
    world->run(0.6);
    CHECK(world->curtains()[0].clear.value());
    // Its changes come on its own 5 ms scan cycle.
    for (const auto& edge : world->curtains()[0].clear.edges()) {
        const double cycles = (edge.time - 0.001) / 0.005;
        CHECK(std::abs(cycles - std::round(cycles)) < 1e-6);
    }
}

TEST_CASE("The sensors demo: the inductive switch counts tins only, the capacitive one everything") {
    auto world = buildWorld(sensorsDemo());
    world->run(20.0);
    const auto rises = [](const Signal& s) {
        return std::count_if(s.edges().begin(), s.edges().end(), [](const Edge& e) { return e.value; });
    };
    const auto inductive = rises(world->proximities()[0].detector.output);
    const auto capacitive = rises(world->proximities()[1].detector.output);
    const auto curtain =
        std::count_if(world->curtains()[0].clear.edges().begin(), world->curtains()[0].clear.edges().end(),
                      [](const Edge& e) { return !e.value; });
    CAPTURE(inductive);
    CAPTURE(capacitive);
    CAPTURE(curtain);
    CHECK(inductive > 3);
    CHECK(capacitive > inductive + 3); // boxes too
    CHECK(curtain >= capacitive - 2);  // every item breaks the curtain (the last ones may not have got there)
    // Heights vary: the laser sees different tops.
    float low = 1.0F;
    float high = 0.0F;
    for (const auto& [time, distance] : world->lasers()[0].samples) {
        if (distance > 0.0F && distance < 0.49F) {
            low = std::min(low, distance);
            high = std::max(high, distance);
        }
    }
    CHECK(high - low > 0.02F);
}

TEST_CASE("The new sensors are saved with the scene and get free addresses") {
    auto scene = sensorsDemo();
    const auto loaded = sceneFromJson(sceneToJson(scene));
    REQUIRE(loaded.has_value());
    CHECK(*loaded == scene);
    CHECK(scene.proximities[0].input == "%IX0.0");
    CHECK(scene.proximities[1].input == "%IX0.1");
    CHECK(scene.lasers[0].output == "%IW8");
    CHECK(scene.curtains[0].clearInput == "%IX0.2");
    CHECK(scene.curtains[0].countInput == "%IW10");
    removeConveyor(scene, 0);
    CHECK(scene.proximities.empty());
    CHECK(scene.lasers.empty());
    CHECK(scene.curtains.empty());
}
