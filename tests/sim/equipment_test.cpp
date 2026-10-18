#include "sim/scene.hpp"
#include <algorithm>
#include <cmath>
#include <doctest/doctest.h>
#include <functional>
#include <numbers>

using namespace fsim;

// Accumulation zones, turntables and lifts, and merges.

namespace {

class Lambda final : public ControlProgram {
public:
    explicit Lambda(std::function<void(IoImage&, double)> f) : f_(std::move(f)) {}
    void scan(IoImage& io, double time) override { f_(io, time); }
    std::string name() const override { return "test"; }

private:
    std::function<void(IoImage&, double)> f_;
};

// The items on conveyor 0, front to back along it: where each centre is.
std::vector<float> positionsOn(const World& world) {
    const auto& c = world.conveyors()[0];
    std::vector<float> along;
    for (const auto& item : world.items()) {
        if (item.position.y > c.spec.beltHeight) {
            along.push_back(glm::dot(item.position - c.beltPoint(0.0F), c.direction()));
        }
    }
    std::sort(along.begin(), along.end(), std::greater<>());
    return along;
}

Scene accumulation() {
    Scene scene;
    PlacedConveyor c;
    c.spec.name = "C1";
    c.spec.length = 4.0F;
    c.spec.targetSpeed = 0.5F;
    c.spec.acceleration = 4.0F;
    scene.conveyors.push_back(c);
    setZones(scene, 0, 4, ZoneLogic::Local);
    scene.conveyors[0].spec.releaseOutput = "%QX0.0";
    SpawnerSpec spawner;
    spawner.interval = 1.0;
    spawner.atBeltSpeed = true;
    scene.spawners.push_back(spawner);
    return scene;
}

} // namespace

TEST_CASE("Zero-pressure accumulation: boxes stop at their zones' eyes, with gaps, until released") {
    bool release = false;
    auto world = buildWorld(accumulation(), [&](const Scene&) {
        return std::make_unique<Lambda>([&](IoImage& io, double) { io.setBit(*parseAddress("%QX0.0"), release); });
    });
    world->run(15.0);
    const auto held = positionsOn(*world);
    REQUIRE(held.size() >= 4);
    // One box per zone, each stopped just past its zone's eye (zone ends at
    // 1, 2, 3, 4 m; eyes 3 cm before them): 0.5 m/s ramping down at 4 m/s²
    // takes 31 mm, so its front stops between the eye and the zone's end.
    for (std::size_t z = 0; z < 4; ++z) {
        CAPTURE(z);
        const float front = held[z] + 0.1F;
        CHECK(front >= 4.0F - static_cast<float>(z) - 0.03F);
        CHECK(front <= 4.0F - static_cast<float>(z) + 0.005F);
    }
    // Never touching: a zone's length minus a box between them. (Behind the
    // four, more boxes may sit in the first zone: the spawner keeps putting
    // them down onto it, and no zone logic can singulate what's dropped onto
    // a full zone. On a real line, the full first zone would stop the infeed.)
    for (std::size_t i = 0; i + 1 < 4; ++i) {
        CHECK(held[i] - held[i + 1] > 0.5F);
    }
    for (const float speed : world->conveyors()[0].zoneSpeed) {
        CHECK(speed == 0.0F); // all held (and the spawner waits for room)
    }
    CHECK(world->io().bit(*parseAddress(world->conveyors()[0].spec.zoneIo[3].eyeInput)));
    // Released: the boxes move on one zone at a time (one box per zone), and
    // go off the end one after another.
    release = true;
    world->run(8.0);
    const auto& lastEye = world->conveyors()[0].eyes[3].output.edges();
    const auto discharged = std::count_if(lastEye.begin(), lastEye.end(), [](const Edge& e) { return !e.value; });
    CHECK(discharged >= 3);
}

TEST_CASE("Zones run by the PLC: each zone's motor is an output") {
    auto scene = accumulation();
    setZones(scene, 0, 4, ZoneLogic::Plc);
    const auto motors = scene.conveyors[0].spec.zoneIo;
    for (const auto& z : motors) {
        CHECK_FALSE(z.motorOutput.empty());
    }
    auto world = buildWorld(scene, [&](const Scene&) {
        // Only the first zone runs.
        return std::make_unique<Lambda>(
            [&](IoImage& io, double) { io.setBit(*parseAddress(motors[0].motorOutput), true); });
    });
    world->run(3.0);
    const auto& c = world->conveyors()[0];
    CHECK(c.zoneSpeed[0] == doctest::Approx(0.5));
    CHECK(c.zoneSpeed[1] == 0.0F);
    // A box carried by zone 0 stops where zone 1 begins, not at an eye: the
    // PLC decides, and it hasn't run zone 1.
    const auto held = positionsOn(*world);
    REQUIRE_FALSE(held.empty());
    CHECK(held.front() == doctest::Approx(1.0).epsilon(0.1));
}

TEST_CASE("A turntable turns its belt, and the box on it, about its middle") {
    auto world = std::make_unique<World>(1);
    ConveyorSpec spec;
    spec.length = 0.5F;
    spec.width = 0.4F;
    spec.targetSpeed = 0.0F;
    spec.carriage = Carriage::Turntable;
    spec.carriageStroke = {0.05, 1.0, 1.0};
    world->addConveyor(spec);
    const auto& c = world->conveyors()[0];
    const auto middle = c.beltPoint(0.25F);
    world->spawnItem(middle + glm::vec3(0.0F, 0.076F, 0.0F), {0.2F, 0.15F, 0.2F}, {1, 1, 1});
    world->run(0.2);
    world->conveyors()[0].carriage.command(world->time(), true);
    world->run(1.2);
    CHECK(world->conveyors()[0].carriage.extended().value());
    CHECK(world->conveyors()[0].spec.heading == doctest::Approx(std::numbers::pi / 2.0));
    CHECK(glm::length(world->conveyors()[0].beltPoint(0.25F) - middle) < 1e-4F); // its middle stayed put
    const auto item = world->items().front();
    CHECK(glm::length(glm::vec2(item.position.x - middle.x, item.position.z - middle.z)) < 0.02F); // still on it
    // ... and turned with it: its own x axis now points along the belt.
    const auto axis = item.rotation * glm::vec3(1, 0, 0);
    CHECK(std::abs(glm::dot(axis, world->conveyors()[0].direction())) > 0.98F);
}

TEST_CASE("A lift carries its box up, belt and all") {
    auto world = std::make_unique<World>(1);
    ConveyorSpec spec;
    spec.length = 0.6F;
    spec.targetSpeed = 0.0F;
    spec.carriage = Carriage::Lift;
    spec.liftHeight = 0.4F;
    spec.carriageStroke = {0.05, 1.0, 1.0};
    world->addConveyor(spec);
    world->spawnItem(world->conveyors()[0].beltPoint(0.3F) + glm::vec3(0.0F, 0.076F, 0.0F), {0.2F, 0.15F, 0.2F},
                     {1, 1, 1});
    world->run(0.2);
    const float before = world->items().front().position.y;
    world->conveyors()[0].carriage.command(world->time(), true);
    world->run(1.3);
    CHECK(world->conveyors()[0].spec.beltHeight == doctest::Approx(spec.beltHeight + 0.4F));
    CHECK(world->items().front().position.y == doctest::Approx(before + 0.4F).epsilon(0.01));
}

TEST_CASE("A conveyor joining another from the side: at right angles, ending at its edge, a little higher") {
    Scene scene;
    PlacedConveyor main;
    main.spec.length = 5.0F;
    main.spec.width = 0.4F;
    scene.conveyors.push_back(main);
    const auto side = addConveyorJoining(scene, 0, Side::Left);
    scene.conveyors[side].joinAt = 3.0F;
    const auto placed = placeConveyors(scene);
    const auto& s = placed[side];
    CHECK(s.heading == doctest::Approx(-std::numbers::pi / 2.0)); // heading 0 is +X; from the left (-Z) towards +Z
    const auto end = s.start + directionOf(s.heading) * s.length;
    CHECK(end.x == doctest::Approx(3.0));
    CHECK(end.z == doctest::Approx(-(0.2 + kTransferGap)));
    CHECK(s.beltHeight == doctest::Approx(main.spec.beltHeight + 0.02F));
    const auto loaded = sceneFromJson(sceneToJson(scene));
    REQUIRE(loaded.has_value());
    CHECK(*loaded == scene);
    removeConveyor(scene, 0); // the side one stays where it was, on its own
    CHECK_FALSE(scene.conveyors[0].joins.has_value());
    CHECK(placeConveyors(scene)[0].start == s.start);
}

TEST_CASE("Zones and carriages are saved with the scene, and change the layout") {
    const auto scene = equipmentDemo("equipment.st");
    const auto loaded = sceneFromJson(sceneToJson(scene));
    REQUIRE(loaded.has_value());
    CHECK(*loaded == scene);
    auto fewer = scene;
    setZones(fewer, 0, 3, ZoneLogic::Local);
    CHECK_FALSE(sameLayout(scene, fewer));
    auto slower = scene;
    slower.conveyors[1].spec.carriageStroke.extendTime = 3.0;
    CHECK(sameLayout(scene, slower)); // timing: changed live
}
