#include "sim/scene.hpp"
#include "sim/world.hpp"
#include <doctest/doctest.h>
#include <map>
#include <set>

using namespace fsim;

namespace {

// A stopped belt with one jet at 1 m on its left, blowing to the right.
struct Rig {
    World world{1};
    explicit Rig(AirJetSpec jet = {}) {
        ConveyorSpec belt;
        belt.length = 3.0F;
        belt.targetSpeed = 0.0F;
        world.addConveyor(belt);
        jet.along = 1.0F;
        world.setAirJets({jet});
        world.run(0.05);
    }
    // An item resting in front of the jet; returns how it has moved after
    // a blow of `seconds` and `settle` more.
    ItemView blow(Shape shape, glm::vec3 size, double seconds, double settle = 0.3) {
        const auto& c = world.conveyors()[0];
        world.spawnItem(c.beltPoint(1.0F) + glm::vec3(0.0F, size.y / 2.0F + 0.001F, 0.0F), size, glm::vec3(0.5F), 0.0F,
                        false, shape);
        world.run(0.5); // at rest
        start = world.items().front();
        world.blowAirJet(0, seconds);
        highest = start.position.y;
        for (int i = 0; i < static_cast<int>((seconds + settle) * 1000.0); ++i) {
            world.step();
            if (!world.items().empty()) {
                highest = std::max(highest, world.items().front().position.y);
            }
        }
        return world.items().front();
    }
    // How far it went across the belt, towards the far (right) side.
    float across(const ItemView& now) const {
        const auto right = -leftOf(world.conveyors()[0].direction());
        return glm::dot(now.position - start.position, right);
    }
    ItemView start{};
    float highest = 0.0F;
};

} // namespace

TEST_CASE("an air jet's air follows its valve after the valve's delays") {
    AirJetSpec spec;
    spec.openTime = 0.004;
    spec.closeTime = 0.006;
    AirJet jet(spec);
    jet.blow(1.0, 0.020); // 20 ms by hand
    double flowed = 0.0;
    for (int i = 0; i < 40; ++i) {
        flowed += jet.update(1.0 + i * 0.001, 1.0 + (i + 1) * 0.001) * 0.001;
    }
    const auto& air = jet.air().edges();
    REQUIRE(air.size() == 2);
    CHECK(air[0].time == doctest::Approx(1.004));
    CHECK(air[1].time == doctest::Approx(1.026));
    CHECK(flowed == doctest::Approx(0.022)); // the step it starts in counts only from then
    // A pulse shorter than the opening delay still lets some air through,
    // because the valve takes longer to close than to open.
    AirJet quick(spec);
    quick.command(2.0, true);
    quick.command(2.001, false);
    for (int i = 0; i < 20; ++i) {
        quick.update(2.0 + i * 0.001, 2.0 + (i + 1) * 0.001);
    }
    REQUIRE(quick.air().edges().size() == 2);
    CHECK(quick.air().edges()[1].time - quick.air().edges()[0].time == doctest::Approx(0.003));
}

TEST_CASE("a jet's streams spread evenly inside its cone") {
    const glm::vec3 axis{0.0F, 0.0F, 1.0F};
    const auto streams = streamsAround(axis, 0.2F);
    CHECK(streams.size() == 37);
    for (const auto& s : streams) {
        CHECK(glm::length(s) == doctest::Approx(1.0F));
        CHECK(std::acos(std::min(1.0F, glm::dot(s, axis))) <= 0.2F + 1e-5F);
    }
    AirJetSpec spec;
    CHECK(thrustOf(spec) == doctest::Approx(2.36F).epsilon(0.01)); // 2 mm at 6 bar
}

TEST_CASE("a flat face square to the jet takes all its thrust, straight across") {
    // A tall, long box right in front of the nozzles catches every stream.
    AirJetSpec spec;
    Rig rig(spec);
    const auto& c = rig.world.conveyors()[0];
    const glm::vec3 size{0.5F, 0.3F, 0.3F};
    // Its near face 3 cm from the nozzles' tips.
    const auto left = leftOf(c.direction());
    const float halfWidth = c.spec.width / 2.0F;
    rig.world.spawnItem(c.beltPoint(1.0F) + left * (halfWidth + 0.035F - 0.03F - size.z / 2.0F) +
                            glm::vec3(0.0F, size.y / 2.0F + 0.001F, 0.0F),
                        size, glm::vec3(0.5F), 0.0F, false, Shape::Box);
    rig.world.run(0.3);
    rig.world.blowAirJet(0, 0.05);
    float push = 0.0F;
    for (int i = 0; i < 30; ++i) {
        rig.world.step();
        push = std::max(push, rig.world.airJets()[0].lastPush);
    }
    CHECK(push == doctest::Approx(3.0F * thrustOf(spec)).epsilon(0.02));
}

TEST_CASE("the same jet throws a bag off the belt and can't move a box") {
    AirJetSpec spec;
    spec.nozzles = 4;
    spec.bore = 0.004F;
    spec.height = 0.01F;
    {
        Rig rig(spec);
        const auto bag = rig.blow(Shape::Bag, typicalSize(Shape::Bag), 0.15);
        CHECK(rig.across(bag) > 0.5F);                    // well past the belt's edge
        CHECK(rig.highest - rig.start.position.y > 0.1F); // and through the air
    }
    {
        Rig rig(spec);
        const auto box = rig.blow(Shape::Box, {0.15F, 0.10F, 0.12F}, 0.15); // cardboard, 1.8 kg
        CHECK(rig.across(box) < 0.15F);                                     // still on the belt
        CHECK(rig.highest - rig.start.position.y < 0.002F);                 // never left it
    }
}

TEST_CASE("a jet under a fish's rounded edge lifts it; on a box's flat side it only pushes") {
    AirJetSpec spec;
    spec.nozzles = 4;
    spec.bore = 0.004F;
    spec.height = 0.01F;
    Rig fish(spec);
    fish.blow(Shape::Fish, typicalSize(Shape::Fish), 0.15);
    CHECK(fish.highest - fish.start.position.y > 0.01F);
    // A box's flat side, square to the jet, gives no push upwards.
    Rig box(spec);
    box.blow(Shape::Box, {0.10F, 0.06F, 0.10F}, 0.15);
    CHECK(box.highest - box.start.position.y < 0.001F);
}

TEST_CASE("the sorter blows a jet for its pulse, rounded to whole scans") {
    SorterSpec spec;
    spec.nokInput = "%IX0.0";
    spec.encoderInput = "%ID8";
    spec.valveOutput = "%QX0.0";
    spec.extendedInput.clear(); // a jet has none
    spec.distance = 100;
    spec.pulse = 0.025;
    BuiltInSorting sorting({spec});
    IoImage io;
    const auto nok = *parseAddress("%IX0.0");
    const auto count = *parseAddress("%ID8");
    const auto valve = *parseAddress("%QX0.0");
    std::vector<double> open;
    for (int scan = 0; scan < 30; ++scan) {
        const double t = scan * 0.01;
        io.setBit(nok, scan == 2);
        io.setValue(count, static_cast<std::uint32_t>(scan * 20));
        sorting.scan(io, t);
        if (io.bit(valve)) {
            open.push_back(t);
        }
    }
    // Seen at scan 2 (count 40), fired at count 140 (scan 7), off at the
    // first scan 25 ms or more later (scan 10).
    REQUIRE(!open.empty());
    CHECK(open.front() == doctest::Approx(0.07));
    CHECK(open.size() == 3);
}

TEST_CASE("the rejecting demo: the pusher takes all kinds, the jet the fish it's set for") {
    auto world = buildWorld(rejectDemo());
    // Which shape each item is, and what lands in each bin.
    std::map<ItemId, Shape> shapes;
    std::set<ItemId> seen;
    std::map<Shape, int> inJetBin;
    for (int i = 0; i < 60000; ++i) {
        world->step();
        for (const auto& item : world->items()) {
            shapes.try_emplace(item.id, item.shape);
        }
        for (const auto id : world->bins()[1].inside) {
            if (seen.insert(id).second) {
                ++inJetBin[shapes[id]];
            }
        }
    }
    const auto& pusherBin = world->bins()[0];
    const auto& jetBin = world->bins()[1];
    CHECK(pusherBin.nokCount >= 10);
    CHECK(pusherBin.count - pusherBin.nokCount < pusherBin.nokCount); // a long paddle catches a neighbour now and then
    CHECK(inJetBin[Shape::Fish] >= 3);
    CHECK(inJetBin[Shape::Box] == 0);       // too heavy to blow
    CHECK(jetBin.count == jetBin.nokCount); // a jet only hits what it's fired at
}

TEST_CASE("air jets survive a trip to JSON, and a sorter's pulse with them") {
    auto scene = rejectDemo();
    scene.airJets[0].aim = 0.1F;
    const auto loaded = sceneFromJson(sceneToJson(scene));
    REQUIRE(loaded.has_value());
    CHECK(*loaded == scene);
    CHECK(loaded->sorters[1].pulse > 0.0);
}
