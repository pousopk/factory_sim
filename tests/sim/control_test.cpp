#include "sim/scene.hpp"
#include <doctest/doctest.h>

using namespace fsim;

// The I/O image, addresses, and control through it.

TEST_CASE("Addresses read and write the way soft_plc writes them") {
    const auto bit = parseAddress("%IX0.3");
    REQUIRE(bit.has_value());
    CHECK(bit->area == IoAddress::Area::Input);
    CHECK(bit->byte == 0);
    CHECK(bit->bit == 3);
    CHECK(parseAddress("%q1.7") == IoAddress{IoAddress::Area::Output, IoAddress::Size::Bit, 1, 7});
    CHECK(formatAddress(*parseAddress("%ID8")) == "%ID8");
    CHECK(formatAddress(*parseAddress("%qw4")) == "%QW4");
    for (const char* bad : {"", "IX0.0", "%MX0.0", "%IX0.8", "%IX0", "%IW", "%IX0.3x"}) {
        CAPTURE(bad);
        CHECK_FALSE(parseAddress(bad).has_value());
    }
}

TEST_CASE("Words and double words are big-endian, as on an S7") {
    IoImage io;
    io.setValue(*parseAddress("%ID8"), 0x12345678U);
    CHECK(io.inputs()[8] == 0x12);
    CHECK(io.inputs()[11] == 0x78);
    CHECK(io.value(*parseAddress("%IW8")) == 0x1234U);
    io.setBit(*parseAddress("%IX0.3"), true);
    CHECK(io.inputs()[0] == 0x08);
    CHECK(io.value(*parseAddress("%IX0.3")) == 1U);
    CHECK(IoImage::fits(*parseAddress("%ID60")));
    CHECK_FALSE(IoImage::fits(*parseAddress("%ID62"))); // bytes 62-65: past the end of 64
}

TEST_CASE("New devices get the next free addresses") {
    auto scene = defaultScene(); // B1 at %IX0.0, C1's encoder at %ID8
    const auto pusher = addPusher(scene, 0);
    CHECK(scene.pushers[pusher].valveOutput == "%QX0.0");
    CHECK(scene.pushers[pusher].extendedInput == "%IX0.1");
    CHECK(scene.pushers[pusher].retractedInput == "%IX0.2");
    CHECK(scene.photocells[addPhotocell(scene, 0)].input == "%IX0.3");
    CHECK(scene.conveyors[addConveyor(scene)].spec.encoderInput == "%ID12");
    CHECK(scene.conveyors[addConveyor(scene)].spec.encoderInput == "%ID16");
    // Bytes 8-63 hold 14 double words: the 15th encoder gets none.
    for (int i = 3; i < 14; ++i) {
        addConveyor(scene);
    }
    CHECK(scene.conveyors.back().spec.encoderInput == "%ID60");
    CHECK(scene.conveyors[addConveyor(scene)].spec.encoderInput.empty());
}

TEST_CASE("A scan reads the devices into the image; a normally closed button reads TRUE until pressed") {
    auto scene = defaultScene();
    scene.buttons.push_back(ButtonSpec{"Stop", "%IX1.0", true, false});
    scene.buttons.push_back(ButtonSpec{"Start", "%IX1.1", false, false});
    auto world = buildWorld(scene);
    world->run(0.02);
    CHECK(world->io().bit(*parseAddress("%IX1.0")));       // stop: not pressed, closed
    CHECK_FALSE(world->io().bit(*parseAddress("%IX1.1"))); // start: not pressed, open
    world->press(0, true);
    world->press(1, true);
    world->run(0.02);
    CHECK_FALSE(world->io().bit(*parseAddress("%IX1.0")));
    CHECK(world->io().bit(*parseAddress("%IX1.1")));
}

namespace {

// A program that copies one input to one output, and counts its scans.
class Copy final : public ControlProgram {
public:
    Copy(IoAddress from, IoAddress to) : from_(from), to_(to) {}
    void scan(IoImage& io, double time) override {
        io.setBit(to_, io.bit(from_));
        lastTime = time;
        ++scans;
    }
    std::string name() const override { return "copy"; }
    int scans = 0;
    double lastTime = -1.0;

private:
    IoAddress from_, to_;
};

} // namespace

TEST_CASE("A program runs every 10 ms and drives valves and motors through the image") {
    auto scene = defaultScene();
    addPusher(scene, 0); // valve %QX0.0, extended %IX0.1
    scene.conveyors[0].spec.runOutput = "%QX0.1";
    auto world = buildWorld(scene);
    // Run the conveyor while the start button (%IX1.0) is held, and fire the pusher with it.
    scene.buttons.push_back(ButtonSpec{"Start", "%IX1.0", false, false});
    world->setPanel(scene.buttons, scene.lamps);
    auto program = std::make_unique<Copy>(*parseAddress("%IX1.0"), *parseAddress("%QX0.1"));
    auto* copy = program.get();
    world->setProgram(std::move(program));

    world->run(1.0);
    CHECK(copy->scans == 100);
    CHECK(world->conveyors()[0].speed == 0.0F); // no run command: stopped
    world->press(0, true);
    world->run(1.0);
    CHECK(world->conveyors()[0].speed == doctest::Approx(0.5F)); // running
    CHECK_FALSE(world->pushers()[0].stroke.commanded());         // %QX0.0 never set
}

TEST_CASE("The sorting demo pushes the bad boxes into the bin, and only those") {
    auto world = buildWorld(sortingDemo());
    world->run(60.0);
    const auto& bin = world->bins()[0];
    MESSAGE("spawned " << world->spawned() << ", bad " << world->spawnedNok() << "; bin: " << bin.count << " ("
                       << bin.nokCount << " bad)");
    REQUIRE(world->spawnedNok() > 5);
    // Every bad box that has had time to reach the pusher is in the bin
    // (the last one or two may still be on their way), and no good one.
    CHECK(bin.nokCount >= world->spawnedNok() - 2);
    CHECK(bin.count == bin.nokCount);
}
