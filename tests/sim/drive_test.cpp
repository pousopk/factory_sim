#include "sim/scene.hpp"
#include <doctest/doctest.h>
#include <functional>

using namespace fsim;

// The drive (PROFIdrive telegram 1), on its own and in the World.

namespace {

// Runs the drive for `seconds` in 1 ms steps: 1/s acceleration, 5/s quick
// stop, 1.5/s coasting (fractions of the reference speed per second).
void run(Drive& drive, double seconds) {
    for (int i = 0; i < static_cast<int>(seconds * 1000.0 + 0.5); ++i) {
        drive.update(0.001, 1.0, 5.0, 1.5);
    }
}

bool has(std::uint16_t word, std::uint16_t bit) {
    return (word & bit) != 0;
}

constexpr std::int16_t kHalf = kFullScale / 2; // 50 %

} // namespace

TEST_CASE("Switching on: 16#047E, then 16#047F; an 'on' left over doesn't start the drive") {
    Drive drive;
    CHECK(drive.state() == Drive::State::SwitchingOnInhibited);
    CHECK(has(drive.statusWord(), zsw1::kSwitchingOnInhibited));
    drive.command(stw1::kRun, kHalf); // "run" without "ready" first: nothing
    CHECK(drive.state() == Drive::State::SwitchingOnInhibited);
    drive.command(stw1::kReady, kHalf);
    CHECK(drive.state() == Drive::State::ReadyToSwitchOn);
    CHECK(has(drive.statusWord(), zsw1::kReadyToSwitchOn));
    CHECK_FALSE(has(drive.statusWord(), zsw1::kReadyToOperate));
    drive.command(stw1::kRun, kHalf);
    CHECK(drive.state() == Drive::State::OperationEnabled);
    const auto status = drive.statusWord();
    CHECK(has(status, zsw1::kReadyToSwitchOn));
    CHECK(has(status, zsw1::kReadyToOperate));
    CHECK(has(status, zsw1::kOperationEnabled));
    CHECK_FALSE(has(status, zsw1::kFault));
}

TEST_CASE("The ramp takes the speed to the setpoint; NIST_A and 'at speed' follow") {
    Drive drive;
    drive.command(stw1::kReady, kHalf);
    drive.command(stw1::kRun, kHalf);
    run(drive, 0.25);
    CHECK(drive.speed() == doctest::Approx(0.25)); // 1 per second
    CHECK_FALSE(has(drive.statusWord(), zsw1::kAtSpeed));
    run(drive, 0.5);
    CHECK(drive.speed() == doctest::Approx(0.5));
    CHECK(drive.actualSpeed() == kHalf);
    CHECK(has(drive.statusWord(), zsw1::kAtSpeed));
    CHECK(has(drive.statusWord(), zsw1::kForward));
    drive.command(stw1::kRun, static_cast<std::int16_t>(-kHalf)); // backwards
    run(drive, 1.5);
    CHECK(drive.actualSpeed() == -kHalf);
    CHECK_FALSE(has(drive.statusWord(), zsw1::kForward));
}

TEST_CASE("The three stops: OFF1 ramps down, OFF3 ramps down fast, OFF2 coasts") {
    Drive drive;
    drive.command(stw1::kReady, kFullScale);
    drive.command(stw1::kRun, kFullScale);
    run(drive, 1.0); // at 100 %
    SUBCASE("OFF1 (16#047E): down the normal ramp, still enabled until standstill, then ready") {
        drive.command(stw1::kReady, kFullScale);
        run(drive, 0.5);
        CHECK(drive.speed() == doctest::Approx(0.5));
        CHECK(drive.state() == Drive::State::OperationEnabled);
        run(drive, 0.6);
        CHECK(drive.speed() == 0.0);
        CHECK(drive.state() == Drive::State::ReadyToSwitchOn);
    }
    SUBCASE("OFF3 (bit 2 off): the quick-stop ramp, then inhibited") {
        drive.command(stw1::kRun & ~stw1::kNoQuickStop, kFullScale);
        CHECK_FALSE(has(drive.statusWord(), zsw1::kNoQuickStop));
        run(drive, 0.1);
        CHECK(drive.speed() == doctest::Approx(0.5)); // 5 per second
        run(drive, 0.2);
        CHECK(drive.state() == Drive::State::SwitchingOnInhibited);
    }
    SUBCASE("OFF2 (bit 1 off): pulses off at once; the belt coasts down by itself") {
        drive.command(stw1::kRun & ~stw1::kNoCoastStop, kFullScale);
        CHECK(drive.state() == Drive::State::SwitchingOnInhibited);
        run(drive, 0.2);
        CHECK(drive.speed() == doctest::Approx(0.7)); // 1.5 per second
    }
    SUBCASE("Setpoint disabled (bit 6 off): down to 0, but still enabled") {
        drive.command(stw1::kRun & ~stw1::kSetpointEnable, kFullScale);
        run(drive, 1.2);
        CHECK(drive.speed() == 0.0);
        CHECK(drive.state() == Drive::State::OperationEnabled);
    }
}

TEST_CASE("A fault: the motor coasts; a rising edge of bit 7 acknowledges it") {
    Drive drive;
    drive.command(stw1::kReady, kHalf);
    drive.command(stw1::kRun, kHalf);
    run(drive, 1.0);
    drive.trip("F30005: overload");
    CHECK(has(drive.statusWord(), zsw1::kFault));
    CHECK_FALSE(has(drive.statusWord(), zsw1::kNoOverload));
    run(drive, 0.1);
    CHECK(drive.speed() == doctest::Approx(0.35)); // coasting
    drive.command(stw1::kRun, kHalf);              // "run" changes nothing while faulted
    CHECK(drive.state() == Drive::State::Fault);
    drive.command(stw1::kReady | stw1::kAcknowledge, kHalf); // the acknowledge edge
    CHECK(drive.state() == Drive::State::ReadyToSwitchOn);   // ... and on through 047E
    CHECK(drive.fault().empty());
    drive.command(stw1::kRun | stw1::kAcknowledge, kHalf); // held: no new edge, no harm
    CHECK(drive.state() == Drive::State::OperationEnabled);
}

TEST_CASE("Without 'control by PLC' the telegram is ignored, and a running drive faults") {
    Drive drive;
    drive.command(stw1::kReady & ~stw1::kControlByPlc, kHalf);
    CHECK(drive.state() == Drive::State::SwitchingOnInhibited);
    drive.command(stw1::kReady, kHalf);
    drive.command(stw1::kRun, kHalf);
    drive.command(0, 0); // the PLC went to STOP: all outputs 0
    CHECK(drive.state() == Drive::State::Fault);
    CHECK(drive.fault().find("F07220") == 0);
}

namespace {

// A control program made of a function, for tests.
class Lambda final : public ControlProgram {
public:
    explicit Lambda(std::function<void(IoImage&, double)> f) : f_(std::move(f)) {}
    void scan(IoImage& io, double time) override { f_(io, time); }
    std::string name() const override { return "test"; }

private:
    std::function<void(IoImage&, double)> f_;
};

} // namespace

TEST_CASE("A belt with a drive moves as its PLC tells it, and reports back through its input words") {
    auto scene = defaultScene();
    scene.spawners.clear();
    fitDrive(scene, 0);
    const auto& d = scene.conveyors[0].spec.drive;
    CHECK(d.controlOutput == "%QW8"); // the encoder has %ID8 in the inputs, not the outputs
    CHECK(d.setpointOutput == "%QW10");
    CHECK(d.statusInput == "%IW12");
    CHECK(d.actualInput == "%IW14");
    std::uint16_t lastStatus = 0;
    auto world = buildWorld(scene, [&](const Scene& s) {
        const auto drive = s.conveyors[0].spec.drive;
        return std::make_unique<Lambda>([drive, &lastStatus](IoImage& io, double time) {
            lastStatus = static_cast<std::uint16_t>(io.value(*parseAddress(drive.statusInput)));
            io.setValue(*parseAddress(drive.controlOutput), time < 0.05 ? stw1::kReady : stw1::kRun);
            io.setValue(*parseAddress(drive.setpointOutput), static_cast<std::uint16_t>(kFullScale * 3 / 4));
        });
    });
    world->run(2.0); // 0.75 m/s at 1 m/s² (the reference speed is 1 m/s)
    CHECK(world->conveyors()[0].speed == doctest::Approx(0.75));
    CHECK(has(lastStatus, zsw1::kOperationEnabled));
    CHECK(has(lastStatus, zsw1::kAtSpeed));
    CHECK(world->io().value(*parseAddress(d.actualInput)) == static_cast<std::uint32_t>(kFullScale * 3 / 4));
    CHECK(world->conveyors()[0].motor.value());
    world->tripDrive(0, "F07900: motor blocked");
    world->run(0.1);
    CHECK(has(static_cast<std::uint16_t>(world->io().value(*parseAddress(d.statusInput))), zsw1::kFault));
    CHECK(world->conveyors()[0].speed < 0.75F); // coasting
}

TEST_CASE("Without a program, a belt with a drive runs at its target speed, as one without does") {
    auto scene = defaultScene();
    fitDrive(scene, 0);
    auto world = buildWorld(scene);
    world->run(1.0);
    CHECK(world->conveyors()[0].speed == doctest::Approx(scene.conveyors[0].spec.targetSpeed));
}

TEST_CASE("A potentiometer is an input word, 0 to 27648") {
    auto scene = defaultScene();
    scene.potentiometers.push_back(PotentiometerSpec{"Speed", "%IW16"});
    auto world = buildWorld(scene);
    world->turn(0, 50000); // past the end: held at full scale
    world->run(0.02);
    CHECK(world->io().value(*parseAddress("%IW16")) == 27648U);
    world->turn(0, 13824);
    world->run(0.02);
    CHECK(world->io().value(*parseAddress("%IW16")) == 13824U);
}

TEST_CASE("Drives and potentiometers are saved with the scene") {
    const auto scene = driveDemo("drive.st");
    const auto loaded = sceneFromJson(sceneToJson(scene));
    REQUIRE(loaded.has_value());
    CHECK(*loaded == scene);
}
