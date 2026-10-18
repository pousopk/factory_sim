#include "analysis/tin_study.hpp"
#include "core/process_image.hpp"
#include "monitor/monitor_client.hpp"
#include "plc/programs.hpp"
#include "plc/s7_link.hpp"
#include "plc/soft_plc_program.hpp"
#include "s7/s7_server.hpp"
#include <chrono>
#include <doctest/doctest.h>
#include <thread>

using namespace fsim;

// The PLC link: soft_plc in lockstep, over its monitoring protocol, and a
// live PLC over S7.

namespace {

std::unique_ptr<SoftPlcProgram> load(const char* source, std::uint16_t port = 0) {
    auto program = SoftPlcProgram::fromSource(source, port);
    REQUIRE_MESSAGE(program.has_value(), (program ? "" : program.error()));
    return std::move(*program);
}

const IoAddress kIn = *parseAddress("%IX0.3");
const IoAddress kOut = *parseAddress("%QX0.0");

} // namespace

TEST_CASE("Each scan is one soft_plc cycle: inputs in, program, outputs out") {
    auto plc = load("PROGRAM Main\nVAR a AT %IX0.3 : BOOL; q AT %QX0.0 : BOOL; n : INT; END_VAR\n"
                    "    q := a;\n    n := n + 1;\nEND_PROGRAM\n");
    IoImage io;
    plc->scan(io, 0.0);
    CHECK_FALSE(io.bit(kOut));
    io.setBit(kIn, true);
    plc->scan(io, 0.01);
    CHECK(io.bit(kOut));
    CHECK(plc->status().cycles == 2);
    const auto variables = plc->variables();
    const auto n = std::find_if(variables.begin(), variables.end(), [](const auto& v) { return v.path == "n"; });
    REQUIRE(n != variables.end());
    CHECK(n->value == "2");
}

TEST_CASE("Timers run on simulated time, however fast the simulation goes") {
    auto plc = load("PROGRAM Main\nVAR t : TON; q AT %QX0.0 : BOOL; END_VAR\n"
                    "    t(IN := TRUE, PT := T#2s);\n    q := t.Q;\nEND_PROGRAM\n");
    IoImage io;
    // 200 scans of 10 ms of simulated time, in far less real time.
    for (int i = 0; i < 200; ++i) {
        plc->scan(io, i * 0.01);
    }
    CHECK_FALSE(io.bit(kOut)); // 1.99 s
    plc->scan(io, 2.0);
    CHECK(io.bit(kOut)); // 2 s of simulated time
}

TEST_CASE("A runtime error stops the PLC with its outputs off; a compile error doesn't start it") {
    auto plc = load("PROGRAM Main\nVAR q AT %QX0.0 : BOOL; d : INT; x : INT; END_VAR\n"
                    "    q := TRUE;\n    x := 10 / d;\nEND_PROGRAM\n");
    IoImage io;
    plc->scan(io, 0.0);
    CHECK_FALSE(plc->status().running);
    CHECK(plc->status().error.find("division by zero") != std::string::npos);
    CHECK_FALSE(io.bit(kOut));

    const auto broken = SoftPlcProgram::fromSource("PROGRAM Main\n    x := 1;\nEND_PROGRAM\n");
    REQUIRE_FALSE(broken.has_value());
    CHECK(broken.error().find("line 2") != std::string::npos);
}

TEST_CASE("Monitoring: a client reads values and loads a new version, answered between cycles") {
    auto plc = load("PROGRAM Main\nVAR n : INT; q AT %QX0.0 : BOOL; END_VAR\n    n := n + 1;\nEND_PROGRAM\n", 15201);
    CHECK(plc->status().monitorPort == 15201);
    IoImage io;
    // The simulation runs on this thread; the client waits on another, as
    // ladder_editor's session does.
    std::atomic<bool> done{false};
    std::string read;
    std::string loaded;
    std::thread client([&] {
        auto connection = softplc::monitor::MonitorClient::connect("127.0.0.1", 15201);
        REQUIRE(connection.has_value());
        read = connection->request("READ n").value_or("");
        loaded = connection
                     ->request("LOAD PROGRAM Main\\nVAR n : INT; q AT %QX0.0 : BOOL; END_VAR\\n"
                               "    n := n + 1;\\n    q := TRUE;\\nEND_PROGRAM")
                     .value_or("");
        done = true;
    });
    for (int i = 0; !done && i < 5000; ++i) {
        plc->scan(io, i * 0.01);
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    client.join();
    CHECK(read.rfind("OK ", 0) == 0);
    CHECK(loaded.rfind("OK kept=", 0) == 0); // an online change: n kept its count
    plc->scan(io, 100.0);
    CHECK(io.bit(kOut)); // the new version runs
}

TEST_CASE("A live S7 link writes the inputs and reads the outputs of a PLC") {
    softplc::ProcessImage image(16, 16, 16);
    softplc::s7::S7Server server(image);
    REQUIRE(server.start(15202).has_value());
    S7Link link("127.0.0.1", 15202);
    IoImage io;
    io.setBit(kIn, true);
    link.scan(io, 0.0);
    REQUIRE_MESSAGE(link.status().connected, link.status().error);
    server.synchronize(); // the PLC takes the client's writes between its cycles
    CHECK(image.readBit(softplc::Area::Input, 0, 3));
    image.writeBit(softplc::Area::Output, 0, 0, true);
    server.synchronize();
    link.scan(io, 0.01);
    CHECK(io.bit(kOut));
    CHECK(link.status().exchanges == 2);
}

TEST_CASE("A live S7 link with no PLC there keeps every output off, and says why") {
    S7Link link("127.0.0.1", 15299);
    IoImage io;
    io.outputs().fill(0xFF);
    link.scan(io, 0.0);
    CHECK_FALSE(link.status().connected);
    CHECK_FALSE(link.status().error.empty());
    CHECK(io.outputs()[0] == 0);
}

TEST_CASE("ladder_editor's conveyor program runs the simulated conveyor") {
    const auto scene = conveyorDemo("conveyor.st");
    std::string error;
    auto world = buildWorld(scene, [&](const Scene& s) {
        auto made = makeAnyProgram(s, FACTORY_SIM_SOURCE_DIR "/examples");
        error = made.error;
        return std::move(made.program);
    });
    REQUIRE_MESSAGE(world->program() != nullptr, error);
    world->run(1.0);
    CHECK(world->conveyors()[0].speed == 0.0F); // not started: the motor output is off

    world->press(0, true); // Start
    world->run(0.1);
    world->press(0, false);
    world->run(2.0);
    CHECK(world->conveyors()[0].speed == doctest::Approx(0.5F)); // the program holds itself running
    CHECK(world->lit(0));                                        // the Motor lamp: %QX0.0

    world->run(20.0); // boxes reach the end light barrier, and the program counts them
    const auto* plc = dynamic_cast<SoftPlcProgram*>(world->program());
    REQUIRE(plc != nullptr);
    const auto variables = plc->variables();
    const auto boxes =
        std::find_if(variables.begin(), variables.end(), [](const auto& v) { return v.path == "boxes"; });
    REQUIRE(boxes != variables.end());
    CHECK(std::stoi(boxes->value) >= 5);

    world->press(1, true); // Stop (normally closed: pressing it opens the contact)
    world->run(0.1);
    world->press(1, false);
    world->run(2.0);
    CHECK(world->conveyors()[0].speed == 0.0F);
}

TEST_CASE("The tin line's triggering in ST, run by soft_plc, triggers the cameras exactly as the built-in one") {
    const auto builtIn = tinDemo();
    auto st = builtIn;
    st.control = Control::SoftPlc;
    st.plcProgram = "tin_cameras.st";
    const ProgramMaker make = [](const Scene& s) {
        auto made = makeAnyProgram(s, FACTORY_SIM_SOURCE_DIR "/examples");
        REQUIRE_MESSAGE(made.program != nullptr, made.error);
        return std::move(made.program);
    };
    auto a = buildWorld(builtIn);
    auto b = buildWorld(st, make);
    a->run(10.0);
    b->run(10.0);
    for (std::size_t c = 0; c < 4; ++c) {
        CAPTURE(c);
        const auto& x = a->cameras()[c].shots;
        const auto& y = b->cameras()[c].shots;
        REQUIRE(x.size() == y.size());
        for (std::size_t i = 0; i < x.size(); ++i) {
            CHECK(x[i].triggerTime == y[i].triggerTime);
            CHECK(x[i].taken == y[i].taken);
        }
    }
    const auto* plc = dynamic_cast<const SoftPlcProgram*>(b->program());
    REQUIRE(plc != nullptr);
    const auto* triggering = dynamic_cast<const CameraTriggering*>(a->program());
    const auto variables = plc->variables();
    const auto lost =
        std::find_if(variables.begin(), variables.end(), [](const auto& v) { return v.path == "k4Lost"; });
    REQUIRE(lost != variables.end());
    CHECK(lost->value == std::to_string(triggering->status()[3].lost));
    // And the gap study, run with the ST program, finds the same answer.
    CHECK(minimumGap(sweepGaps(st, 0.06, 0.12, 0.005, make)) == minimumGap(sweepGaps(builtIn, 0.06, 0.12, 0.005)));
}
