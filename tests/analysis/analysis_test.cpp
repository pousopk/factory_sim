#include "analysis/analysis.hpp"
#include "sim/scene.hpp"
#include <cmath>
#include <doctest/doctest.h>

using namespace fsim;

// The logic analyser's data, from real runs of the simulation.

TEST_CASE("Every digital signal is a trace, named and with its address") {
    auto scene = sortingDemo();
    scene.buttons.push_back(ButtonSpec{"Start", "%IX1.0", false, false});
    const auto world = buildWorld(scene);
    const auto list = traces(*world);
    // C1 motor, BQ, P1 valve / extended / retracted, Start button, Reject lamp.
    REQUIRE(list.size() == 7);
    CHECK(list[0].name == "C1 motor");
    CHECK(list[1].name == "BQ inspection");
    CHECK(list[1].address == "%IX0.0");
    CHECK(list[2].name == "P1 valve");
    CHECK(list[2].address == "%QX0.0");
    CHECK(list[5].name == "Start button");
    CHECK(list[6].name == "Reject lamp");
}

TEST_CASE("A signal's step shape: its value at the start, a step at each change, its value at the end") {
    Signal signal;
    signal.set(1.0, true);
    signal.set(1.5, false);
    signal.set(3.0, true);
    const auto shape = steps(signal, 0.5, 2.0);
    CHECK(shape.times == std::vector<double>{0.5, 1.0, 1.0, 1.5, 1.5, 2.0});
    CHECK(shape.values == std::vector<double>{0, 0, 1, 1, 0, 0});
    const auto inside = steps(signal, 1.2, 1.4); // no change in between: a flat line
    CHECK(inside.values == std::vector<double>{1, 1});
}

TEST_CASE("The sorter's chain, measured: valve opens, the cylinder goes out after its dead time and stroke") {
    auto world = buildWorld(sortingDemo());
    world->run(20.0);
    const auto& pusher = world->pushers()[0];
    REQUIRE_FALSE(pusher.stroke.valve().edges().empty());
    const auto opened = pusher.stroke.valve().edges().front();
    REQUIRE(opened.value);
    // "extended" comes exactly dead time + stroke time after the valve.
    const auto out = std::find_if(pusher.stroke.extended().edges().begin(), pusher.stroke.extended().edges().end(),
                                  [&](const Edge& e) { return e.value && e.time > opened.time; });
    REQUIRE(out != pusher.stroke.extended().edges().end());
    CHECK(out->time - opened.time == doctest::Approx(0.015 + 0.5));
    // The valve is switched at a scan: a whole multiple of 10 ms.
    CHECK(std::abs(opened.time * 100.0 - std::round(opened.time * 100.0)) < 1e-6);
}

TEST_CASE("The event log is every switch, in time order, and goes to CSV") {
    auto world = buildWorld(sortingDemo());
    world->run(10.0);
    const auto log = eventLog(traces(*world));
    REQUIRE(log.size() > 10);
    CHECK(std::is_sorted(log.begin(), log.end(), [](const Event& a, const Event& b) { return a.time < b.time; }));
    const auto csv = eventsCsv(log);
    CHECK(csv.rfind("time_s,signal,address,value\n", 0) == 0);
    CHECK(std::count(csv.begin(), csv.end(), '\n') == static_cast<long>(log.size()) + 1);
    CHECK(eventsCsv({Event{1.25, "say \"hi\", B1", "%IX0.0", true}}) ==
          "time_s,signal,address,value\n1.250000,\"say \"\"hi\"\", B1\",%IX0.0,1\n");
}

TEST_CASE("Statistics of a spread") {
    const std::vector<double> values{1.0, 2.0, 3.0, 4.0};
    const auto s = statistics(values);
    CHECK(s.count == 4);
    CHECK(s.mean == doctest::Approx(2.5));
    CHECK(s.deviation == doctest::Approx(std::sqrt(1.25)));
    CHECK(s.min == 1.0);
    CHECK(s.max == 4.0);
    CHECK(statistics(std::vector<double>{}).count == 0);
}

TEST_CASE("Position errors: the sensor's own, and wider as a PLC scan sees them") {
    auto world = buildWorld(sortingDemo()); // jittered spacing: boxes meet the scan at random moments
    world->run(60.0);
    const auto& photocell = world->photocells()[0];
    const auto own = statistics(positionErrors(photocell));
    const auto atScan = statistics(positionErrorsAtScan(photocell, 0.010));
    REQUIRE(own.count > 5);
    CHECK(own.deviation < 0.01);          // the sensor alone: the same error every time (mm)
    CHECK(atScan.mean > own.mean);        // the scan only ever adds
    CHECK(atScan.max - atScan.min > 1.0); // and spreads it over up to 5 mm (0.5 m/s x 10 ms)
    CHECK(atScan.max - own.max <= 5.0 + 1e-6);
}
