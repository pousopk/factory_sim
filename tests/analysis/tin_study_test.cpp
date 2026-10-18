#include "analysis/tin_study.hpp"
#include <algorithm>
#include <doctest/doctest.h>

using namespace fsim;

TEST_CASE("The gap study: K4's timer needs more than its delay between tins, the others far less") {
    const auto scene = tinDemo(); // 1 m/s, 73 mm tins, K4 delayed 150 ms by a timer
    const auto sweep = sweepGaps(scene, 0.0, 0.2, 0.005);
    const auto k1 = minimumGap(sweep, 0);
    const auto k4 = minimumGap(sweep, 3);
    REQUIRE(k1.has_value());
    REQUIRE(k4.has_value());
    // The timer runs 15 scans, and an edge in the scan it finishes is lost
    // too: tins must be more than 150 ms apart, or 160 ms when the scan
    // rounds against them. 160 mm pitch - 73 mm tin.
    CHECK(*k4 == doctest::Approx(0.085).epsilon(0.05));
    CHECK(*k1 < 0.03);
    CHECK(minimumGap(sweep) == k4); // the line works from K4's gap on
    // Below that, K4 misses tins and the others don't.
    const auto at = [&](double gap) {
        return *std::find_if(sweep.begin(), sweep.end(), [&](const GapOutcome& g) { return g.gap > gap - 1e-9; });
    };
    CHECK(at(0.05).cameras[3].missed > 0);
    CHECK(at(0.05).cameras[0].missed == 0);
    CHECK(at(0.05).cameras[3].ignored == 0); // the timer lost them: the camera never saw a trigger
}

TEST_CASE("The gap study: per-item triggering, or the delay in the camera") {
    auto scene = tinDemo();
    SUBCASE("A trigger per tin: K4 is as good as the others") {
        scene.cameraTriggers[3].method = DelayMethod::PerItem;
        const auto sweep = sweepGaps(scene, 0.0, 0.2, 0.005);
        CHECK(*minimumGap(sweep, 3) < 0.03);
    }
    SUBCASE("The delay in the camera: busy for delay + exposure + readout") {
        scene.cameras[3].triggerDelay = scene.cameraTriggers[3].delay;
        scene.cameraTriggers[3].delay = 0.0;
        const auto sweep = sweepGaps(scene, 0.0, 0.25, 0.005);
        // 150 + 0.5 + 40 ms busy: more than 190.5 ms apart, up to a scan more.
        CHECK(*minimumGap(sweep, 3) == doctest::Approx(0.125).epsilon(0.08));
        const auto below = studyGap(scene, 0.09);
        CHECK(below.cameras[3].ignored > 0); // here the camera itself ignores them
    }
}

TEST_CASE("The study agrees with the full simulation") {
    // Evenly spaced tins in the simulated line: 130 mm pitch (57 mm gap),
    // too close for K4, and 250 mm (177 mm gap), fine for all.
    for (const double interval : {0.13, 0.25}) {
        CAPTURE(interval);
        auto scene = tinDemo();
        scene.spawners[0].intervalJitter = 0.0;
        scene.spawners[0].interval = interval;
        auto world = buildWorld(scene);
        world->run(8.0);
        const double gap = interval * 1.0 - 0.073;
        const auto study = studyGap(scene, gap);
        const auto& k4 = world->cameras()[3];
        const auto taken = std::count_if(k4.shots.begin(), k4.shots.end(), [](const Shot& s) { return s.taken; });
        const auto tins = static_cast<long>(world->photocells()[0].triggers.size());
        const bool simulationMisses = taken + 2 < tins;
        CHECK(simulationMisses == (study.cameras[3].missed > 0));
        CHECK(study.cameras[0].missed == 0);
    }
}
