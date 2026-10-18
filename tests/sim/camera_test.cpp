#include "sim/scene.hpp"
#include <algorithm>
#include <cmath>
#include <doctest/doctest.h>

using namespace fsim;

// Cameras, the built-in camera triggering, and the tin line.

TEST_CASE("A camera busy with a picture ignores a trigger, and takes the next one once it's free") {
    CameraTiming camera;
    CHECK(camera.trigger(1.0, 0.05));
    CHECK(camera.busyAt(1.04));
    CHECK_FALSE(camera.trigger(1.03, 0.05));            // still reading out: lost
    CHECK(camera.busyUntil() == doctest::Approx(1.05)); // and an ignored trigger doesn't extend it
    CHECK(camera.trigger(1.05, 0.05));
}

namespace {

// Runs a camera triggering program for `scans` scans of 10 ms, with the
// sensor on during the given scans; returns the scans in which the trigger
// output was on.
std::vector<int> triggersFor(const CameraTriggerSpec& spec, const std::vector<int>& sensorOn, int scans) {
    CameraTriggering program({spec});
    IoImage io;
    std::vector<int> fired;
    for (int n = 0; n < scans; ++n) {
        io.setBit(*parseAddress(spec.sensorInput), std::find(sensorOn.begin(), sensorOn.end(), n) != sensorOn.end());
        program.scan(io, n * 0.010);
        if (io.bit(*parseAddress(spec.triggerOutput))) {
            fired.push_back(n);
        }
    }
    return fired;
}

} // namespace

TEST_CASE("Camera triggering with one timer: an item that comes while it runs gets no trigger") {
    CameraTriggerSpec spec;
    spec.delay = 0.050;
    // The sensor sees items in scans 2, 4 and 10: the second one comes while
    // the timer started by the first is still running.
    CHECK(triggersFor(spec, {2, 4, 10}, 20) == std::vector<int>{7, 15}); // 50 ms after 2 and after 10
    CameraTriggering program({spec});
    IoImage io;
    for (int n = 0; n < 20; ++n) {
        io.setBit(*parseAddress(spec.sensorInput), n == 2 || n == 4 || n == 10);
        program.scan(io, n * 0.010);
    }
    CHECK(program.status()[0].fired == 2);
    CHECK(program.status()[0].lost == 1);
}

TEST_CASE("Camera triggering per item: every item gets its own trigger, one scan long") {
    CameraTriggerSpec spec;
    spec.delay = 0.050;
    spec.method = DelayMethod::PerItem;
    CHECK(triggersFor(spec, {2, 4, 10}, 20) == std::vector<int>{7, 9, 15});
    // Two items due in consecutive scans: the output must go off in between,
    // so the second trigger comes a scan late.
    CHECK(triggersFor(spec, {2, 4}, 20) == std::vector<int>{7, 9});
    CHECK(triggersFor(spec, {2, 3}, 20).size() == 1); // one long pulse is one item
    spec.delay = 0.0;
    CHECK(triggersFor(spec, {1, 3}, 10) == std::vector<int>{1, 3}); // no delay: in the scan that sees it
}

TEST_CASE("Tins stand upright on the belt and trigger the photocell with their round front") {
    auto world = buildWorld(tinDemo());
    world->run(3.0);
    REQUIRE(world->itemCount() > 3);
    for (const auto& item : world->items()) {
        CHECK(item.shape == Shape::Tin);
        CHECK(std::abs((item.rotation * glm::vec3(0, 1, 0)).y) > 0.99F);
    }
    const auto& photocell = world->photocells()[0];
    REQUIRE(photocell.triggers.size() > 3);
    // The front of a round tin at the beam: the same error as a box's flat front.
    CHECK(photocell.triggers.front().positionError * 1000.0F == doctest::Approx(1.8).epsilon(0.2));
}

TEST_CASE("The tin line: three cameras take every tin, the fourth misses those that come too close") {
    auto world = buildWorld(tinDemo());
    world->run(12.0);
    const auto& cameras = world->cameras();
    REQUIRE(cameras.size() == 4);
    const auto tins = world->photocells()[0].triggers.size();
    REQUIRE(tins > 40);
    const auto count = [](const Camera& camera, bool taken) {
        return static_cast<std::size_t>(
            std::count_if(camera.shots.begin(), camera.shots.end(), [&](const Shot& s) { return s.taken == taken; }));
    };
    for (std::size_t c = 0; c < 3; ++c) {
        CAPTURE(c);
        CHECK(count(cameras[c], true) + 1 >= tins); // every tin (the last may be on its way)
        CHECK(count(cameras[c], false) == 0);
        for (const auto& shot : cameras[c].shots) {
            CHECK(shot.whole); // the whole tin in the picture
            CHECK(std::abs(shot.offset) < 0.015F);
            CHECK(shot.blur == doctest::Approx(0.0005)); // 1 m/s x 0.5 ms
        }
    }
    // K4 is triggered by a timer that can't time two tins at once.
    const auto* triggering = dynamic_cast<const CameraTriggering*>(world->program());
    REQUIRE(triggering != nullptr);
    const auto lost = triggering->status()[3].lost;
    CHECK(lost > 0);
    CHECK(count(cameras[3], true) + lost + 2 >= tins);
    CHECK(count(cameras[3], true) + lost <= tins);
    for (const auto& shot : cameras[3].shots) {
        CHECK(std::abs(shot.offset) < 0.015F); // its delay puts the tins it does take in the middle
    }
}

TEST_CASE("A camera's own trigger delay keeps it busy: a trigger during the delay is ignored") {
    auto scene = tinDemo();
    // K4's delay in the camera instead of the PLC, every tin triggered at once.
    scene.cameras[3].triggerDelay = scene.cameraTriggers[3].delay;
    scene.cameraTriggers[3].delay = 0.0;
    auto world = buildWorld(scene);
    world->run(12.0);
    const auto& k4 = world->cameras()[3];
    const auto ignored = std::count_if(k4.shots.begin(), k4.shots.end(), [](const Shot& s) { return !s.taken; });
    CHECK(ignored > 0);
    for (const auto& shot : k4.shots) {
        if (shot.taken) {
            CHECK(std::abs(shot.offset) < 0.015F);
        }
    }
    // The ready input is off while it's busy.
    CHECK(k4.busy.edges().size() > 4);
}

TEST_CASE("Cameras and their triggers are saved with the scene") {
    const auto scene = tinDemo();
    const auto loaded = sceneFromJson(sceneToJson(scene));
    REQUIRE(loaded.has_value());
    CHECK(*loaded == scene);
    CHECK(loaded->spawners[0].shape == Shape::Tin);
    CHECK(loaded->cameraTriggers[3].delay == doctest::Approx(0.150)); // whole scans
}

TEST_CASE("Removing a conveyor removes its cameras; a new camera gets free addresses") {
    auto scene = tinDemo();
    const auto k5 = addCamera(scene, 0);
    CHECK(scene.cameras[k5].name == "K5");
    CHECK(scene.cameras[k5].triggerOutput == "%QX0.4");
    CHECK(scene.cameras[k5].readyInput == "%IX0.5");
    removeConveyor(scene, 0);
    CHECK(scene.cameras.empty());
}

TEST_CASE("Each bin is built once") {
    const auto world = buildWorld(sortingDemo());
    CHECK(world->bins().size() == 1);
}
