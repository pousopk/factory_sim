#include "sim/scene.hpp"
#include <cmath>
#include <doctest/doctest.h>
#include <filesystem>
#include <fstream>
#include <numbers>

using namespace fsim;
namespace fs = std::filesystem;

// The scene: snapping conveyors, editing, building worlds, files.

namespace {

constexpr float kPi = std::numbers::pi_v<float>;

glm::vec3 endOf(const ConveyorSpec& spec) {
    return spec.start + glm::vec3(std::cos(spec.heading), 0.0F, -std::sin(spec.heading)) * spec.length;
}

bool near(glm::vec3 a, glm::vec3 b) {
    return glm::length(a - b) < 1e-5F;
}

} // namespace

TEST_CASE("A conveyor that follows another starts just past its end, in the same direction") {
    auto scene = defaultScene();
    const auto second = addConveyorAfter(scene, 0, Turn::Straight);
    const auto placed = placeConveyors(scene);
    CHECK(near(placed[second].start, endOf(placed[0]) + glm::vec3(kTransferGap, 0, 0)));
    CHECK(placed[second].heading == placed[0].heading);
    CHECK(scene.conveyors[second].spec.name == "C2");
}

TEST_CASE("Turns are 90 degrees, and the next belt runs across the end of the one before") {
    auto scene = defaultScene();
    const auto left = addConveyorAfter(scene, 0, Turn::Left);
    const auto placed = placeConveyors(scene);
    CHECK(placed[left].heading == doctest::Approx(kPi / 2.0F)); // towards -Z
    const auto& first = placed[0];
    // Its centre line passes just past the first one's end...
    CHECK(placed[left].start.x ==
          doctest::Approx(endOf(first).x + kTransferGap + placed[left].width / 2.0F).epsilon(1e-5));
    // ...and it starts half the first one's width back, to catch the box.
    CHECK(placed[left].start.z == doctest::Approx(first.width / 2.0F).epsilon(1e-5));
}

TEST_CASE("Moving or lengthening the first conveyor carries the whole chain") {
    auto scene = defaultScene();
    addConveyorAfter(scene, 0, Turn::Straight);
    addConveyorAfter(scene, 1, Turn::Right);
    const auto before = placeConveyors(scene);
    scene.conveyors[0].spec.length += 1.0F;
    scene.conveyors[0].spec.start.z += 0.5F;
    const auto after = placeConveyors(scene);
    for (std::size_t i = 1; i < 3; ++i) {
        CAPTURE(i);
        CHECK(near(after[i].start, before[i].start + glm::vec3(1.0F, 0.0F, 0.5F)));
    }
}

TEST_CASE("Removing a conveyor: what was on it goes, what followed it stays where it was") {
    auto scene = defaultScene();                // C1, a spawner and B1 on it
    addConveyorAfter(scene, 0, Turn::Straight); // C2 follows C1
    addConveyorAfter(scene, 1, Turn::Straight); // C3 follows C2
    addPhotocell(scene, 2);                     // B2 on C3
    const auto before = placeConveyors(scene);

    removeConveyor(scene, 1); // C2
    REQUIRE(scene.conveyors.size() == 2);
    CHECK(scene.conveyors[1].spec.name == "C3");
    CHECK_FALSE(scene.conveyors[1].follows.has_value());          // on its own now...
    CHECK(near(placeConveyors(scene)[1].start, before[2].start)); // ...where it was
    REQUIRE(scene.photocells.size() == 2);
    CHECK(scene.photocells[1].conveyor == 1); // B2 moved down with C3

    removeConveyor(scene, 0); // C1, with its spawner and B1
    CHECK(scene.spawners.empty());
    REQUIRE(scene.photocells.size() == 1);
    CHECK(scene.photocells[0].name == "B2");
    CHECK(scene.photocells[0].conveyor == 0);
}

TEST_CASE("Names are the first free ones; a separate conveyor goes beside the others") {
    auto scene = defaultScene();
    CHECK(scene.photocells[addPhotocell(scene, 0)].name == "B2");
    removePhotocell(scene, 0); // B1
    CHECK(scene.photocells[addPhotocell(scene, 0)].name == "B1");
    const auto apart = addConveyor(scene);
    CHECK(placeConveyors(scene)[apart].start.z > scene.conveyors[0].spec.width);
}

TEST_CASE("Speeds and sensors change live; moving a conveyor needs a new world") {
    const auto scene = defaultScene();
    auto changed = scene;
    changed.conveyors[0].spec.targetSpeed = 1.5F;
    changed.photocells[0].responseTime = 0.005;
    changed.spawners[0].interval = 0.3;
    CHECK(sameLayout(scene, changed));
    changed.conveyors[0].spec.length = 4.0F;
    CHECK_FALSE(sameLayout(scene, changed));
    auto added = scene;
    addConveyorAfter(added, 0, Turn::Straight);
    CHECK_FALSE(sameLayout(scene, added));
}

TEST_CASE("A live change keeps the boxes and the time; the photocell keeps its triggers") {
    const auto scene = defaultScene();
    auto world = buildWorld(scene);
    world->run(4.0);
    const auto boxes = world->itemCount();
    const auto triggers = world->photocells()[0].triggers.size();
    REQUIRE(boxes > 0);
    REQUIRE(triggers > 0);

    auto changed = scene;
    changed.conveyors[0].spec.targetSpeed = 1.0F;
    changed.photocells[0].along = 2.0F;
    applyLive(*world, changed);
    CHECK(world->itemCount() == boxes);
    CHECK(world->time() == doctest::Approx(4.0));
    CHECK(world->photocells()[0].triggers.size() == triggers);
    CHECK(world->photocells()[0].spec.along == 2.0F);
    world->run(1.0);
    CHECK(world->conveyors()[0].speed == doctest::Approx(1.0F)); // ramped to the new speed
}

TEST_CASE("Scenes survive a trip to JSON and back, exactly") {
    auto scene = defaultScene();
    addConveyorAfter(scene, 0, Turn::Left);
    addConveyor(scene);
    scene.conveyors[2].spec.heading = 0.3F;
    scene.photocells[0].responseTime = 0.0025;
    scene.spawners[0].size = {0.1F, 0.12F, 0.15F};
    const auto loaded = sceneFromJson(sceneToJson(scene));
    REQUIRE_MESSAGE(loaded.has_value(), (loaded ? "" : loaded.error()));
    CHECK(*loaded == scene);
}

TEST_CASE("Older files with missing fields load with defaults; broken ones say why") {
    const auto minimal = sceneFromJson(R"({"format": 1, "conveyors": [{"name": "Only"}]})");
    REQUIRE(minimal.has_value());
    CHECK(minimal->conveyors[0].spec.length == ConveyorSpec{}.length);
    CHECK(sceneFromJson("{").error().find("isn't valid") != std::string::npos);
    CHECK(sceneFromJson(R"({"format": 9})").error().find("unknown scene format 9") != std::string::npos);
    CHECK(sceneFromJson(R"({"format": 1, "photocells": [{"name": "B1", "conveyor": 3}]})").error() ==
          "photocell B1 is on conveyor 3, which doesn't exist");
    CHECK(sceneFromJson(R"({"format": 1, "conveyors": [{"name": "A", "follows": 0}]})")
              .error()
              .find("doesn't come before it") != std::string::npos);
}

TEST_CASE("Saving replaces the file in one step; loading reads it back") {
    const auto folder = fs::temp_directory_path() / "factory_sim_scene_test";
    fs::create_directories(folder);
    const auto path = folder / "scene.json";
    auto scene = defaultScene();
    addConveyorAfter(scene, 0, Turn::Right);
    REQUIRE(saveScene(scene, path).has_value());
    CHECK_FALSE(fs::exists(folder / "scene.json.tmp"));
    const auto loaded = loadScene(path);
    REQUIRE(loaded.has_value());
    CHECK(*loaded == scene);
    CHECK(loadScene(folder / "missing.json").error().find("can't read") != std::string::npos);
}
