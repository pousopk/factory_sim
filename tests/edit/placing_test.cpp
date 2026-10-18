#include "edit/placing.hpp"
#include <doctest/doctest.h>

using namespace fsim;

// Editing in 3D, as geometry: belts under the mouse, moving along them,
// placing, copy and paste.

namespace {

// A ray from a camera above and behind, through a point.
std::pair<glm::vec3, glm::vec3> rayThrough(glm::vec3 point) {
    const glm::vec3 eye{-1.0F, 3.0F, 2.0F};
    return {eye, glm::normalize(point - eye)};
}

} // namespace

TEST_CASE("The belt under the mouse, and where along it") {
    const auto scene = defaultScene(); // C1: 3 m along +X from the origin, 0.4 m wide, at 0.8 m
    const auto placed = placeConveyors(scene);
    const auto [origin, direction] = rayThrough(beltPoint(placed[0], 1.25F) + glm::vec3(0.0F, 0.0F, 0.1F));
    const auto hit = beltUnder(placed, origin, direction);
    REQUIRE(hit.has_value());
    CHECK(hit->conveyor == 0);
    CHECK(hit->along == doctest::Approx(1.25F).epsilon(0.001));
    // Off the belt (beside it): no belt under the mouse...
    const auto [o2, d2] = rayThrough(beltPoint(placed[0], 1.25F) + glm::vec3(0.0F, 0.0F, 0.5F));
    CHECK_FALSE(beltUnder(placed, o2, d2).has_value());
    // ... but still a place along it, for dragging.
    CHECK(alongNearest(placed[0], o2, d2) == doctest::Approx(1.25F).epsilon(0.05));
}

TEST_CASE("Where a ray meets a level, for dragging conveyors across the floor") {
    const auto at = onPlane({0.0F, 2.0F, 0.0F}, glm::normalize(glm::vec3(1.0F, -1.0F, 0.0F)), 0.5F);
    REQUIRE(at.has_value());
    CHECK(at->x == doctest::Approx(1.5F));
    CHECK(at->y == doctest::Approx(0.5F));
    CHECK_FALSE(onPlane({0.0F, 2.0F, 0.0F}, {1.0F, 0.0F, 0.0F}, 0.5F).has_value()); // level: never meets it
    CHECK_FALSE(onPlane({0.0F, 2.0F, 0.0F}, {0.0F, 1.0F, 0.0F}, 0.5F).has_value()); // looking up: behind
    CHECK(snap(1.234F, 0.01F) == doctest::Approx(1.23F));
    CHECK(snap(1.234F, 0.0F) == 1.234F);
}

TEST_CASE("Moving a device along its conveyor stays on it") {
    auto scene = defaultScene();
    const Pick photocell{Pick::Kind::Photocell, 0};
    moveAlong(scene, photocell, 2.2F);
    CHECK(placeOf(scene, photocell)->second == doctest::Approx(2.2F));
    moveAlong(scene, photocell, 9.0F);
    CHECK(placeOf(scene, photocell)->second == doctest::Approx(3.0F)); // the conveyor's end
    moveAlong(scene, Pick{Pick::Kind::Spawner, 0}, 0.4F);
    CHECK(scene.spawners[0].offset == doctest::Approx(0.4F)); // a spawner's "along"
    CHECK_FALSE(placeOf(scene, Pick{Pick::Kind::Conveyor, 0}).has_value());
}

TEST_CASE("Placing by clicking: a new device where the belt was clicked") {
    auto scene = defaultScene();
    const auto added = addAt(scene, Pick::Kind::Laser, 0, 1.8F);
    REQUIRE(added.has_value());
    CHECK(scene.lasers[added->index].along == doctest::Approx(1.8F));
    CHECK(scene.lasers[added->index].output == "%IW12"); // C1's encoder has %ID8
    CHECK_FALSE(addAt(scene, Pick::Kind::Conveyor, 0, 1.0F).has_value());
}

TEST_CASE("Copy and paste: a new name, its own wiring, a little further on") {
    auto scene = sortingDemo(); // P1 on %QX0.0, %IX0.1, %IX0.2
    const auto clip = copyOf(scene, Pick{Pick::Kind::Pusher, 0});
    REQUIRE(clip.has_value());
    const auto pasted = paste(scene, *clip);
    REQUIRE(pasted.kind == Pick::Kind::Pusher);
    const auto& copy = scene.pushers[pasted.index];
    CHECK(copy.name == "P2");
    CHECK(copy.along == doctest::Approx(scene.pushers[0].along + 0.2F));
    CHECK(copy.valveOutput != scene.pushers[0].valveOutput); // not fighting the original
    CHECK(copy.extendedInput != scene.pushers[0].extendedInput);
    CHECK_FALSE(copy.valveOutput.empty());
    // Pasted somewhere else, by clicking.
    const auto there = paste(scene, *clip, 0, 1.0F);
    CHECK(scene.pushers[there.index].along == doctest::Approx(1.0F));
    CHECK(scene.pushers[there.index].name == "P3");
}

TEST_CASE("Copy and paste a conveyor: beside the original, on its own, wired afresh") {
    auto scene = defaultScene();
    setZones(scene, 0, 3, ZoneLogic::Local);
    const auto clip = copyOf(scene, Pick{Pick::Kind::Conveyor, 0});
    REQUIRE(clip.has_value());
    const auto pasted = paste(scene, *clip);
    const auto& copy = scene.conveyors[pasted.index].spec;
    CHECK(copy.name == "C2");
    CHECK(copy.start.z == doctest::Approx(-(0.4F + 0.3F))); // to the left of a conveyor heading +X is -Z
    CHECK(copy.encoderInput == "%ID12");
    for (std::size_t z = 0; z < 3; ++z) {
        CHECK(copy.zoneIo[z].eyeInput != scene.conveyors[0].spec.zoneIo[z].eyeInput);
    }
    CHECK(sameLayout(scene, scene)); // (a valid scene)
}
