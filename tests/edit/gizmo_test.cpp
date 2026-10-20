#include "edit/gizmo.hpp"
#include "edit/placing.hpp"
#include "render/orbit_camera.hpp"
#include "sim/scene.hpp"
#include "sim/world.hpp"
#include <doctest/doctest.h>
#include <numbers>

using namespace fsim;

namespace {

constexpr float kQuarter = std::numbers::pi_v<float> / 2.0F;

bool near(glm::vec3 a, glm::vec3 b, float tolerance = 1e-4F) {
    return glm::length(a - b) <= tolerance;
}

} // namespace

TEST_CASE("a drag along an arrow is where the mouse ray passes nearest its line") {
    // The x axis; a ray straight down through x = 2, from above.
    const auto t = alongAxis({0, 0, 0}, {1, 0, 0}, {2, 5, 0.3F}, {0, -1, 0});
    REQUIRE(t);
    CHECK(*t == doctest::Approx(2.0F));
    // Looking straight along the axis, there's no such point.
    CHECK_FALSE(alongAxis({0, 0, 0}, {1, 0, 0}, {-5, 0, 0}, {1, 0, 0}));
}

TEST_CASE("a drag around a ring is the angle turned on its plane") {
    const glm::vec3 up{0, 1, 0};
    const auto from = onRingPlane({0, 0, 0}, up, {1, 3, 0}, {0, -1, 0});
    const auto to = onRingPlane({0, 0, 0}, up, {0, 3, -1}, {0, -1, 0});
    REQUIRE(from);
    REQUIRE(to);
    // From +x to -z about +y: a quarter turn anticlockwise (right-hand rule).
    CHECK(angleBetween(*from, *to, up) == doctest::Approx(kQuarter));
    CHECK(angleBetween(*to, *from, up) == doctest::Approx(-kQuarter));
    // Seen edge-on, the plane gives no answer rather than a wild one.
    CHECK_FALSE(onRingPlane({0, 0, 0}, up, {5, 0.01F, 0}, {-1, 0, 0}));
}

TEST_CASE("the mouse on an arrow picks it; the gizmo keeps its size on the screen") {
    OrbitCamera camera;
    camera.target = {0, 0, 0};
    camera.distance = 3.0F;
    const GizmoView view{camera.projection(1.5F) * camera.view(), {900.0F, 600.0F}};
    Gizmo gizmo;
    gizmo.size = gizmoSize(camera.distance, OrbitCamera::kFieldOfView, 600.0F);
    for (int axis = 0; axis < 3; ++axis) {
        CAPTURE(axis);
        const auto outline = handleOutline(gizmo, GizmoHandle{GizmoMode::Move, axis}, view);
        REQUIRE(outline.size() == 2);
        const auto onIt = (outline[0] + outline[1]) / 2.0F;
        const auto picked = handleAt(gizmo, GizmoMode::Move, onIt, view);
        REQUIRE(picked);
        CHECK(picked->axis == axis);
    }
    CHECK_FALSE(handleAt(gizmo, GizmoMode::Move, {5.0F, 5.0F}, view)); // a corner of the view: nothing
    // Twice as far away, twice as big in metres: the same on the screen.
    CHECK(gizmoSize(6.0F, 0.8F, 600.0F) == doctest::Approx(2.0F * gizmoSize(3.0F, 0.8F, 600.0F)));
}

TEST_CASE("moving a device along its belt slides it; moving it otherwise nudges it") {
    auto scene = sortingDemo(); // P1 on C1 at 3.5 m
    const Pick pusher{Pick::Kind::Pusher, 0};
    const auto start = gizmoTarget(scene, pusher);
    REQUIRE(start);
    const auto along = directionOf(scene.conveyors[0].spec.heading);
    // Half a metre along the belt: further along it, still as usual.
    auto moved = scene;
    placeAt(moved, scene, pusher, Frame{start->frame.origin + along * 0.5F, start->frame.rotation});
    CHECK(moved.pushers[0].along == doctest::Approx(4.0F));
    CHECK(moved.pushers[0].place.asUsual());
    // 10 cm up: the same place along, nudged.
    auto raised = scene;
    placeAt(raised, scene, pusher, Frame{start->frame.origin + glm::vec3(0, 0.1F, 0), start->frame.rotation});
    CHECK(raised.pushers[0].along == doctest::Approx(3.5F));
    CHECK_FALSE(raised.pushers[0].place.asUsual());
    CHECK(near(raised.pushers[0].place.offset, {0, 0.1F, 0}));
    // And back down: as usual again (the rounding left over is no nudge).
    auto back = raised;
    placeAt(back, raised, pusher, start->frame);
    CHECK(back.pushers[0].place.asUsual());
    // Its paddle follows: drawn and pushing 10 cm higher.
    const auto belt = placeConveyors(raised)[0];
    const auto before = pusherPaddle(scene.pushers[0], belt, 0.0);
    const auto after = pusherPaddle(raised.pushers[0], belt, 0.0);
    CHECK(near(after.center, before.center + glm::vec3(0, 0.1F, 0)));
}

TEST_CASE("detaching leaves a device exactly where it is; attaching puts it on a belt") {
    auto scene = sortingDemo();
    const Pick pusher{Pick::Kind::Pusher, 0};
    const Pick bin{Pick::Kind::Bin, 0};
    const auto belt = placeConveyors(scene)[0];
    const auto paddle = pusherPaddle(scene.pushers[0], belt, 0.5);
    const auto inside = binInside(scene.bins[0], belt);
    detach(scene, pusher);
    detach(scene, bin);
    CHECK(scene.pushers[0].place.parent == Parent::World);
    // The same paddle at the same stroke, the bin on the floor where it was.
    const auto freePaddle = pusherPaddle(scene.pushers[0], belt, 0.5);
    CHECK(near(freePaddle.center, paddle.center));
    CHECK(near(freePaddle.size, paddle.size));
    CHECK(near(binInside(scene.bins[0], belt).center, inside.center));
    // Moved 2 m away (across the floor), the bin keeps standing on the floor.
    const auto target = gizmoTarget(scene, bin);
    REQUIRE(target);
    placeAt(scene, Scene(scene), bin, Frame{target->frame.origin + glm::vec3(0, 0, 2.0F), target->frame.rotation});
    CHECK(binInside(scene.bins[0], belt).center.y == doctest::Approx(inside.center.y));
    // Attached again: on the belt at its usual place nearest to where it is.
    attach(scene, pusher, 0);
    CHECK(scene.pushers[0].place.asUsual());
    CHECK(scene.pushers[0].along == doctest::Approx(3.5F).epsilon(0.01));
}

TEST_CASE("a conveyor standing on its own moves, rises and turns about its middle") {
    auto scene = defaultScene();
    const Pick conveyor{Pick::Kind::Conveyor, 0};
    const auto target = gizmoTarget(scene, conveyor);
    REQUIRE(target);
    CHECK_FALSE(target->canTurn[0]); // only about the vertical
    CHECK(target->canTurn[1]);
    const auto middle = target->frame.origin;
    const auto turn = glm::angleAxis(kQuarter, glm::vec3(0, 1, 0));
    placeAt(scene, Scene(scene), conveyor, Frame{middle + glm::vec3(0, 0.1F, 0), turn * target->frame.rotation});
    CHECK(scene.conveyors[0].spec.heading == doctest::Approx(kQuarter));
    CHECK(scene.conveyors[0].spec.beltHeight == doctest::Approx(0.9F));
    const auto after = gizmoTarget(scene, conveyor);
    REQUIRE(after);
    CHECK(near(after->frame.origin, middle + glm::vec3(0, 0.1F, 0)));
}

TEST_CASE("devices on their own: saved, kept when a conveyor goes, and at work where they stand") {
    auto scene = sortingDemo();
    detach(scene, Pick{Pick::Kind::Bin, 0});
    auto raised = scene;
    const auto target = gizmoTarget(scene, Pick{Pick::Kind::Pusher, 0});
    REQUIRE(target);
    placeAt(raised, scene, Pick{Pick::Kind::Pusher, 0},
            Frame{target->frame.origin + glm::vec3(0, 0.05F, 0), target->frame.rotation});
    const auto loaded = sceneFromJson(sceneToJson(raised));
    REQUIRE(loaded);
    CHECK(*loaded == raised);
    // Removing the conveyor takes what was on it, not what stands on its own.
    auto removed = raised;
    removeConveyor(removed, 0);
    CHECK(removed.pushers.empty());
    CHECK(removed.bins.size() == 1);
    // A spawner standing on its own drops its items where it stands.
    auto spawning = defaultScene();
    detach(spawning, Pick{Pick::Kind::Spawner, 0});
    auto& spawner = spawning.spawners[0];
    spawner.place.offset = {5.0F, 2.0F, 5.0F};
    spawner.interval = 0.2;
    auto world = buildWorld(spawning, [](const Scene&) { return nullptr; }, 1);
    world->run(0.25);
    REQUIRE(world->items().size() == 1);
    CHECK(near(world->items().front().position, {5.0F, 2.0F + spawner.size.y / 2.0F + 0.002F, 5.0F}, 0.05F));
}
