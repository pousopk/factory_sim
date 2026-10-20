#include "sim/scene.hpp"
#include "sim/world.hpp"
#include <doctest/doctest.h>

using namespace fsim;

namespace {

ConveyorSpec belt(float speed) {
    ConveyorSpec spec;
    spec.length = 4.0F;
    spec.targetSpeed = speed;
    spec.acceleration = 100.0F; // at speed at once
    return spec;
}

} // namespace

TEST_CASE("every shape comes to rest on the belt and rides it") {
    for (const auto shape : {Shape::Bottle, Shape::Bag, Shape::Ball, Shape::Bracket, Shape::Fish}) {
        CAPTURE(shapeName(shape));
        World world(1);
        world.addConveyor(belt(0.4F));
        world.run(0.05);
        const auto& c = world.conveyors()[0];
        const auto size = typicalSize(shape);
        world.spawnItem(c.beltPoint(0.5F) + glm::vec3(0.0F, size.y / 2.0F + 0.002F, 0.0F), size, world.itemColor(shape),
                        0.0F, false, shape, c.direction() * 0.4F);
        world.run(1.5);
        REQUIRE(world.items().size() == 1);
        const auto item = world.items().front();
        CHECK(item.geometry != nullptr);
        CHECK(glm::dot(item.velocity, c.direction()) == doctest::Approx(0.4F).epsilon(0.1));
        const float above = item.position.y - c.spec.beltHeight;
        CHECK(above > 0.0F);
        CHECK(above < size.y);
    }
}

TEST_CASE("a photocell sees a fish's real outline, not its bounding box") {
    // Photocells at three heights at the same place, a fish and then a box
    // of the fish's size passing them. The box blocks each beam for its
    // whole length. The fish, resting on its thickest part and its tail
    // fin, blocks each for a different stretch, and none for all of it.
    const float heights[] = {0.003F, 0.025F, 0.045F};
    const auto blocked = [&](Shape shape) {
        World world(1);
        world.addConveyor(belt(0.4F));
        for (const float h : heights) {
            PhotocellSpec p;
            p.along = 2.0F;
            p.height = h;
            world.addPhotocell(p);
        }
        world.run(0.05);
        const auto& c = world.conveyors()[0];
        const auto size = typicalSize(Shape::Fish);
        world.spawnItem(c.beltPoint(1.0F) + glm::vec3(0.0F, size.y / 2.0F + 0.001F, 0.0F), size, glm::vec3(0.6F), 0.0F,
                        false, shape, c.direction() * 0.4F);
        world.run(4.0);
        std::vector<double> lengths; // m of belt travel while blocked
        for (const auto& photocell : world.photocells()) {
            const auto& edges = photocell.output.edges();
            lengths.push_back(edges.size() >= 2 ? (edges[1].time - edges[0].time) * 0.4 : 0.0);
        }
        return lengths;
    };
    const float length = typicalSize(Shape::Fish).x;
    const auto box = blocked(Shape::Box);
    for (const double l : box) {
        CHECK(l == doctest::Approx(length).epsilon(0.03));
    }
    const auto fish = blocked(Shape::Fish);
    for (const double l : fish) {
        CHECK(l > 0.1 * length);
        CHECK(l < 0.8 * length);
    }
    CHECK(fish[2] < fish[1] - 0.05); // near its top, only the thick part
}

TEST_CASE("a laser measures a standing bottle's height") {
    World world(1);
    world.addConveyor(belt(0.4F));
    LaserSpec laser;
    laser.along = 2.0F;
    laser.mountHeight = 0.5F;
    laser.rangeMin = 0.1F;
    laser.rangeMax = 0.5F;
    laser.noise = 0.0F;
    world.setFieldSensors({}, {laser}, {});
    world.run(0.05);
    const auto& c = world.conveyors()[0];
    const auto bottle = typicalSize(Shape::Bottle);
    world.spawnItem(c.beltPoint(1.0F) + glm::vec3(0.0F, bottle.y / 2.0F + 0.001F, 0.0F), bottle, glm::vec3(0.3F), 0.0F,
                    false, Shape::Bottle, c.direction() * 0.4F);
    float nearest = 1.0F;
    for (int i = 0; i < 400; ++i) { // until it's past
        world.run(0.01);
        nearest = std::min(nearest, world.lasers()[0].trueDistance);
    }
    CHECK(0.5F - nearest == doctest::Approx(bottle.y).epsilon(0.02));
}

TEST_CASE("the odd shapes demo runs: every product made, seen and kept on the line") {
    setShapeFolders({FACTORY_SIM_EXAMPLES_DIR});
    auto world = buildWorld(shapesDemo());
    world->run(30.0);
    for (std::size_t i = 0; i < world->spawners().size(); ++i) {
        CAPTURE(i);
        CHECK(world->spawnerProblem(i).empty());
    }
    CHECK(world->spawned() >= 25);
    CHECK(world->photocells()[0].triggers.size() >= 15);
    CHECK(world->cameras()[0].image.taken > 5000);
    for (const auto& item : world->items()) {
        CHECK(item.position.y > -1.0F); // none fell through the belt
    }
    // A missing STL file stops only its spawner, and says why.
    auto scene = shapesDemo();
    scene.spawners.back().stl.file = "no such part.stl";
    auto missing = buildWorld(scene);
    missing->run(8.0);
    CHECK(missing->spawnerProblem(scene.spawners.size() - 1).find("can't find") != std::string::npos);
}
