#include "sim/world.hpp"
#include <cmath>
#include <doctest/doctest.h>

using namespace fsim;

// The line-scan camera: lines from its clock or from the encoder, and what
// they show.

namespace {

// A belt at `speed` (reached at once), a line-scan camera 2 m along it with
// 1 mm pixels across (400 over 0.4 m), and a red box 0.2 m long and 0.1 m
// wide put down at 1 m, already moving with the belt.
std::unique_ptr<World> line(float speed, CameraSpec camera) {
    auto world = std::make_unique<World>(1);
    ConveyorSpec spec;
    spec.length = 4.0F;
    spec.targetSpeed = speed;
    spec.acceleration = 1000.0F;
    world->addConveyor(spec);
    world->run(0.01);
    camera.kind = CameraKind::Line;
    camera.along = 2.0F;
    camera.lineWidth = 0.4F;
    camera.pixelsAcross = 400;
    world->addCamera(camera);
    const auto& c = world->conveyors()[0];
    world->spawnItem(c.beltPoint(1.0F) + glm::vec3(0.0F, 0.05F + 0.001F, 0.0F), {0.2F, 0.1F, 0.1F}, {1.0F, 0.0F, 0.0F},
                     0.0F, false, Shape::Box, c.direction() * speed);
    return world;
}

bool red(const std::vector<std::uint8_t>& line, int x) {
    const auto at = static_cast<std::size_t>(x) * 3;
    return line[at] > 128 && line[at + 1] < 100;
}

// How many lines show the box (its middle pixel red), and how wide it is in
// the line that shows it widest.
std::pair<int, int> boxInImage(const World& world) {
    int rows = 0;
    int widest = 0;
    for (const auto& l : world.cameras()[0].image.lines) {
        if (red(l, 200)) {
            ++rows;
            int width = 0;
            for (int x = 0; x < 400; ++x) {
                width += red(l, x) ? 1 : 0;
            }
            widest = std::max(widest, width);
        }
    }
    return {rows, widest};
}

} // namespace

TEST_CASE("A free-running line camera: the faster the belt, the shorter the product in the image") {
    CameraSpec camera;
    camera.lineRate = 1000.0; // a line every 1 ms
    camera.exposure = 1e-6;   // too short to blur
    auto slow = line(0.5F, camera);
    auto fast = line(1.0F, camera);
    slow->run(2.5);
    fast->run(1.5);
    const auto [slowRows, slowWidth] = boxInImage(*slow);
    const auto [fastRows, fastWidth] = boxInImage(*fast);
    CHECK(slowRows == doctest::Approx(400).epsilon(0.01));  // 0.2 m at 0.5 m/s: 0.4 s, 400 lines
    CHECK(fastRows == doctest::Approx(200).epsilon(0.01));  // twice as fast: half as long
    CHECK(slowWidth == doctest::Approx(100).epsilon(0.02)); // 0.1 m of 1 mm pixels, whatever the speed
    CHECK(fastWidth == doctest::Approx(100).epsilon(0.02));
}

TEST_CASE("An encoder-triggered line camera: the same image at any speed") {
    CameraSpec camera;
    camera.lineTrigger = LineTrigger::Encoder;
    camera.pulsesPerLine = 13; // 6519 pulses/m: about 2 mm per line
    camera.exposure = 1e-6;
    auto slow = line(0.5F, camera);
    auto fast = line(1.0F, camera);
    slow->run(2.5);
    fast->run(1.5);
    const double expected = 0.2 * EncoderSpec{}.pulsesPerMetre() / 13.0; // about 100 lines
    CHECK(boxInImage(*slow).first == doctest::Approx(expected).epsilon(0.02));
    CHECK(boxInImage(*fast).first == doctest::Approx(expected).epsilon(0.02));
}

TEST_CASE("A long exposure blurs the product's edges along the travel") {
    // How many lines are in between belt and box (neither dark nor full red),
    // at its front and back edges.
    const auto blurred = [](double exposure) {
        CameraSpec camera;
        camera.lineRate = 1000.0;
        camera.exposure = exposure;
        auto world = line(1.0F, camera);
        world->run(1.5);
        int between = 0;
        for (const auto& l : world->cameras()[0].image.lines) {
            const int r = l[200 * 3];
            between += r > 60 && r < 230 ? 1 : 0;
        }
        return between;
    };
    CHECK(blurred(1e-6) <= 2);  // sharp: at most a line at each edge, caught halfway
    CHECK(blurred(0.004) >= 6); // 4 mm of travel during each line, 4 lines at 1 mm a line: soft edges
}

TEST_CASE("A line camera's noise: the belt's grey spreads by the set amount") {
    CameraSpec camera;
    camera.lineRate = 1000.0;
    camera.noise = 5.0F;
    auto world = line(0.5F, camera);
    world->run(0.5); // before the box arrives: belt only
    double sum = 0.0;
    double squares = 0.0;
    int n = 0;
    for (const auto& l : world->cameras()[0].image.lines) {
        for (int x = 0; x < 400; ++x) {
            const double v = l[static_cast<std::size_t>(x) * 3];
            sum += v;
            squares += v * v;
            ++n;
        }
    }
    const double mean = sum / n;
    CHECK(mean == doctest::Approx(0.18 * 255).epsilon(0.02));
    CHECK(std::sqrt(squares / n - mean * mean) == doctest::Approx(5.0).epsilon(0.1));
}
