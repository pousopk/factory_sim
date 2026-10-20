#include "sim/item_shape.hpp"
#include <cstdint>
#include <cstring>
#include <doctest/doctest.h>
#include <filesystem>
#include <fstream>

using namespace fsim;

namespace {

const glm::mat3 kUpright(1.0F); // not turned

// The volume a surface encloses: positive if its triangles face outward.
float enclosed(const Mesh& mesh) {
    float volume = 0.0F;
    for (std::size_t i = 0; i + 2 < mesh.positions.size(); i += 3) {
        volume += glm::dot(mesh.positions[i], glm::cross(mesh.positions[i + 1], mesh.positions[i + 2])) / 6.0F;
    }
    return volume;
}

float covered(const std::vector<std::pair<float, float>>& stretches) {
    float length = 0.0F;
    for (const auto& [from, to] : stretches) {
        length += to - from;
    }
    return length;
}

const std::filesystem::path kChannel = std::filesystem::path(FACTORY_SIM_EXAMPLES_DIR) / "u_channel.stl";

} // namespace

TEST_CASE("each built-in shape fits its size, faces outward and has a weight") {
    for (const auto shape : {Shape::Bottle, Shape::Bag, Shape::Ball, Shape::Bracket, Shape::Fish}) {
        CAPTURE(shapeName(shape));
        const auto size = typicalSize(shape);
        const auto geometry = itemGeometry(shape, size);
        REQUIRE(geometry);
        REQUIRE(*geometry != nullptr);
        const auto& g = **geometry;
        CHECK(g.size().x == doctest::Approx(size.x).epsilon(0.02));
        CHECK(g.size().y == doctest::Approx(size.y).epsilon(0.02));
        CHECK(g.size().z == doctest::Approx(size.z).epsilon(0.02));
        CHECK(enclosed(g.surface()) > 0.0F);
        CHECK(g.mass() > 0.0F);
        // The physics' pieces and the surface hold about the same volume.
        CHECK(g.volume() == doctest::Approx(enclosed(g.surface())).epsilon(0.25));
    }
}

TEST_CASE("the same shape and size is made once and shared") {
    const auto a = itemGeometry(Shape::Fish, {0.4F, 0.05F, 0.11F});
    const auto b = itemGeometry(Shape::Fish, {0.4002F, 0.05F, 0.11F}); // the same to the millimetre
    const auto c = itemGeometry(Shape::Fish, {0.5F, 0.05F, 0.11F});
    CHECK(a->get() == b->get());
    CHECK(a->get() != c->get());
    CHECK(itemGeometry(Shape::Box, {0.2F, 0.2F, 0.2F}).value() == nullptr); // boxes need no geometry
}

TEST_CASE("a frozen fish is heavier at its head, and its tail is thin") {
    const auto size = typicalSize(Shape::Fish);
    const auto& fish = **itemGeometry(Shape::Fish, size);
    // The head is at +x: its weight is ahead of the middle.
    CHECK(fish.centerOfMass().x > 0.02F);
    CHECK(propertiesOf(fish.material()).friction < 0.1F); // the ice

    // Lying on the belt (its bottom at y = 0), rigid: every cross-section is
    // centred on the same plane, so the thin tail is lifted off the belt, at
    // mid-height. A beam at that height is blocked by its whole length; one
    // near the top only by the thick part behind the head.
    const glm::vec3 center{0.0F, size.y / 2.0F, 0.0F};
    const glm::vec3 along{1.0F, 0.0F, 0.0F};
    std::vector<std::pair<float, float>> middle;
    fish.slice(center, kUpright, size.y / 2.0F, along, middle);
    std::vector<std::pair<float, float>> high;
    fish.slice(center, kUpright, size.y * 0.85F, along, high);
    REQUIRE(!middle.empty());
    REQUIRE(!high.empty());
    CHECK(covered(middle) > 0.97F * size.x);
    CHECK(covered(high) < 0.5F * size.x);
    CHECK(high.front().first > -0.1F * size.x); // the thick part is ahead of the middle
    // A beam just above the belt sees only where it's thickest.
    std::vector<std::pair<float, float>> low;
    fish.slice(center, kUpright, 0.002F, along, low);
    CHECK(covered(low) < covered(high));

    // From above: the body is as high as the fish is thick; the fin, in the
    // middle plane, half that.
    const auto down = glm::vec3(0.0F, -1.0F, 0.0F);
    const auto body = fish.castRay(center, kUpright, {0.1F, 1.0F, 0.0F}, down, 2.0F);
    const auto fin = fish.castRay(center, kUpright, {-0.19F, 1.0F, 0.0F}, down, 2.0F);
    const auto beside = fish.castRay(center, kUpright, {0.0F, 1.0F, 0.2F}, down, 2.0F);
    REQUIRE(body);
    REQUIRE(fin);
    CHECK(1.0F - *body > 0.9F * size.y); // the top's height above the belt
    CHECK(1.0F - *fin < 0.6F * size.y);
    CHECK_FALSE(beside); // misses it
}

TEST_CASE("how far a shape reaches follows its turning") {
    const auto size = typicalSize(Shape::Bottle);
    const auto& bottle = **itemGeometry(Shape::Bottle, size);
    const glm::vec3 up{0.0F, 1.0F, 0.0F};
    const auto [low, high] = bottle.span(kUpright, up);
    CHECK(low == doctest::Approx(-size.y / 2.0F).epsilon(0.02));
    CHECK(high == doctest::Approx(size.y / 2.0F).epsilon(0.02));
    // Fallen over (turned a quarter about the belt's direction): its height is its diameter.
    const glm::mat3 fallen{glm::vec3(1, 0, 0), glm::vec3(0, 0, 1), glm::vec3(0, -1, 0)};
    const auto [lowFallen, highFallen] = bottle.span(fallen, up);
    CHECK(highFallen - lowFallen == doctest::Approx(size.x).epsilon(0.03));
}

TEST_CASE("an angle bracket's weight is towards its upright plate") {
    const auto& bracket = **itemGeometry(Shape::Bracket, typicalSize(Shape::Bracket));
    CHECK(bracket.centerOfMass().x < -0.01F); // the upright is at the back (-x)
    CHECK(bracket.pieceCount() == 2);
    CHECK(propertiesOf(bracket.material()).metal);
}

TEST_CASE("an STL file is read as text or binary, in its units, z up") {
    const auto text = readStl(kChannel);
    REQUIRE(text);
    CHECK(text->size() == 28 * 3);

    // The same triangles written as binary STL read back the same.
    const auto binary = std::filesystem::temp_directory_path() / "factory_sim_channel.stl";
    {
        std::ofstream out(binary, std::ios::binary);
        const char header[80] = "binary";
        out.write(header, 80);
        const auto count = static_cast<std::uint32_t>(text->size() / 3);
        out.write(reinterpret_cast<const char*>(&count), 4);
        for (std::size_t i = 0; i < text->size(); i += 3) {
            const float normal[3] = {0.0F, 0.0F, 0.0F};
            out.write(reinterpret_cast<const char*>(normal), 12);
            for (std::size_t c = 0; c < 3; ++c) {
                out.write(reinterpret_cast<const char*>(&(*text)[i + c]), 12);
            }
            const std::uint16_t spare = 0;
            out.write(reinterpret_cast<const char*>(&spare), 2);
        }
    }
    const auto read = readStl(binary);
    std::filesystem::remove(binary);
    REQUIRE(read);
    CHECK(*read == *text);

    // In millimetres, z up: 120 long, 40 high, 60 wide, in metres with y up.
    const auto part = itemGeometry(Shape::Stl, {}, StlSpec{kChannel.string(), 0.001F, Material::Plastic, 1, true});
    REQUIRE(part);
    CHECK((*part)->size().x == doctest::Approx(0.12F));
    CHECK((*part)->size().y == doctest::Approx(0.04F));
    CHECK((*part)->size().z == doctest::Approx(0.06F));
    CHECK(enclosed((*part)->surface()) == doctest::Approx(92160e-9F).epsilon(0.001));

    CHECK_FALSE(itemGeometry(Shape::Stl, {}, StlSpec{"no such file.stl"}));
}

TEST_CASE("a concave STL part is split into convex pieces that follow its shape") {
    const StlSpec stl{kChannel.string(), 0.001F, Material::Plastic, 8, true};
    const auto split = itemGeometry(Shape::Stl, {}, stl);
    REQUIRE(split);
    const auto& channel = **split;
    CHECK(channel.pieceCount() >= 2);
    // One convex hull would fill the channel's inside: about 0.12 x 0.04 x
    // 0.06 = 288 cm³. The pieces hold about the material's 92 cm³.
    CHECK(channel.volume() < 0.6F * 0.12F * 0.04F * 0.06F);
    auto hull = stl;
    hull.pieces = 1;
    CHECK((*itemGeometry(Shape::Stl, {}, hull))->volume() == doctest::Approx(0.12F * 0.04F * 0.06F).epsilon(0.05));
    // A beam across the channel at mid-height is blocked by its two walls
    // only: two stretches with the open middle between them.
    const glm::vec3 across{0.0F, 0.0F, 1.0F};
    std::vector<std::pair<float, float>> walls;
    channel.slice(glm::vec3(0.0F), kUpright, 0.0F, across, walls);
    REQUIRE(walls.size() == 2);
    CHECK(walls[0].second - walls[0].first == doctest::Approx(0.006F).epsilon(0.05));
    CHECK(walls[1].first - walls[0].second == doctest::Approx(0.048F).epsilon(0.05));
}
