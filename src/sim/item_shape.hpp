#pragma once
#include <cstdint>
#include <filesystem>
#include <glm/glm.hpp>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <tl/expected.hpp>
#include <utility>
#include <vector>

// What items look like, beyond boxes and tins: bottles, bags, balls, an
// angle bracket, a frozen fish, or any part read from an STL file.
//
// Such an item has two descriptions of its shape:
//   - its surface, a triangle mesh: what's drawn, and what the sensors see
//     (a photocell's beam, a laser's spot, a line camera's pixels);
//   - a few convex pieces that together follow that surface: what the
//     physics collides. A physics engine needs convex pieces (a box, a
//     sphere, the hull of some points), because the contact between two of
//     them is quick to find; a fish or an L is concave, so it's several.
//
// Boxes and tins keep their own exact formulas (no geometry), so everything
// measured with them before stays exactly the same.
namespace JPH {
class Shape;
}

namespace fsim {

enum class Shape { Box, Tin, Bottle, Bag, Ball, Bracket, Fish, Stl };
constexpr Shape kShapes[] = {Shape::Box,  Shape::Tin,     Shape::Bottle, Shape::Bag,
                             Shape::Ball, Shape::Bracket, Shape::Fish,   Shape::Stl};
const char* shapeName(Shape shape); // "box", "tin", ... as in the scene file
std::optional<Shape> shapeNamed(std::string_view name);
glm::vec3 typicalSize(Shape shape); // length (along the belt), height, width

// What an item is made of: how it slides, bounces and weighs, and whether
// an inductive proximity switch sees it.
enum class Material { Cardboard, Tinplate, Glass, Film, Rubber, Steel, Aluminium, Plastic, FrozenFish };
constexpr Material kMaterials[] = {Material::Cardboard, Material::Tinplate, Material::Glass,
                                   Material::Film,      Material::Rubber,   Material::Steel,
                                   Material::Aluminium, Material::Plastic,  Material::FrozenFish};
struct MaterialProperties {
    const char* name;  // as in the scene file
    float friction;    // combined with the belt's (the square root of their product)
    float restitution; // how much of a bounce it keeps: 0 none, 1 all
    float density;     // kg/m³ of its whole volume (a bag is mostly air)
    bool metal;
};
const MaterialProperties& propertiesOf(Material material);
std::optional<Material> materialNamed(std::string_view name);
// What a built-in shape is made of (an STL part is made of what it says).
Material materialOf(Shape shape);

// A triangle mesh: three corners per triangle, counter-clockwise seen from
// outside, each with its normal (the same for a flat face, the surface's
// own for a smooth one).
struct Mesh {
    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> normals;
};

// An item's shape that isn't a box or a tin. Made once and shared by every
// item of that shape (and by the drawing), so it never changes: it's used
// as `const`. Coordinates are its own, in metres, with the origin in the
// middle of its bounding box (which is where the item is said to be).
class ItemGeometry {
public:
    // `pieces`: the points of each convex piece for the physics. A ball
    // gives none and a `sphereRadius` instead: a true sphere rolls better
    // than any polyhedron.
    ItemGeometry(Shape shape, Material material, Mesh surface, std::vector<std::vector<glm::vec3>> pieces,
                 float sphereRadius = 0.0F);
    ~ItemGeometry();
    ItemGeometry(const ItemGeometry&) = delete;
    ItemGeometry& operator=(const ItemGeometry&) = delete;

    std::uint64_t id() const { return id_; } // unique for the program's run (the drawing's key for its mesh)
    Shape shape() const { return shape_; }
    Material material() const { return material_; }
    const Mesh& surface() const { return surface_; }
    std::size_t pieceCount() const { return pieces_.size(); }
    glm::vec3 size() const { return size_; } // its bounding box
    float mass() const { return mass_; }
    float volume() const { return volume_; }
    glm::vec3 centerOfMass() const { return centerOfMass_; } // from the middle of its bounding box
    const JPH::Shape* physicsShape() const;                  // for the physics: the convex pieces, with its density

    // ---- What the sensors ask. The item is at `center`, turned by `rotation`.

    // The lowest and highest it reaches along `direction` (a unit vector),
    // from its centre: of the convex hull of its surface, which is enough to
    // tell whether it can be anywhere near a sensor.
    std::pair<float, float> span(const glm::mat3& rotation, glm::vec3 direction) const;
    // Where its surface crosses the horizontal plane at height `y`, along
    // `axis` (horizontal, a unit vector): the stretches of that axis its
    // cross-section covers, as world positions (dot products with `axis`),
    // added to `out`. That's exactly what a light beam across the belt at
    // that height is blocked by, gaps included.
    void slice(glm::vec3 center, const glm::mat3& rotation, float y, glm::vec3 axis,
               std::vector<std::pair<float, float>>& out) const;
    // The distance along a ray from `origin` in `direction` (a unit vector)
    // to its surface, if it hits within `length`.
    std::optional<float> castRay(glm::vec3 center, const glm::mat3& rotation, glm::vec3 origin, glm::vec3 direction,
                                 float length) const;

private:
    struct Jolt; // the Jolt shapes, kept out of this header
    std::uint64_t id_;
    Shape shape_;
    Material material_;
    Mesh surface_;
    std::vector<std::vector<glm::vec3>> pieces_;
    std::vector<glm::vec3> hull_; // the corners of its surface's convex hull, for span()
    glm::vec3 size_{0.0F};
    float mass_ = 0.0F;
    float volume_ = 0.0F;
    glm::vec3 centerOfMass_{0.0F};
    std::unique_ptr<Jolt> jolt_;
};

// An item read from an STL file: the file, its unit (STL files have none:
// 0.001 if it was drawn in millimetres), what it's made of, and how many
// convex pieces the physics may split it into (1: its convex hull). Its x
// runs along the belt.
struct StlSpec {
    std::string file;
    float unit = 0.001F;
    Material material = Material::Plastic;
    int pieces = 8;
    bool zUp = true; // drawn with z up, as most CAD programs do (here y is up)

    bool operator==(const StlSpec&) const = default;
};

// The geometry for an item of `shape` and `size` (its bounding box: length,
// height, width), or for an STL part. Null for boxes and tins, which need
// none. Made once for each shape, size and file, then shared; an STL file
// is read and split again only when it changes.
tl::expected<std::shared_ptr<const ItemGeometry>, std::string> itemGeometry(Shape shape, glm::vec3 size,
                                                                            const StlSpec& stl = {});

// Where relative STL file names are looked for, in order: the scene's own
// folder, the examples... (absolute names are used as they are).
void setShapeFolders(std::vector<std::filesystem::path> folders);
std::filesystem::path findShapeFile(const std::string& file);

// ---- The pieces, for tests and tools.

// The triangles of an STL file (binary or text), in the file's units.
tl::expected<std::vector<glm::vec3>, std::string> readStl(const std::filesystem::path& file);
// An STL part from its triangles (three corners each, in metres).
tl::expected<std::shared_ptr<const ItemGeometry>, std::string> stlGeometry(std::vector<glm::vec3> triangles,
                                                                           Material material, int pieces);
// A built-in shape (not a box or a tin) fitting `size`.
std::shared_ptr<const ItemGeometry> builtInGeometry(Shape shape, glm::vec3 size);

} // namespace fsim
