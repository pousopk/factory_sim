// Jolt.h (its configuration) must come before any other Jolt header, so
// the formatter isn't allowed to sort these.
// clang-format off
#include "sim/item_shape.hpp"
#include "sim/convex_pieces.hpp"
#include "sim/physics.hpp"
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/SubShapeID.h>
// clang-format on
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <fmt/format.h>
#include <fstream>
#include <limits>
#include <map>
#include <mutex>
#include <sstream>

namespace fsim {

namespace {

constexpr float kPi = 3.14159265F;

JPH::Vec3 toJolt(glm::vec3 v) {
    return {v.x, v.y, v.z};
}
glm::vec3 toGlm(JPH::Vec3 v) {
    return {v.GetX(), v.GetY(), v.GetZ()};
}

} // namespace

// ---------------------------------------------------------------- Names and materials

const char* shapeName(Shape shape) {
    switch (shape) {
    case Shape::Box:
        return "box";
    case Shape::Tin:
        return "tin";
    case Shape::Bottle:
        return "bottle";
    case Shape::Bag:
        return "bag";
    case Shape::Ball:
        return "ball";
    case Shape::Bracket:
        return "bracket";
    case Shape::Fish:
        return "fish";
    case Shape::Stl:
        return "stl";
    }
    return "box";
}

std::optional<Shape> shapeNamed(std::string_view name) {
    for (const auto shape : kShapes) {
        if (name == shapeName(shape)) {
            return shape;
        }
    }
    return std::nullopt;
}

glm::vec3 typicalSize(Shape shape) {
    switch (shape) {
    case Shape::Box:
        return {0.2F, 0.15F, 0.2F};
    case Shape::Tin:
        return {0.073F, 0.11F, 0.073F};
    case Shape::Bottle:
        return {0.075F, 0.25F, 0.075F}; // a 1 l bottle
    case Shape::Bag:
        return {0.26F, 0.07F, 0.19F}; // a bag of crisps
    case Shape::Ball:
        return {0.068F, 0.068F, 0.068F}; // a tennis ball
    case Shape::Bracket:
        return {0.12F, 0.06F, 0.08F};
    case Shape::Fish:
        return {0.40F, 0.05F, 0.11F}; // a frozen fish lying on its side
    case Shape::Stl:
        return {0.1F, 0.1F, 0.1F};
    }
    return {0.1F, 0.1F, 0.1F};
}

const MaterialProperties& propertiesOf(Material material) {
    // Densities are of the whole item's volume: a filled bottle about
    // water's, a ball mostly air inside, a bag of crisps far less.
    static const MaterialProperties table[] = {
        {"cardboard", 0.6F, 0.0F, 1000.0F, false},     {"tinplate", 0.4F, 0.0F, 1000.0F, true},
        {"glass", 0.35F, 0.05F, 1050.0F, false},       {"film", 0.5F, 0.0F, 60.0F, false},
        {"rubber", 0.8F, 0.6F, 150.0F, false},         {"steel", 0.4F, 0.1F, 7850.0F, true},
        {"aluminium", 0.45F, 0.1F, 2700.0F, true},     {"plastic", 0.4F, 0.2F, 1050.0F, false},
        {"frozen fish", 0.06F, 0.25F, 1000.0F, false},
    };
    return table[static_cast<std::size_t>(material)];
}

std::optional<Material> materialNamed(std::string_view name) {
    for (const auto material : kMaterials) {
        if (name == propertiesOf(material).name) {
            return material;
        }
    }
    return std::nullopt;
}

Material materialOf(Shape shape) {
    switch (shape) {
    case Shape::Box:
        return Material::Cardboard;
    case Shape::Tin:
        return Material::Tinplate;
    case Shape::Bottle:
        return Material::Glass;
    case Shape::Bag:
        return Material::Film;
    case Shape::Ball:
        return Material::Rubber;
    case Shape::Bracket:
        return Material::Steel;
    case Shape::Fish:
        return Material::FrozenFish;
    case Shape::Stl:
        return Material::Plastic;
    }
    return Material::Cardboard;
}

// ---------------------------------------------------------------- The geometry

struct ItemGeometry::Jolt {
    JPH::RefConst<JPH::Shape> physics; // the convex pieces
    JPH::RefConst<JPH::Shape> surface; // the surface as a mesh, for rays (never in the physics world)
    JPH::Vec3 surfaceCenter;           // Jolt casts rays from a shape's centre of mass
};

ItemGeometry::ItemGeometry(Shape shape, Material material, Mesh surface, std::vector<std::vector<glm::vec3>> pieces,
                           float sphereRadius)
    : shape_(shape), material_(material), surface_(std::move(surface)), pieces_(std::move(pieces)),
      jolt_(std::make_unique<Jolt>()) {
    static std::atomic<std::uint64_t> next{1};
    id_ = next++;
    initialiseJolt();

    glm::vec3 low(std::numeric_limits<float>::max());
    glm::vec3 high(-std::numeric_limits<float>::max());
    for (const auto& p : surface_.positions) {
        low = glm::min(low, p);
        high = glm::max(high, p);
    }
    size_ = surface_.positions.empty() ? glm::vec3(0.0F) : high - low;

    // The physics: the convex pieces, or a sphere. Jolt keeps a small
    // rounding (the convex radius) inside each piece's outline: 2 mm.
    const float density = propertiesOf(material_).density;
    if (sphereRadius > 0.0F) {
        JPH::SphereShapeSettings sphere(sphereRadius);
        sphere.SetDensity(density);
        jolt_->physics = sphere.Create().Get();
    } else {
        std::vector<JPH::Ref<JPH::ConvexHullShapeSettings>> hulls;
        for (const auto& piece : pieces_) {
            JPH::Array<JPH::Vec3> points;
            for (const auto& p : piece) {
                points.push_back(toJolt(p));
            }
            auto hull = new JPH::ConvexHullShapeSettings(points, 0.002F);
            hull->SetDensity(density);
            if (hull->Create().IsValid()) { // flat or too few points: not a piece
                hulls.emplace_back(hull);
            } else {
                delete hull;
            }
        }
        if (hulls.size() == 1) {
            jolt_->physics = hulls.front()->Create().Get();
        } else if (hulls.size() > 1) {
            JPH::StaticCompoundShapeSettings compound;
            for (const auto& hull : hulls) {
                compound.AddShape(JPH::Vec3::sZero(), JPH::Quat::sIdentity(), hull);
            }
            jolt_->physics = compound.Create().Get();
        }
        if (jolt_->physics == nullptr) { // nothing usable: its bounding box
            JPH::BoxShapeSettings box(toJolt(glm::max(size_, glm::vec3(0.01F)) / 2.0F), 0.002F);
            box.SetDensity(density);
            jolt_->physics = box.Create().Get();
        }
    }
    const auto massProperties = jolt_->physics->GetMassProperties();
    mass_ = massProperties.mMass;
    volume_ = jolt_->physics->GetVolume();
    centerOfMass_ = toGlm(jolt_->physics->GetCenterOfMass());

    // The convex hull of the surface: its corners are enough to know how
    // far it reaches in any direction.
    JPH::Array<JPH::Vec3> all;
    for (const auto& p : surface_.positions) {
        all.push_back(toJolt(p));
    }
    const auto hull = JPH::ConvexHullShapeSettings(all, 0.0F).Create();
    if (hull.IsValid()) {
        const auto* convex = static_cast<const JPH::ConvexHullShape*>(hull.Get().GetPtr());
        const auto center = toGlm(convex->GetCenterOfMass());
        for (JPH::uint i = 0; i < convex->GetNumPoints(); ++i) {
            hull_.push_back(toGlm(convex->GetPoint(i)) + center);
        }
    } else {
        for (int corner = 0; corner < 8; ++corner) {
            hull_.push_back(
                glm::vec3(corner & 1 ? high.x : low.x, corner & 2 ? high.y : low.y, corner & 4 ? high.z : low.z));
        }
    }

    // The surface as a Jolt mesh, which sorts its triangles into a tree, so
    // a ray only tests the few near it.
    JPH::TriangleList triangles;
    for (std::size_t i = 0; i + 2 < surface_.positions.size(); i += 3) {
        const auto& p = surface_.positions;
        triangles.push_back(JPH::Triangle(JPH::Float3(p[i].x, p[i].y, p[i].z),
                                          JPH::Float3(p[i + 1].x, p[i + 1].y, p[i + 1].z),
                                          JPH::Float3(p[i + 2].x, p[i + 2].y, p[i + 2].z)));
    }
    const auto mesh = JPH::MeshShapeSettings(triangles).Create();
    if (mesh.IsValid()) {
        jolt_->surface = mesh.Get();
        jolt_->surfaceCenter = jolt_->surface->GetCenterOfMass();
    }
}

ItemGeometry::~ItemGeometry() = default;

const JPH::Shape* ItemGeometry::physicsShape() const {
    return jolt_->physics.GetPtr();
}

std::pair<float, float> ItemGeometry::span(const glm::mat3& rotation, glm::vec3 direction) const {
    const auto local = glm::transpose(rotation) * direction; // the direction in its own axes
    float low = std::numeric_limits<float>::max();
    float high = -std::numeric_limits<float>::max();
    for (const auto& p : hull_) {
        const float d = glm::dot(p, local);
        low = std::min(low, d);
        high = std::max(high, d);
    }
    return {low, high};
}

void ItemGeometry::slice(glm::vec3 center, const glm::mat3& rotation, float y, glm::vec3 axis,
                         std::vector<std::pair<float, float>>& out) const {
    // Each corner's height above the plane, and its place along the axis.
    const auto up = glm::transpose(rotation) * glm::vec3(0.0F, 1.0F, 0.0F);
    const auto along = glm::transpose(rotation) * axis;
    const float base = center.y - y;
    const float offset = glm::dot(center, axis);
    std::vector<std::pair<float, float>> pieces;
    const auto& p = surface_.positions;
    for (std::size_t i = 0; i + 2 < p.size(); i += 3) {
        const float h[3] = {base + glm::dot(p[i], up), base + glm::dot(p[i + 1], up), base + glm::dot(p[i + 2], up)};
        if ((h[0] < 0.0F) == (h[1] < 0.0F) && (h[1] < 0.0F) == (h[2] < 0.0F)) {
            continue; // all on one side
        }
        // The two edges that cross the plane meet it at the ends of a
        // segment; the segment's stretch of the axis is covered.
        float from = std::numeric_limits<float>::max();
        float to = -std::numeric_limits<float>::max();
        for (int e = 0; e < 3; ++e) {
            const int a = e;
            const int b = (e + 1) % 3;
            if ((h[a] < 0.0F) != (h[b] < 0.0F)) {
                const float t = h[a] / (h[a] - h[b]);
                const float at = glm::dot(
                    glm::mix(p[i + static_cast<std::size_t>(a)], p[i + static_cast<std::size_t>(b)], t), along);
                from = std::min(from, at);
                to = std::max(to, at);
            }
        }
        pieces.emplace_back(offset + from, offset + to);
    }
    // The cross-section's outline covers exactly what the cross-section does
    // (along any line across it, it starts and ends on the outline): merged
    // into as few stretches as there are, gaps kept.
    std::sort(pieces.begin(), pieces.end());
    std::vector<std::pair<float, float>> merged;
    for (const auto& piece : pieces) {
        if (!merged.empty() && piece.first <= merged.back().second) {
            merged.back().second = std::max(merged.back().second, piece.second);
        } else {
            merged.push_back(piece);
        }
    }
    out.insert(out.end(), merged.begin(), merged.end());
}

std::optional<float> ItemGeometry::castRay(glm::vec3 center, const glm::mat3& rotation, glm::vec3 origin,
                                           glm::vec3 direction, float length, glm::vec3* normal) const {
    if (jolt_->surface == nullptr) {
        return std::nullopt;
    }
    const auto from = glm::transpose(rotation) * (origin - center);
    const auto way = glm::transpose(rotation) * direction * length;
    const JPH::RayCast ray(toJolt(from) - jolt_->surfaceCenter, toJolt(way));
    JPH::RayCastResult hit;
    if (!jolt_->surface->CastRay(ray, JPH::SubShapeIDCreator(), hit)) {
        return std::nullopt;
    }
    if (normal != nullptr) {
        // The triangle's normal, turned to face the ray (a ray can hit a
        // triangle from behind), back in the world's axes.
        auto n = toGlm(jolt_->surface->GetSurfaceNormal(hit.mSubShapeID2, ray.GetPointOnRay(hit.mFraction)));
        if (glm::dot(n, way) > 0.0F) {
            n = -n;
        }
        *normal = rotation * n;
    }
    return hit.mFraction * length;
}

// ---------------------------------------------------------------- Building surfaces

namespace {

// Corners and triangles (three indices each) before the normals are worked out.
struct Indexed {
    std::vector<glm::vec3> corners;
    std::vector<std::array<std::size_t, 3>> triangles;
    std::vector<bool> smooth; // per triangle: its corners' normals are the surface's, not the face's
};

// The volume a closed surface encloses, positive when its triangles turn
// counter-clockwise seen from outside (the sum of the tetrahedra from the
// origin to each triangle).
float signedVolume(const Indexed& mesh) {
    float volume = 0.0F;
    for (const auto& t : mesh.triangles) {
        volume += glm::dot(mesh.corners[t[0]], glm::cross(mesh.corners[t[1]], mesh.corners[t[2]])) / 6.0F;
    }
    return volume;
}

// Turned outward if it was inward, then each corner's normal: on a smooth
// part, the average of the faces around it (weighted by their area, which
// the cross product's length is), so the light makes it look round.
void appendMesh(Indexed mesh, Mesh& out) {
    if (signedVolume(mesh) < 0.0F) {
        for (auto& t : mesh.triangles) {
            std::swap(t[1], t[2]);
        }
    }
    std::vector<glm::vec3> normals(mesh.corners.size(), glm::vec3(0.0F));
    for (std::size_t k = 0; k < mesh.triangles.size(); ++k) {
        if (mesh.smooth[k]) {
            const auto& t = mesh.triangles[k];
            const auto n = glm::cross(mesh.corners[t[1]] - mesh.corners[t[0]], mesh.corners[t[2]] - mesh.corners[t[0]]);
            for (const auto i : t) {
                normals[i] += n;
            }
        }
    }
    for (std::size_t k = 0; k < mesh.triangles.size(); ++k) {
        const auto& t = mesh.triangles[k];
        const auto face = glm::cross(mesh.corners[t[1]] - mesh.corners[t[0]], mesh.corners[t[2]] - mesh.corners[t[0]]);
        const float area = glm::length(face);
        if (area < 1e-12F) {
            continue; // degenerate: draws nothing, and has no normal
        }
        for (const auto i : t) {
            const auto n = mesh.smooth[k] && glm::length(normals[i]) > 0.0F ? glm::normalize(normals[i]) : face / area;
            out.positions.push_back(mesh.corners[i]);
            out.normals.push_back(n);
        }
    }
}

// A surface swept through rings (closed loops with the same number of
// points), each joined to the next, and closed with a flat lid at each end.
Indexed loft(const std::vector<std::vector<glm::vec3>>& rings) {
    Indexed mesh;
    const std::size_t n = rings.front().size();
    for (const auto& ring : rings) {
        mesh.corners.insert(mesh.corners.end(), ring.begin(), ring.end());
    }
    const auto at = [&](std::size_t ring, std::size_t i) { return ring * n + i % n; };
    for (std::size_t k = 0; k + 1 < rings.size(); ++k) {
        for (std::size_t i = 0; i < n; ++i) {
            mesh.triangles.push_back({at(k, i), at(k, i + 1), at(k + 1, i + 1)});
            mesh.triangles.push_back({at(k, i), at(k + 1, i + 1), at(k + 1, i)});
            mesh.smooth.insert(mesh.smooth.end(), {true, true});
        }
    }
    // The lids: a fan from each end ring's middle, turned to face away from
    // the ring next to it. They're flat, with corners of their own.
    for (const bool last : {false, true}) {
        const std::size_t k = last ? rings.size() - 1 : 0;
        const auto& ring = rings[k];
        glm::vec3 middle(0.0F);
        for (const auto& p : ring) {
            middle += p;
        }
        middle /= static_cast<float>(n);
        glm::vec3 next(0.0F);
        for (const auto& p : rings[last ? k - 1 : 1]) {
            next += p;
        }
        next /= static_cast<float>(n);
        const std::size_t first = mesh.corners.size();
        mesh.corners.insert(mesh.corners.end(), ring.begin(), ring.end());
        mesh.corners.push_back(middle);
        for (std::size_t i = 0; i < n; ++i) {
            std::array<std::size_t, 3> t{first + n, first + i, first + (i + 1) % n};
            const auto face =
                glm::cross(mesh.corners[t[1]] - mesh.corners[t[0]], mesh.corners[t[2]] - mesh.corners[t[0]]);
            if (glm::dot(face, middle - next) < 0.0F) {
                std::swap(t[1], t[2]);
            }
            mesh.triangles.push_back(t);
            mesh.smooth.push_back(false);
        }
    }
    // The sides' winding follows the rings' direction, which may be either
    // way: made to face outward, away from the line through the rings'
    // middles, if most of them don't.
    std::vector<glm::vec3> middles;
    for (const auto& r : rings) {
        glm::vec3 m(0.0F);
        for (const auto& p : r) {
            m += p;
        }
        middles.push_back(m / static_cast<float>(n));
    }
    float outward = 0.0F;
    for (std::size_t k = 0; k + 1 < rings.size(); ++k) {
        for (std::size_t i = 0; i < 2 * n; ++i) {
            const auto& t = mesh.triangles[k * 2 * n + i];
            const auto face =
                glm::cross(mesh.corners[t[1]] - mesh.corners[t[0]], mesh.corners[t[2]] - mesh.corners[t[0]]);
            const auto centre = (mesh.corners[t[0]] + mesh.corners[t[1]] + mesh.corners[t[2]]) / 3.0F;
            outward += glm::dot(face, centre - (middles[k] + middles[k + 1]) / 2.0F);
        }
    }
    if (outward < 0.0F) {
        for (std::size_t k = 0; k < mesh.triangles.size(); ++k) {
            if (mesh.smooth[k]) {
                std::swap(mesh.triangles[k][1], mesh.triangles[k][2]);
            }
        }
    }
    return mesh;
}

// A box's six faces, flat.
Indexed boxMesh(glm::vec3 low, glm::vec3 high) {
    Indexed mesh;
    for (int axis = 0; axis < 3; ++axis) {
        for (const bool top : {false, true}) {
            const int u = (axis + 1) % 3;
            const int v = (axis + 2) % 3;
            const std::size_t first = mesh.corners.size();
            for (const auto [a, b] : {std::pair{0, 0}, std::pair{1, 0}, std::pair{1, 1}, std::pair{0, 1}}) {
                glm::vec3 p;
                p[axis] = top ? high[axis] : low[axis];
                p[u] = a != 0 ? high[u] : low[u];
                p[v] = b != 0 ? high[v] : low[v];
                mesh.corners.push_back(p);
            }
            // Counter-clockwise seen from outside: u then v on the top side.
            if (top) {
                mesh.triangles.push_back({first, first + 1, first + 2});
                mesh.triangles.push_back({first, first + 2, first + 3});
            } else {
                mesh.triangles.push_back({first, first + 2, first + 1});
                mesh.triangles.push_back({first, first + 3, first + 2});
            }
            mesh.smooth.insert(mesh.smooth.end(), {false, false});
        }
    }
    return mesh;
}

std::vector<glm::vec3> boxCorners(glm::vec3 low, glm::vec3 high) {
    std::vector<glm::vec3> corners;
    for (int c = 0; c < 8; ++c) {
        corners.emplace_back(c & 1 ? high.x : low.x, c & 2 ? high.y : low.y, c & 4 ? high.z : low.z);
    }
    return corners;
}

// A ring of `n` points around `middle`: `ru` along `u`, `rv` along `v`. A
// `squareness` above 1 makes it fuller towards the corners (a superellipse),
// still convex.
std::vector<glm::vec3> ring(glm::vec3 middle, glm::vec3 u, glm::vec3 v, float ru, float rv, int n,
                            float squareness = 1.0F) {
    std::vector<glm::vec3> points;
    for (int i = 0; i < n; ++i) {
        const float a = 2.0F * kPi * static_cast<float>(i) / static_cast<float>(n);
        const float c = std::cos(a);
        const float s = std::sin(a);
        const float cu = std::copysign(std::pow(std::abs(c), 1.0F / squareness), c);
        const float sv = std::copysign(std::pow(std::abs(s), 1.0F / squareness), s);
        points.push_back(middle + u * (ru * cu) + v * (rv * sv));
    }
    return points;
}

std::vector<glm::vec3> joined(const std::vector<std::vector<glm::vec3>>& rings, std::size_t from, std::size_t to) {
    std::vector<glm::vec3> points;
    for (std::size_t k = from; k <= to && k < rings.size(); ++k) {
        points.insert(points.end(), rings[k].begin(), rings[k].end());
    }
    return points;
}

const glm::vec3 kX{1.0F, 0.0F, 0.0F};
const glm::vec3 kY{0.0F, 1.0F, 0.0F};
const glm::vec3 kZ{0.0F, 0.0F, 1.0F};

// A bottle standing up: a round body, a shoulder, a neck. Its body and
// shoulder are one convex piece, its neck another (the narrowing then the
// straight neck isn't convex together).
std::shared_ptr<const ItemGeometry> bottle(glm::vec3 size) {
    // (height from the bottom, radius) as fractions of the bottle's.
    const std::pair<float, float> profile[] = {{0.0F, 0.88F},  {0.03F, 1.0F},  {0.58F, 1.0F}, {0.68F, 0.86F},
                                               {0.78F, 0.45F}, {0.80F, 0.38F}, {1.0F, 0.38F}};
    std::vector<std::vector<glm::vec3>> rings;
    for (const auto& [h, r] : profile) {
        rings.push_back(ring(kY * (size.y * (h - 0.5F)), kX, kZ, r * size.x / 2.0F, r * size.z / 2.0F, 20));
    }
    Mesh surface;
    appendMesh(loft(rings), surface);
    return std::make_shared<ItemGeometry>(
        Shape::Bottle, Material::Glass, std::move(surface),
        std::vector<std::vector<glm::vec3>>{joined(rings, 0, 4), joined(rings, 4, 6)});
}

// A bag of crisps lying flat: full in the middle, sealed flat at both ends.
std::shared_ptr<const ItemGeometry> bag(glm::vec3 size) {
    std::vector<std::vector<glm::vec3>> rings;
    constexpr int kStations = 13;
    for (int k = 0; k < kStations; ++k) {
        const float u = -1.0F + 2.0F * static_cast<float>(k) / (kStations - 1); // -1 .. 1 along it
        const float thickness = size.y * std::max(0.05F, std::sqrt(std::max(0.0F, 1.0F - std::pow(u, 4.0F))));
        rings.push_back(ring(kX * (u * size.x / 2.0F), kZ, kY, size.z / 2.0F, thickness / 2.0F, 20, 2.0F));
    }
    Mesh surface;
    appendMesh(loft(rings), surface);
    return std::make_shared<ItemGeometry>(Shape::Bag, Material::Film, std::move(surface),
                                          std::vector<std::vector<glm::vec3>>{joined(rings, 0, rings.size() - 1)});
}

// A ball: drawn as rings of latitude, a true sphere to the physics.
std::shared_ptr<const ItemGeometry> ball(glm::vec3 size) {
    const float radius = size.x / 2.0F;
    std::vector<std::vector<glm::vec3>> rings;
    constexpr int kRings = 14;
    for (int k = 0; k <= kRings; ++k) {
        const float latitude = kPi * (-0.5F + static_cast<float>(k) / kRings);
        const float r = std::max(radius * std::cos(latitude), radius * 1e-3F);
        rings.push_back(ring(kY * (radius * std::sin(latitude)), kX, kZ, r, r, 24));
    }
    Mesh surface;
    appendMesh(loft(rings), surface);
    return std::make_shared<ItemGeometry>(Shape::Ball, Material::Rubber, std::move(surface),
                                          std::vector<std::vector<glm::vec3>>{}, radius);
}

// An angle bracket: a plate lying on the belt, and one standing up at its
// back end. Its weight isn't in the middle.
std::shared_ptr<const ItemGeometry> bracket(glm::vec3 size) {
    const glm::vec3 low = -size / 2.0F;
    const glm::vec3 high = size / 2.0F;
    const float plate = std::clamp(0.1F * std::min(size.x, size.y), 0.003F, 0.008F);
    const glm::vec3 baseHigh{high.x, low.y + plate, high.z};
    const glm::vec3 uprightHigh{low.x + plate, high.y, high.z};
    Mesh surface;
    appendMesh(boxMesh(low, baseHigh), surface);
    appendMesh(boxMesh(low, uprightHigh), surface);
    return std::make_shared<ItemGeometry>(
        Shape::Bracket, Material::Steel, std::move(surface),
        std::vector<std::vector<glm::vec3>>{boxCorners(low, baseHigh), boxCorners(low, uprightHigh)});
}

// A frozen fish lying on its side, head first (towards +x): its thickness
// is the height, its depth from back to belly the width. Thick at the
// head, thin at the tail, then a flat tail fin. Each stretch between two
// cross-sections is a convex piece.
std::shared_ptr<const ItemGeometry> fish(glm::vec3 size) {
    struct Station {
        float u;     // from the snout (0) to the tail's end (1)
        float depth; // as a fraction of the width
        float thick; // as a fraction of the height
    };
    const Station stations[] = {{0.0F, 0.12F, 0.2F}, {0.05F, 0.5F, 0.6F},   {0.16F, 0.9F, 0.95F},
                                {0.30F, 1.0F, 1.0F}, {0.48F, 0.85F, 0.75F}, {0.66F, 0.48F, 0.42F},
                                {0.80F, 0.2F, 0.2F}, {0.85F, 0.3F, 0.08F},  {1.0F, 0.85F, 0.06F}};
    const float fin = std::max(0.004F, 0.06F * size.y); // the fin's thickness, at least 4 mm
    std::vector<std::vector<glm::vec3>> rings;
    for (const auto& s : stations) {
        const float thickness = std::max(s.thick * size.y, fin);
        rings.push_back(ring(kX * (size.x * (0.5F - s.u)), kZ, kY, s.depth * size.z / 2.0F, thickness / 2.0F, 18));
    }
    Mesh surface;
    appendMesh(loft(rings), surface);
    std::vector<std::vector<glm::vec3>> pieces;
    for (std::size_t k = 0; k + 1 < rings.size(); ++k) {
        pieces.push_back(joined(rings, k, k + 1));
    }
    return std::make_shared<ItemGeometry>(Shape::Fish, Material::FrozenFish, std::move(surface), std::move(pieces));
}

} // namespace

std::shared_ptr<const ItemGeometry> builtInGeometry(Shape shape, glm::vec3 size) {
    switch (shape) {
    case Shape::Bottle:
        return bottle(size);
    case Shape::Bag:
        return bag(size);
    case Shape::Ball:
        return ball(size);
    case Shape::Bracket:
        return bracket(size);
    case Shape::Fish:
        return fish(size);
    case Shape::Box:
    case Shape::Tin:
    case Shape::Stl:
        break;
    }
    return nullptr;
}

// ---------------------------------------------------------------- STL files

tl::expected<std::vector<glm::vec3>, std::string> readStl(const std::filesystem::path& file) {
    std::ifstream in(file, std::ios::binary);
    if (!in) {
        return tl::unexpected(fmt::format("can't open {}", file.string()));
    }
    const std::string data((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    std::vector<glm::vec3> corners;
    // Binary: an 80-byte header, a 4-byte triangle count, then 50 bytes per
    // triangle (a normal and three corners as 32-bit floats, two spare
    // bytes). Text starts with "solid", but so do some binary headers, so
    // the size decides.
    if (data.size() >= 84) {
        std::uint32_t count = 0;
        std::memcpy(&count, data.data() + 80, 4);
        if (data.size() == 84 + std::size_t{count} * 50) {
            for (std::uint32_t t = 0; t < count; ++t) {
                const char* record = data.data() + 84 + std::size_t{t} * 50;
                for (int c = 0; c < 3; ++c) {
                    float xyz[3];
                    std::memcpy(xyz, record + 12 + c * 12, 12);
                    corners.emplace_back(xyz[0], xyz[1], xyz[2]);
                }
            }
            return corners;
        }
    }
    if (data.rfind("solid", 0) != 0) {
        return tl::unexpected(fmt::format("{} isn't an STL file", file.filename().string()));
    }
    // Text: only the corners matter ("vertex x y z"); normals are worked out again.
    std::istringstream text(data);
    std::string word;
    while (text >> word) {
        if (word == "vertex") {
            glm::vec3 p;
            if (!(text >> p.x >> p.y >> p.z)) {
                return tl::unexpected(fmt::format("{}: a vertex without three numbers", file.filename().string()));
            }
            corners.push_back(p);
        }
    }
    if (corners.size() % 3 != 0) {
        return tl::unexpected(fmt::format("{}: corners that don't make whole triangles", file.filename().string()));
    }
    return corners;
}

tl::expected<std::shared_ptr<const ItemGeometry>, std::string> stlGeometry(std::vector<glm::vec3> triangles,
                                                                           Material material, int pieces) {
    // Triangles with no area (some exporters write them) are dropped.
    std::vector<glm::vec3> kept;
    for (std::size_t i = 0; i + 2 < triangles.size(); i += 3) {
        if (glm::length(glm::cross(triangles[i + 1] - triangles[i], triangles[i + 2] - triangles[i])) > 1e-12F) {
            kept.insert(kept.end(), {triangles[i], triangles[i + 1], triangles[i + 2]});
        }
    }
    if (kept.size() < 12) {
        return tl::unexpected(std::string("too few triangles for a solid"));
    }
    // Its origin in the middle of its bounding box, as for any item.
    glm::vec3 low(std::numeric_limits<float>::max());
    glm::vec3 high(-std::numeric_limits<float>::max());
    for (const auto& p : kept) {
        low = glm::min(low, p);
        high = glm::max(high, p);
    }
    const auto middle = (low + high) / 2.0F;
    // The same corner is written once per triangle it belongs to; the
    // convex split needs each corner once, with triangles pointing to it.
    Indexed mesh;
    std::map<std::array<float, 3>, std::size_t> index;
    for (std::size_t i = 0; i < kept.size(); i += 3) {
        std::array<std::size_t, 3> t{};
        for (std::size_t c = 0; c < 3; ++c) {
            const auto p = kept[i + c] - middle;
            const auto [it, added] = index.try_emplace({p.x, p.y, p.z}, mesh.corners.size());
            if (added) {
                mesh.corners.push_back(p);
            }
            t[c] = it->second;
        }
        mesh.triangles.push_back(t);
        mesh.smooth.push_back(false); // a CAD mesh's faces are drawn as they are
    }
    Mesh surface;
    appendMesh(mesh, surface); // turned outward if the file had it inside out
    std::vector<std::vector<glm::vec3>> convex;
    if (pieces > 1) {
        std::vector<std::uint32_t> indices;
        for (const auto& t : mesh.triangles) {
            for (const auto i : t) {
                indices.push_back(static_cast<std::uint32_t>(i));
            }
        }
        convex = convexPieces(mesh.corners, indices, pieces);
    }
    if (convex.empty()) {
        convex.push_back(mesh.corners); // one piece: its convex hull
    }
    return std::make_shared<ItemGeometry>(Shape::Stl, material, std::move(surface), std::move(convex));
}

// ---------------------------------------------------------------- The shared geometries

namespace {

struct Library {
    std::mutex mutex;
    std::vector<std::filesystem::path> folders;
    // Built-in shapes by shape and size in whole millimetres: an item a
    // fraction of a millimetre taller is the same shape (and a spawner
    // varying its items' heights makes a few dozen, not thousands).
    std::map<std::array<long, 4>, std::shared_ptr<const ItemGeometry>> builtIn;
    struct Read {
        std::filesystem::file_time_type changed;
        std::shared_ptr<const ItemGeometry> geometry;
    };
    std::map<std::string, Read> stl; // by file, unit, material and pieces
};

Library& library() {
    static Library instance;
    return instance;
}

} // namespace

void setShapeFolders(std::vector<std::filesystem::path> folders) {
    auto& lib = library();
    const std::lock_guard lock(lib.mutex);
    lib.folders = std::move(folders);
}

std::filesystem::path findShapeFile(const std::string& file) {
    std::error_code error;
    const std::filesystem::path path(file);
    if (path.is_absolute()) {
        return std::filesystem::exists(path, error) ? path : std::filesystem::path();
    }
    auto& lib = library();
    const std::lock_guard lock(lib.mutex);
    for (const auto& folder : lib.folders) {
        if (std::filesystem::exists(folder / path, error)) {
            return folder / path;
        }
    }
    return {};
}

tl::expected<std::shared_ptr<const ItemGeometry>, std::string> itemGeometry(Shape shape, glm::vec3 size,
                                                                            const StlSpec& stl) {
    if (shape == Shape::Box || shape == Shape::Tin) {
        return nullptr;
    }
    auto& lib = library();
    if (shape != Shape::Stl) {
        if (shape == Shape::Ball) {
            size = glm::vec3(size.x); // round every way
        } else if (shape == Shape::Bottle) {
            size.z = size.x;
        }
        size = glm::max(glm::round(size * 1000.0F) / 1000.0F, glm::vec3(0.005F));
        const std::array<long, 4> key{static_cast<long>(shape), std::lround(size.x * 1000.0F),
                                      std::lround(size.y * 1000.0F), std::lround(size.z * 1000.0F)};
        const std::lock_guard lock(lib.mutex);
        auto& geometry = lib.builtIn[key];
        if (geometry == nullptr) {
            geometry = builtInGeometry(shape, size);
        }
        return geometry;
    }
    if (stl.file.empty()) {
        return tl::unexpected(std::string("no STL file chosen"));
    }
    const auto path = findShapeFile(stl.file);
    if (path.empty()) {
        return tl::unexpected(fmt::format("can't find {}", stl.file));
    }
    std::error_code error;
    const auto changed = std::filesystem::last_write_time(path, error);
    const auto key = fmt::format("{}|{}|{}|{}|{}", std::filesystem::absolute(path).string(), stl.unit,
                                 static_cast<int>(stl.material), stl.pieces, stl.zUp);
    {
        const std::lock_guard lock(lib.mutex);
        const auto found = lib.stl.find(key);
        if (found != lib.stl.end() && found->second.changed == changed) {
            return found->second.geometry;
        }
    }
    auto triangles = readStl(path);
    if (!triangles) {
        return tl::unexpected(triangles.error());
    }
    for (auto& p : *triangles) {
        p *= stl.unit;
        if (stl.zUp) {
            p = glm::vec3(p.x, p.z, -p.y); // most CAD programs have z up; here y is
        }
    }
    auto geometry = stlGeometry(std::move(*triangles), stl.material, stl.pieces);
    if (!geometry) {
        return tl::unexpected(fmt::format("{}: {}", path.filename().string(), geometry.error()));
    }
    const std::lock_guard lock(lib.mutex);
    lib.stl[key] = Library::Read{changed, *geometry};
    return *geometry;
}

} // namespace fsim
