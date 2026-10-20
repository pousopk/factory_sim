#pragma once
#include "sim/placement.hpp"
#include <array>
#include <glm/glm.hpp>
#include <optional>
#include <vector>

// The 3D handles on the selection: three arrows to move it along an axis,
// or three rings to turn it about one, as in CAD programs.
//
// It's only maths: where the handles are on the screen, which one the
// mouse is on, and what a drag along an arrow or around a ring means in
// metres or radians. The app draws it and feeds it the mouse; no window or
// OpenGL here, so it's tested on its own.
//
//   moving:  the mouse ray's closest point to the arrow's line, from where
//            the drag began, is how far along the axis it has gone;
//   turning: where the mouse ray meets the ring's plane, seen from the
//            centre, has turned by some angle since the drag began.
namespace fsim {

enum class GizmoMode { Move, Turn };

struct GizmoHandle {
    GizmoMode mode = GizmoMode::Move;
    int axis = 0; // 0 x (red), 1 y (green), 2 z (blue)

    bool operator==(const GizmoHandle&) const = default;
};

struct Gizmo {
    Frame frame;       // where it is, and its axes (the selection's own, or the world's)
    float size = 0.3F; // the arrows' length and the rings' radius, in metres
    std::array<bool, 3> canMove{true, true, true};
    std::array<bool, 3> canTurn{true, true, true};

    glm::vec3 axis(int i) const; // the axis in the world, a unit vector
};

// The arrows' length (or rings' radius) that looks `pixels` long on a view
// `viewHeight` pixels high, with a vertical field of view `fieldOfView`, at
// `distance` from the camera: the gizmo stays the same size on the screen.
float gizmoSize(float distance, float fieldOfView, float viewHeight, float pixels = 90.0F);

// A view: world points to pixels in it (x right, y down), or nothing for a
// point behind the camera.
struct GizmoView {
    glm::mat4 viewProjection{1.0F};
    glm::vec2 size{1.0F};
    std::optional<glm::vec2> project(glm::vec3 point) const;
};

// The points a handle is drawn through, on the screen: an arrow's two ends,
// or a ring's points all round (closed).
std::vector<glm::vec2> handleOutline(const Gizmo& gizmo, GizmoHandle handle, const GizmoView& view);
static constexpr int kRingPoints = 48;

// The handle of `mode` the mouse (at `mouse`, in the view's pixels) is on:
// the nearest one within `reach` pixels of its outline.
std::optional<GizmoHandle> handleAt(const Gizmo& gizmo, GizmoMode mode, glm::vec2 mouse, const GizmoView& view,
                                    float reach = 8.0F);

// How far along the line through `origin` in `axis` (a unit vector) the
// point nearest the ray is. Nothing if the ray runs (nearly) along it,
// where the nearest point is undefined.
std::optional<float> alongAxis(glm::vec3 origin, glm::vec3 axis, glm::vec3 rayOrigin, glm::vec3 rayDirection);

// Where the ray meets the plane through `origin` square to `axis`, as a
// direction from `origin` in that plane (not normalized). Nothing if the
// ray is (nearly) parallel to the plane, which then shows edge-on.
std::optional<glm::vec3> onRingPlane(glm::vec3 origin, glm::vec3 axis, glm::vec3 rayOrigin, glm::vec3 rayDirection);

// The angle from `from` to `to` (both in the plane square to `axis`),
// positive turning anticlockwise about `axis` (the right-hand rule).
float angleBetween(glm::vec3 from, glm::vec3 to, glm::vec3 axis);

// A drag's amount in whole steps: 1 cm (or 1 mm, fine), 15 degrees (or 1).
float snapped(float value, float step);

} // namespace fsim
