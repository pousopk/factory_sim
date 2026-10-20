#include "edit/gizmo.hpp"
#include <algorithm>
#include <cmath>

namespace fsim {

namespace {

constexpr float kPi = 3.14159265F;

glm::vec3 unit(int axis) {
    glm::vec3 v(0.0F);
    v[axis] = 1.0F;
    return v;
}

// The distance from `p` to the segment from `a` to `b`, on the screen.
float toSegment(glm::vec2 p, glm::vec2 a, glm::vec2 b) {
    const auto ab = b - a;
    const float length2 = glm::dot(ab, ab);
    const float t = length2 > 0.0F ? std::clamp(glm::dot(p - a, ab) / length2, 0.0F, 1.0F) : 0.0F;
    return glm::length(p - (a + ab * t));
}

} // namespace

glm::vec3 Gizmo::axis(int i) const {
    return glm::normalize(frame.rotation * unit(i));
}

float gizmoSize(float distance, float fieldOfView, float viewHeight, float pixels) {
    // The view's height covers 2 tan(fov / 2) x distance metres there.
    const float metresPerPixel = 2.0F * std::tan(fieldOfView / 2.0F) * distance / std::max(viewHeight, 1.0F);
    return pixels * metresPerPixel;
}

std::optional<glm::vec2> GizmoView::project(glm::vec3 point) const {
    const auto clip = viewProjection * glm::vec4(point, 1.0F);
    if (clip.w <= 1e-4F) {
        return std::nullopt; // behind the camera
    }
    const glm::vec2 ndc(clip.x / clip.w, clip.y / clip.w);
    return glm::vec2((ndc.x + 1.0F) / 2.0F * size.x, (1.0F - ndc.y) / 2.0F * size.y);
}

std::vector<glm::vec2> handleOutline(const Gizmo& gizmo, GizmoHandle handle, const GizmoView& view) {
    std::vector<glm::vec2> points;
    const auto origin = gizmo.frame.origin;
    if (handle.mode == GizmoMode::Move) {
        const auto a = view.project(origin + gizmo.axis(handle.axis) * (gizmo.size * 0.15F)); // (past the middle's dot)
        const auto b = view.project(origin + gizmo.axis(handle.axis) * gizmo.size);
        if (a && b) {
            points = {*a, *b};
        }
        return points;
    }
    // A ring about the axis: the other two axes span its plane.
    const auto u = gizmo.axis((handle.axis + 1) % 3);
    const auto v = gizmo.axis((handle.axis + 2) % 3);
    for (int i = 0; i < kRingPoints; ++i) {
        const float a = 2.0F * kPi * static_cast<float>(i) / kRingPoints;
        if (const auto p = view.project(origin + (u * std::cos(a) + v * std::sin(a)) * gizmo.size)) {
            points.push_back(*p);
        } else {
            return {}; // part of it behind the camera: not drawn or picked
        }
    }
    return points;
}

std::optional<GizmoHandle> handleAt(const Gizmo& gizmo, GizmoMode mode, glm::vec2 mouse, const GizmoView& view,
                                    float reach) {
    std::optional<GizmoHandle> best;
    float nearest = reach;
    for (int axis = 0; axis < 3; ++axis) {
        if (!(mode == GizmoMode::Move ? gizmo.canMove : gizmo.canTurn)[static_cast<std::size_t>(axis)]) {
            continue;
        }
        const GizmoHandle handle{mode, axis};
        const auto outline = handleOutline(gizmo, handle, view);
        const std::size_t segments =
            mode == GizmoMode::Move ? outline.size() - (outline.empty() ? 0 : 1) : outline.size();
        for (std::size_t i = 0; i < segments; ++i) {
            const float d = toSegment(mouse, outline[i], outline[(i + 1) % outline.size()]);
            if (d <= nearest) {
                nearest = d;
                best = handle;
            }
        }
    }
    return best;
}

std::optional<float> alongAxis(glm::vec3 origin, glm::vec3 axis, glm::vec3 rayOrigin, glm::vec3 rayDirection) {
    // The two lines' closest points: with w = origin - rayOrigin, the
    // parameter t along the axis is (b e - c d) / (a c - b²) for
    // a = axis·axis, b = axis·ray, c = ray·ray, d = axis·w, e = ray·w.
    const float a = glm::dot(axis, axis);
    const float b = glm::dot(axis, rayDirection);
    const float c = glm::dot(rayDirection, rayDirection);
    const auto w = origin - rayOrigin;
    const float d = glm::dot(axis, w);
    const float e = glm::dot(rayDirection, w);
    const float denominator = a * c - b * b;
    if (denominator < 1e-4F * a * c) {
        return std::nullopt; // looking along the axis
    }
    return (b * e - c * d) / denominator;
}

std::optional<glm::vec3> onRingPlane(glm::vec3 origin, glm::vec3 axis, glm::vec3 rayOrigin, glm::vec3 rayDirection) {
    const float facing = glm::dot(rayDirection, axis);
    if (std::abs(facing) < 0.05F) {
        return std::nullopt; // the plane edge-on: where the ray meets it jumps about
    }
    const float t = glm::dot(origin - rayOrigin, axis) / facing;
    if (t < 0.0F) {
        return std::nullopt;
    }
    return rayOrigin + rayDirection * t - origin;
}

float angleBetween(glm::vec3 from, glm::vec3 to, glm::vec3 axis) {
    return std::atan2(glm::dot(axis, glm::cross(from, to)), glm::dot(from, to));
}

float snapped(float value, float step) {
    return step > 0.0F ? std::round(value / step) * step : value;
}

} // namespace fsim
