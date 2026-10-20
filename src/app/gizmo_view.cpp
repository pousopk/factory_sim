// The gizmo on the selection, in the 3D view: arrows to move it along an
// axis (W), rings to turn it about one (E), along its own axes or the
// world's (T). Its maths is in edit/gizmo.hpp; this draws it over the view
// and turns mouse moves into placements (edit/placing.hpp). A drag is
// drawn on a preview, as other drags are, and is one undo step.
#include "app/app.hpp"
#include "app/widgets.hpp"
#include <cmath>
#include <fmt/format.h>
#include <imgui.h>
#include <limits>
#include <numbers>

namespace fsim {

namespace {

constexpr float kDegree = std::numbers::pi_v<float> / 180.0F;

ImU32 axisColor(int axis, bool lit) {
    if (lit) {
        return IM_COL32(250, 210, 60, 255);
    }
    constexpr ImU32 colors[] = {IM_COL32(230, 80, 80, 255), IM_COL32(110, 200, 90, 255), IM_COL32(80, 140, 240, 255)};
    return colors[axis];
}

} // namespace

GizmoView App::gizmoView() const {
    const float aspect = viewSize_.y > 0.0F ? viewSize_.x / viewSize_.y : 1.0F;
    return GizmoView{camera_.projection(aspect) * camera_.view(), glm::vec2(viewSize_.x, viewSize_.y)};
}

std::optional<Gizmo> App::currentGizmo() const {
    if (gizmoDrag_) {
        return gizmoDrag_->gizmo; // it stays where the drag began; the preview shows the move
    }
    if (!selected_ || placing_ || drag_) {
        return std::nullopt;
    }
    const auto target = gizmoTarget(history_.current(), *selected_);
    if (!target) {
        return std::nullopt;
    }
    Gizmo gizmo;
    gizmo.frame = target->frame;
    // On the device itself, not on its usual place (which for a pusher or a
    // bin is out on the belt's centre line): the middle of its parts.
    if (selected_->kind != Pick::Kind::Conveyor) {
        glm::vec3 low(std::numeric_limits<float>::max());
        glm::vec3 high(-std::numeric_limits<float>::max());
        for (const auto& box : sceneBoxes(*world_)) {
            if (box.pick == *selected_) {
                low = glm::min(low, box.center);
                high = glm::max(high, box.center);
            }
        }
        if (low.x <= high.x) {
            gizmo.frame.origin = (low + high) / 2.0F;
        }
    }
    if (gizmoWorldAxes_) {
        gizmo.frame.rotation = glm::quat(1.0F, 0.0F, 0.0F, 0.0F);
    }
    gizmo.canMove = target->canMove;
    gizmo.canTurn = target->canTurn;
    gizmo.size =
        gizmoSize(glm::length(camera_.position() - gizmo.frame.origin), OrbitCamera::kFieldOfView, viewSize_.y);
    return gizmo;
}

bool App::gizmoMouse(const OrbitCamera::Ray& ray, bool pressed, bool held) {
    if (gizmoDrag_ && !held) {
        gizmoDrag_.reset(); // let go: the next frame commits it as one undo step, and the world follows
        preview_.reset();
        return true;
    }
    gizmoHover_.reset();
    const auto gizmo = currentGizmo();
    if (!gizmo) {
        return false;
    }
    if (!gizmoDrag_) {
        const auto& io = ImGui::GetIO();
        const glm::vec2 mouse(io.MousePos.x - viewMin_.x, io.MousePos.y - viewMin_.y);
        gizmoHover_ = handleAt(*gizmo, gizmoMode_, mouse, gizmoView());
        if (!gizmoHover_ || !pressed) {
            return gizmoHover_.has_value();
        }
        // A drag begins: where the mouse is on the arrow, or on the ring's plane.
        GizmoDrag drag{*gizmoHover_, *selected_, *gizmo, history_.current(), 0.0F, {}, {}};
        const auto origin = gizmo->frame.origin;
        const auto axis = gizmo->axis(drag.handle.axis);
        if (drag.handle.mode == GizmoMode::Move) {
            const auto at = alongAxis(origin, axis, ray.origin, ray.direction);
            if (!at) {
                return true;
            }
            drag.startAlong = *at;
        } else {
            const auto at = onRingPlane(origin, axis, ray.origin, ray.direction);
            if (!at) {
                return true;
            }
            drag.startOnPlane = *at;
        }
        gizmoDrag_ = std::move(drag);
        return true;
    }
    // Dragging: from where it began, by whole steps (Shift: fine ones).
    auto& drag = *gizmoDrag_;
    gizmoHover_ = drag.handle;
    const bool fine = ImGui::GetIO().KeyShift;
    const auto& start = drag.gizmo.frame;
    const auto axis = drag.gizmo.axis(drag.handle.axis);
    // The selection's own frame where the drag began (the gizmo's may be the
    // world's axes): what's moved or turned.
    const auto own = gizmoTarget(drag.original, drag.what);
    if (!own) {
        gizmoDrag_.reset();
        return true;
    }
    // What the drag does to the gizmo's frame, done to the selection's: a
    // move moves it the same; a turn turns it about the gizmo's middle.
    Frame to = own->frame;
    if (drag.handle.mode == GizmoMode::Move) {
        const auto at = alongAxis(start.origin, axis, ray.origin, ray.direction);
        if (!at) {
            return true;
        }
        const float moved = snapped(*at - drag.startAlong, fine ? 0.001F : 0.01F);
        to.origin += axis * moved;
        drag.amount = fmt::format("{:+.3f} m", moved);
    } else {
        const auto at = onRingPlane(start.origin, axis, ray.origin, ray.direction);
        if (!at || glm::length(*at) < 1e-4F || glm::length(drag.startOnPlane) < 1e-4F) {
            return true; // edge-on, or through the very middle: wait for a better angle
        }
        const float turned = snapped(angleBetween(drag.startOnPlane, *at, axis), (fine ? 1.0F : 15.0F) * kDegree);
        const auto turn = glm::angleAxis(turned, axis);
        to.origin = start.origin + turn * (to.origin - start.origin);
        to.rotation = turn * to.rotation;
        drag.amount = fmt::format("{:+.0f} deg", turned / kDegree);
    }
    auto& s = scene();
    placeAt(s, drag.original, drag.what, to);
    return true;
}

void App::drawGizmo() {
    const auto gizmo = currentGizmo();
    if (!gizmo) {
        return;
    }
    auto* draw = ImGui::GetBackgroundDrawList(); // over the 3D view, under the panels
    draw->PushClipRect(viewMin_, ImVec2(viewMin_.x + viewSize_.x, viewMin_.y + viewSize_.y), true);
    const auto view = gizmoView();
    const auto toScreen = [&](glm::vec2 p) { return ImVec2(viewMin_.x + p.x, viewMin_.y + p.y); };
    const auto active = gizmoDrag_ ? std::optional<GizmoHandle>(gizmoDrag_->handle) : gizmoHover_;
    for (int axis = 0; axis < 3; ++axis) {
        const GizmoHandle handle{gizmoMode_, axis};
        if (!(gizmoMode_ == GizmoMode::Move ? gizmo->canMove : gizmo->canTurn)[static_cast<std::size_t>(axis)]) {
            continue;
        }
        const auto outline = handleOutline(*gizmo, handle, view);
        if (outline.size() < 2) {
            continue;
        }
        const bool lit = active == handle;
        const auto color = axisColor(axis, lit);
        if (gizmoMode_ == GizmoMode::Move) {
            // A shaft and a head pointing along it.
            const auto a = toScreen(outline[0]);
            const auto b = toScreen(outline[1]);
            const float dx = b.x - a.x;
            const float dy = b.y - a.y;
            const float length = std::max(std::sqrt(dx * dx + dy * dy), 1.0F);
            const ImVec2 along(dx / length, dy / length);
            const ImVec2 across(-along.y, along.x);
            draw->AddLine(a, b, color, lit ? 4.0F : 3.0F);
            draw->AddTriangleFilled(ImVec2(b.x + along.x * 14.0F, b.y + along.y * 14.0F),
                                    ImVec2(b.x + across.x * 6.0F, b.y + across.y * 6.0F),
                                    ImVec2(b.x - across.x * 6.0F, b.y - across.y * 6.0F), color);
        } else {
            std::vector<ImVec2> points;
            for (const auto& p : outline) {
                points.push_back(toScreen(p));
            }
            draw->AddPolyline(points.data(), static_cast<int>(points.size()), color, ImDrawFlags_Closed,
                              lit ? 4.0F : 2.5F);
        }
    }
    if (const auto middle = view.project(gizmo->frame.origin)) {
        draw->AddCircleFilled(toScreen(*middle), 4.0F, IM_COL32(240, 240, 240, 255));
    }
    draw->PopClipRect();
    // What the drag has done, and the keys.
    if (gizmoDrag_ && !gizmoDrag_->amount.empty()) {
        ImGui::SetTooltip("%s%s", gizmoDrag_->amount.c_str(), ImGui::GetIO().KeyShift ? "  (fine)" : "");
    }
    const auto hint = fmt::format("{}  W move  E turn  T {} axes", gizmoMode_ == GizmoMode::Move ? "moving" : "turning",
                                  gizmoWorldAxes_ ? "world" : "own");
    draw->AddText(ImVec2(viewMin_.x + 10.0F, viewMin_.y + viewSize_.y - ImGui::GetFontSize() - 8.0F),
                  ImGui::GetColorU32(kDim), hint.c_str());
}

} // namespace fsim
