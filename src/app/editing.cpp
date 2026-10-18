// Editing in the 3D view: dragging the selection (a device along its belt, a
// conveyor across the floor), placing a new device by clicking on a belt,
// copy and paste, turning a conveyor. The geometry is in edit/placing.hpp;
// this turns the mouse into it.
#include "app/app.hpp"
#include <imgui.h>
#include <numbers>

namespace fsim {

std::optional<OrbitCamera::Ray> App::mouseRay(int width, int height) const {
    const auto& io = ImGui::GetIO();
    if (!ImGui::IsMousePosValid()) {
        return std::nullopt;
    }
    // Relative to the 3D view (the dockspace's middle), in window points: the
    // ray only needs the proportions, so the framebuffer's pixels don't matter.
    (void)width;
    (void)height;
    return camera_.rayThrough(io.MousePos.x - viewMin_.x, io.MousePos.y - viewMin_.y, viewSize_.x, viewSize_.y);
}

bool App::canDrag(Pick pick) const {
    const auto& s = history_.current();
    if (pick.kind == Pick::Kind::Conveyor) {
        // Only one standing on its own: one that follows or joins another is
        // placed by it.
        return pick.index < s.conveyors.size() && !s.conveyors[pick.index].follows && !s.conveyors[pick.index].joins;
    }
    return placeOf(s, pick).has_value();
}

// `pressed`: the left button went down this frame; `held`: it's down;
// `clicked`: it came up without the mouse moving (a click, not a drag).
void App::editInView(const OrbitCamera::Ray& ray, bool pressed, bool held, bool clicked) {
    auto& s = scene();
    const auto placed = placeConveyors(s);

    if (placing_) {
        // Placing: the belt under the mouse lights up; a click puts the device there.
        const auto hit = beltUnder(placed, ray.origin, ray.direction);
        hovered_ = hit ? std::optional<Pick>(Pick{Pick::Kind::Conveyor, hit->conveyor}) : std::nullopt;
        if (hit) {
            ImGui::SetTooltip("Click to place it here: %.2f m along %s. Esc: cancel.", hit->along,
                              placed[hit->conveyor].name.c_str());
        } else {
            ImGui::SetTooltip("Click on a belt to place it. Esc: cancel.");
        }
        if (clicked && hit) {
            selected_ = addAt(s, *placing_, hit->conveyor, snap(hit->along, 0.01F));
            placing_.reset();
            hovered_.reset();
        }
        return;
    }

    // A drag starts on the selected thing itself; anywhere else, the mouse
    // turns the camera as before.
    if (pressed && selected_ && canDrag(*selected_) &&
        pickAt(sceneBoxes(*world_), ray.origin, ray.direction) == selected_) {
        Drag drag{*selected_};
        if (selected_->kind == Pick::Kind::Conveyor) {
            const auto& c = placed[selected_->index];
            if (const auto at = onPlane(ray.origin, ray.direction, c.beltHeight)) {
                drag.grabPoint = *at;
                drag.startedAt = s.conveyors[selected_->index].spec.start;
                drag_ = drag;
            }
        } else if (const auto place = placeOf(s, *selected_); place && place->first < placed.size()) {
            drag.grab = alongNearest(placed[place->first], ray.origin, ray.direction) - place->second;
            drag_ = drag;
        }
    }
    if (!drag_) {
        return;
    }
    if (!held) {
        drag_.reset(); // let go: the next frame commits it as one undo step, and the world follows
        preview_.reset();
        return;
    }
    // Hold Shift for fine steps (1 mm), otherwise 1 cm along a belt and 5 cm on the floor.
    const bool fine = ImGui::GetIO().KeyShift;
    if (drag_->what.kind == Pick::Kind::Conveyor) {
        const auto& c = placed[drag_->what.index];
        if (const auto at = onPlane(ray.origin, ray.direction, c.beltHeight)) {
            const auto moved = drag_->startedAt + (*at - drag_->grabPoint);
            auto& start = s.conveyors[drag_->what.index].spec.start;
            start.x = snap(moved.x, fine ? 0.001F : 0.05F);
            start.z = snap(moved.z, fine ? 0.001F : 0.05F);
        }
    } else if (const auto place = placeOf(s, drag_->what); place && place->first < placed.size()) {
        const float along = alongNearest(placed[place->first], ray.origin, ray.direction) - drag_->grab;
        moveAlong(s, drag_->what, snap(along, fine ? 0.001F : 0.01F));
    }
}

void App::copySelection() {
    if (selected_) {
        if (auto clip = copyOf(scene(), *selected_)) {
            clipboard_ = std::move(clip);
        }
    }
}

// Pasted onto the belt under the mouse, if there is one (a device), or next
// to the original.
void App::pasteClip(std::optional<BeltHit> at) {
    if (!clipboard_) {
        return;
    }
    if (at && !std::holds_alternative<PlacedConveyor>(*clipboard_)) {
        selected_ = paste(scene(), *clipboard_, at->conveyor, snap(at->along, 0.01F));
    } else {
        selected_ = paste(scene(), *clipboard_);
    }
}

void App::turnSelection(float quarterTurns) {
    if (selected_ && selected_->kind == Pick::Kind::Conveyor && canDrag(*selected_)) {
        auto& heading = scene().conveyors[selected_->index].spec.heading;
        heading =
            std::remainder(heading + quarterTurns * std::numbers::pi_v<float> / 2.0F, 2.0F * std::numbers::pi_v<float>);
    }
}

// What to draw: the world as it runs, or while a drag is going on, its
// items over a preview of the layout being dragged (a world built from the
// scene as it is now, which nothing runs).
std::vector<DrawBox> App::boxesToDraw() {
    const auto highlight = hovered_ ? hovered_ : selected_;
    if (!drag_) {
        return sceneBoxes(*world_, 1.0, highlight);
    }
    if (!preview_ || previewed_ != scene()) {
        preview_ = buildWorld(scene(), [](const Scene&) { return nullptr; }, 1);
        previewed_ = scene();
    }
    auto boxes = sceneBoxes(*preview_, 1.0, highlight);
    std::erase_if(boxes, [](const DrawBox& b) { return b.pick.kind == Pick::Kind::None; }); // the preview has no items
    for (const auto& box : sceneBoxes(*world_)) {
        if (box.pick.kind == Pick::Kind::None) {
            boxes.push_back(box); // the items, where they are
        }
    }
    return boxes;
}

} // namespace fsim
