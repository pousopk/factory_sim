// Where the selected device is: on its belt as usual, moved from there, or
// standing on its own; and taking it off its belt or putting it on one. The
// same section for every kind of device, under its own settings.
#include "app/app.hpp"
#include "app/widgets.hpp"
#include <glm/gtc/quaternion.hpp>
#include <imgui.h>

namespace fsim {

namespace {

// A turn as three angles in degrees (about x, then y, then z), to type in.
glm::vec3 degreesOf(const glm::quat& turn) {
    return glm::degrees(glm::eulerAngles(turn));
}

glm::quat turnOf(glm::vec3 degrees) {
    return glm::normalize(glm::quat(glm::radians(degrees)));
}

} // namespace

void App::drawPlacement(Pick pick) {
    auto& s = scene();
    auto* place = placementOf(s, pick);
    if (place == nullptr) {
        return;
    }
    ImGui::SeparatorText("Placement");
    if (place->parent == Parent::World) {
        ImGui::TextUnformatted("standing on its own");
        if (const auto nearest = nearestConveyor(s, pick)) {
            ImGui::SameLine();
            const auto label = "Put it on " + s.conveyors[*nearest].spec.name;
            if (ImGui::SmallButton(label.c_str())) {
                attach(s, pick, *nearest);
                return;
            }
            ImGui::SetItemTooltip("At its usual place on that belt, as near as it can be to where it is now");
        }
        ImGui::SetNextItemWidth(fieldWidth());
        ImGui::DragFloat3("at", &place->offset.x, 0.005F, -100.0F, 100.0F, "%.3f m");
        ImGui::SetItemTooltip("Where its usual place on a belt would be: on the belt's centre line, at its surface");
        auto degrees = degreesOf(place->turn);
        ImGui::SetNextItemWidth(fieldWidth());
        if (ImGui::DragFloat3("turned", &degrees.x, 0.5F, -180.0F, 180.0F, "%.1f deg")) {
            place->turn = turnOf(degrees);
        }
        ImGui::SetItemTooltip("About x, y and z (x along what would be its belt, y up)");
        scaledSlider("as if on a belt", place->span, 1000.0F, 50.0F, 2000.0F, "%.0f mm wide");
        ImGui::SetItemTooltip("It's worked out as if on a belt this wide (a pusher's stroke, a beam's length)");
        scaledSlider("that's", place->height, 1000.0F, 50.0F, 3000.0F, "%.0f mm high");
        ImGui::SetItemTooltip("and this high: what's on the floor below it (a bin) stays on the floor");
    } else {
        const bool usual = place->asUsual();
        ImGui::TextUnformatted(usual ? "on its belt, as usual" : "on its belt, moved");
        if (!usual) {
            ImGui::SameLine();
            if (ImGui::SmallButton("Back to its usual place")) {
                *place = Placement{};
                return;
            }
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Take it off")) {
            detach(s, pick);
            return;
        }
        ImGui::SetItemTooltip("It stays exactly where it is, standing on its own: the belt no longer carries it along");
        auto millimetres = place->offset * 1000.0F;
        ImGui::SetNextItemWidth(fieldWidth());
        if (ImGui::DragFloat3("moved by", &millimetres.x, 1.0F, -5000.0F, 5000.0F, "%.0f mm")) {
            place->offset = millimetres / 1000.0F;
        }
        ImGui::SetItemTooltip("From its usual place: along the belt, up, and across to the right");
        auto degrees = degreesOf(place->turn);
        ImGui::SetNextItemWidth(fieldWidth());
        if (ImGui::DragFloat3("turned", &degrees.x, 0.5F, -180.0F, 180.0F, "%.1f deg")) {
            place->turn = turnOf(degrees);
        }
        ImGui::SetItemTooltip("About the belt's direction, the vertical and across");
    }
    if (isSensor(pick.kind) && !place->asUsual()) {
        ImGui::TextColored(kProblem, "Not sensing yet: off its belt's usual place (milestone 22).");
    }
    ImGui::TextColored(kDim, "Or drag its arrows (W) or rings (E) in the view; T: its own axes or the world's.");
}

} // namespace fsim
