#include "app/widgets.hpp"
#include <imgui_stdlib.h>

namespace fsim {

// A float slider whose value is stored in another unit (ms shown, s stored).
bool scaledSlider(const char* label, double& value, double scale, float min, float max, const char* format) {
    float shown = static_cast<float>(value * scale);
    ImGui::SetNextItemWidth(kFieldWidth);
    if (ImGui::SliderFloat(label, &shown, min, max, format)) {
        value = shown / scale;
        return true;
    }
    return false;
}

bool scaledSlider(const char* label, float& value, float scale, float min, float max, const char* format) {
    double v = value;
    const bool changed = scaledSlider(label, v, scale, min, max, format);
    value = static_cast<float>(v);
    return changed;
}

// An address, typed in: red when it isn't an address of the expected kind.
void addressField(const char* label, std::string& text, IoAddress::Area area, IoAddress::Size size) {
    const auto address = parseAddress(text);
    const bool ok =
        text.empty() || (address && address->area == area && address->size == size && IoImage::fits(*address));
    if (!ok) {
        ImGui::PushStyleColor(ImGuiCol_Text, kProblem);
    }
    ImGui::SetNextItemWidth(kFieldWidth);
    const bool input = area == IoAddress::Area::Input;
    const char* hint = size == IoAddress::Size::Bit    ? (input ? "%IX0.0" : "%QX0.0")
                       : size == IoAddress::Size::Word ? (input ? "%IW12" : "%QW12")
                                                       : (input ? "%ID8" : "%QD8");
    ImGui::InputTextWithHint(label, hint, &text);
    if (!ok) {
        ImGui::PopStyleColor();
        const char* kind = size == IoAddress::Size::Bit    ? "bit"
                           : size == IoAddress::Size::Word ? "word"
                                                           : "double word";
        ImGui::SetItemTooltip("Not a valid %s %s here", input ? "input" : "output", kind);
    }
}

// A combo to pick one of the scene's conveyors, by name.
void conveyorCombo(const char* label, std::size_t& conveyor, const Scene& scene) {
    ImGui::SetNextItemWidth(kFieldWidth);
    const auto current = conveyor < scene.conveyors.size() ? scene.conveyors[conveyor].spec.name : std::string("?");
    if (ImGui::BeginCombo(label, current.c_str())) {
        for (std::size_t i = 0; i < scene.conveyors.size(); ++i) {
            if (ImGui::Selectable(scene.conveyors[i].spec.name.c_str(), i == conveyor)) {
                conveyor = i;
            }
        }
        ImGui::EndCombo();
    }
}

} // namespace fsim
