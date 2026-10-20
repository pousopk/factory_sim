#pragma once
#include "sim/scene.hpp"
#include <algorithm>
#include <imgui.h>
#include <numbers>
#include <string>

// Small widgets the panels share: colours, sliders in another unit, typed
// addresses, the conveyor combo.
namespace fsim {

inline const ImVec4 kDim{0.65F, 0.67F, 0.72F, 1.0F};
inline const ImVec4 kProblem{0.95F, 0.45F, 0.40F, 1.0F};
// How wide a value field is: a share of the panel, so labels fit beside it
// and wide panels use their room, within reason.
inline float fieldWidth() {
    return std::clamp(ImGui::GetContentRegionAvail().x * 0.55F, ImGui::GetFontSize() * 7.0F,
                      ImGui::GetFontSize() * 16.0F);
}
inline constexpr float kDegrees = 180.0F / std::numbers::pi_v<float>;

// A float slider whose value is stored in another unit (ms shown, s stored).
bool scaledSlider(const char* label, double& value, double scale, float min, float max, const char* format);
bool scaledSlider(const char* label, float& value, float scale, float min, float max, const char* format);

// An address, typed in: red when it isn't an address of the expected kind.
void addressField(const char* label, std::string& text, IoAddress::Area area, IoAddress::Size size);

// Left or right of the belt, looking along its travel.
void sideCombo(Side& side);

// A combo to pick one of the scene's conveyors, by name.
void conveyorCombo(const char* label, std::size_t& conveyor, const Scene& scene);

} // namespace fsim
