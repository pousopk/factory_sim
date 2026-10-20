// The air jet's inspector: its nozzles, air and valve, what it pushes now,
// and blowing by hand.
#include "app/app.hpp"
#include "app/widgets.hpp"
#include <imgui.h>
#include <imgui_stdlib.h>

namespace fsim {

void App::drawAirJetInspector(std::size_t index) {
    auto& s = scene();
    auto& spec = s.airJets[index];
    if (index < world_->airJets().size()) {
        const auto& jet = world_->airJets()[index];
        const auto lamp = [](bool on) {
            return on ? ImVec4(1.0F, 0.85F, 0.1F, 1.0F) : ImVec4(0.25F, 0.22F, 0.12F, 1.0F);
        };
        ImGui::ColorButton("##valve", lamp(jet.valve().value()), ImGuiColorEditFlags_NoTooltip, ImVec2(14, 14));
        ImGui::SameLine();
        ImGui::TextUnformatted("valve");
        ImGui::SameLine();
        ImGui::ColorButton("##air", lamp(jet.air().value()), ImGuiColorEditFlags_NoTooltip, ImVec2(14, 14));
        ImGui::SameLine();
        ImGui::TextUnformatted("air");
        ImGui::SameLine();
        ImGui::TextColored(kDim, "  pushing %.2f N now", jet.lastPush);
        ImGui::SetItemTooltip("What its air pushes products with, all its streams together\n"
                              "(only the share of each that's square to the surface it hits)");
    }
    ImGui::SetNextItemWidth(fieldWidth());
    ImGui::InputText("name", &spec.name);
    conveyorCombo("on conveyor", spec.conveyor, s);
    const float length = spec.conveyor < s.conveyors.size() ? s.conveyors[spec.conveyor].spec.length : 1.0F;
    ImGui::SetNextItemWidth(fieldWidth());
    ImGui::SliderFloat("position", &spec.along, 0.0F, length, "%.3f m");
    sideCombo(spec.side);
    ImGui::SetItemTooltip("Where it's mounted: it blows towards the other side");
    scaledSlider("height", spec.height, 1000.0F, 2.0F, 150.0F, "%.0f mm above the belt");
    ImGui::SetNextItemWidth(fieldWidth());
    ImGui::SliderInt("nozzles", &spec.nozzles, 1, 12);
    scaledSlider("apart", spec.pitch, 1000.0F, 5.0F, 100.0F, "%.0f mm");
    scaledSlider("aimed", spec.aim, kDegrees, -30.0F, 30.0F, "%.0f deg up");
    ImGui::SeparatorText("Air");
    ImGui::SetNextItemWidth(fieldWidth());
    ImGui::SliderFloat("pressure", &spec.pressure, 0.5F, 10.0F, "%.1f bar");
    ImGui::SetItemTooltip("The supply's pressure, held steady (gauge: above the atmosphere's)");
    scaledSlider("bore", spec.bore, 1000.0F, 0.5F, 8.0F, "%.1f mm");
    scaledSlider("spread", spec.spread, kDegrees, 2.0F, 25.0F, "%.0f deg");
    ImGui::SetItemTooltip("From the jet's axis to its edge. A free jet of air spreads by about 12 degrees.");
    const float thrust = thrustOf(spec);
    ImGui::TextColored(kDim, "%.2f N a nozzle, %.1f N in all", thrust, thrust * static_cast<float>(spec.nozzles));
    ImGui::SetItemTooltip("About 1.25 x pressure x the bore's area, as blow-off nozzles' datasheets give");
    ImGui::SeparatorText("Valve");
    scaledSlider("opens in", spec.openTime, 1000.0, 0.5F, 30.0F, "%.1f ms");
    ImGui::SetItemTooltip("From the output switching on to the air flowing: the coil pulling the armature in");
    scaledSlider("closes in", spec.closeTime, 1000.0, 0.5F, 30.0F, "%.1f ms");
    ImGui::SetItemTooltip("From the output switching off to the air stopping: the armature dropping out,\n"
                          "the hose emptying");
    addressField("valve", spec.valveOutput, IoAddress::Area::Output, IoAddress::Size::Bit);
    ImGui::TextColored(kDim, "Blows products off the other side: put a bin there.");
    if (index < world_->airJets().size()) {
        ImGui::SeparatorText("By hand");
        for (const double seconds : {0.02, 0.05, 0.2}) {
            const auto label = std::to_string(static_cast<int>(seconds * 1000.0)) + " ms";
            if (ImGui::Button(label.c_str())) {
                world_->blowAirJet(index, seconds);
            }
            ImGui::SameLine();
        }
        ImGui::TextUnformatted("blow");
        if (world_->program() != nullptr) {
            ImGui::TextColored(kDim, "A control program is running: it sets the valve every scan.");
        }
    }
}

} // namespace fsim
