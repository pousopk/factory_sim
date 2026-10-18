// The inspectors of proximity switches, laser distance sensors and light
// curtains: what they see now, their settings and their wiring.
#include "app/app.hpp"
#include "app/widgets.hpp"
#include <cmath>
#include <imgui.h>
#include <imgui_stdlib.h>

namespace fsim {

namespace {

void ledText(bool on, const char* text) {
    ImGui::ColorButton("##led", on ? ImVec4(1.0F, 0.85F, 0.1F, 1.0F) : ImVec4(0.25F, 0.22F, 0.12F, 1.0F),
                       ImGuiColorEditFlags_NoTooltip, ImVec2(14, 14));
    ImGui::SameLine();
    ImGui::TextUnformatted(text);
}

float conveyorLength(const Scene& scene, std::size_t conveyor) {
    return conveyor < scene.conveyors.size() ? scene.conveyors[conveyor].spec.length : 1.0F;
}

} // namespace

void App::drawProximityInspector(std::size_t index) {
    auto& s = scene();
    auto& spec = s.proximities[index];
    if (index < world_->proximities().size()) {
        const auto& running = world_->proximities()[index];
        ledText(running.detector.output.value(), running.detector.output.value() ? "output ON" : "output off");
        ImGui::SameLine();
        if (running.distance >= 0.0F) {
            ImGui::TextColored(kDim, "  nearest item %.1f mm from the face", running.distance * 1000.0F);
        } else {
            ImGui::TextColored(kDim, "  nothing within reach");
        }
    }
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::InputText("name", &spec.name);
    addressField("input", spec.input, IoAddress::Area::Input, IoAddress::Size::Bit);
    int kind = spec.inductive ? 0 : 1;
    ImGui::SetNextItemWidth(kFieldWidth);
    if (ImGui::Combo("kind", &kind, "inductive (metal only)\0capacitive (anything)\0")) {
        spec.inductive = kind == 0;
    }
    ImGui::SetItemTooltip("Inductive: a coil's field, damped only by metal: tins, not cardboard.\n"
                          "Capacitive: sees anything, but non-metal much closer (here at 40 %% of sn).");
    conveyorCombo("on conveyor", spec.conveyor, s);
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::SliderFloat("position", &spec.along, 0.0F, conveyorLength(s, spec.conveyor), "%.3f m");
    int mount = static_cast<int>(spec.mount);
    ImGui::SetNextItemWidth(kFieldWidth);
    if (ImGui::Combo("mounted", &mount, "above, looking down\0on the left\0on the right\0")) {
        spec.mount = static_cast<Mount>(mount);
    }
    scaledSlider(spec.mount == Mount::Above ? "face above belt" : "face height", spec.height, 1000.0F, 5.0F, 300.0F,
                 "%.0f mm");
    if (spec.mount != Mount::Above) {
        scaledSlider("in from the edge", spec.inset, 1000.0F, -30.0F, 50.0F, "%.0f mm");
        ImGui::SetItemTooltip("How far the face reaches in over the belt from its edge (through the guide rail)");
    }
    scaledSlider("face", spec.faceDiameter, 1000.0F, 8.0F, 40.0F, "M%.0f");
    scaledSlider("switching distance", spec.range, 1000.0F, 1.0F, 40.0F, "sn %.1f mm");
    ImGui::SetItemTooltip("For steel. Typical flush-mounted: M8 1.5 mm, M12 2 mm, M18 5 mm, M30 10 mm;\n"
                          "non-flush about double.");
    scaledSlider("response", spec.responseTime, 1000.0, 0.1F, 20.0F, "%.1f ms");
}

void App::drawLaserInspector(std::size_t index) {
    auto& s = scene();
    auto& spec = s.lasers[index];
    if (index < world_->lasers().size()) {
        const auto& running = world_->lasers()[index];
        if (running.distance >= 0.0F) {
            ImGui::Text("%.1f mm away: %u", running.distance * 1000.0F, running.value);
            ImGui::TextColored(kDim, "an item %.1f mm high (the belt is %.0f mm away)",
                               (spec.mountHeight - running.distance) * 1000.0F, spec.mountHeight * 1000.0F);
        } else {
            ImGui::TextColored(kProblem, "out of range: %u (overflow)", running.value);
        }
    }
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::InputText("name", &spec.name);
    addressField("output word", spec.output, IoAddress::Area::Input, IoAddress::Size::Word);
    ImGui::SetItemTooltip("The PLC input word it's wired to: 0..27648 over its range, 32767 out of it");
    conveyorCombo("over conveyor", spec.conveyor, s);
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::SliderFloat("position", &spec.along, 0.0F, conveyorLength(s, spec.conveyor), "%.3f m");
    scaledSlider("across", spec.across, 1000.0F, -300.0F, 300.0F, "%.0f mm from the middle");
    scaledSlider("mounted", spec.mountHeight, 1000.0F, 50.0F, 1500.0F, "%.0f mm above the belt");
    scaledSlider("range from", spec.rangeMin, 1000.0F, 10.0F, 1500.0F, "%.0f mm");
    scaledSlider("range to", spec.rangeMax, 1000.0F, 20.0F, 2000.0F, "%.0f mm");
    spec.rangeMax = std::max(spec.rangeMax, spec.rangeMin + 0.001F);
    scaledSlider("resolution", spec.resolution, 1000.0F, 0.01F, 5.0F, "%.2f mm");
    scaledSlider("noise", spec.noise, 1000.0F, 0.0F, 5.0F, "%.2f mm (1 sigma)");
    scaledSlider("measures every", spec.measureTime, 1000.0, 0.1F, 50.0F, "%.1f ms");
    const double step = (spec.rangeMax - spec.rangeMin) / kAnalogFullScale * 1000.0;
    ImGui::TextColored(kDim, "one count of the word is %.4f mm", step);
}

void App::drawCurtainInspector(std::size_t index) {
    auto& s = scene();
    auto& spec = s.curtains[index];
    if (index < world_->curtains().size()) {
        const auto& running = world_->curtains()[index];
        ledText(running.clear.value(), running.clear.value() ? "clear" : "broken");
        ImGui::SameLine();
        ImGui::TextColored(kDim, "  %d beams broken: about %.0f mm high", running.blocked,
                           running.blocked > 0
                               ? (spec.lowest + static_cast<float>(running.blocked - 1) * spec.pitch) * 1000.0F
                               : 0.0F);
    }
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::InputText("name", &spec.name);
    addressField("clear", spec.clearInput, IoAddress::Area::Input, IoAddress::Size::Bit);
    ImGui::SetItemTooltip("On while every beam is clear (as a safety curtain's OSSD outputs are)");
    addressField("beams broken", spec.countInput, IoAddress::Area::Input, IoAddress::Size::Word);
    conveyorCombo("on conveyor", spec.conveyor, s);
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::SliderFloat("position", &spec.along, 0.0F, conveyorLength(s, spec.conveyor), "%.3f m");
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::SliderInt("beams", &spec.beams, 1, 64);
    scaledSlider("pitch", spec.pitch, 1000.0F, 2.5F, 50.0F, "%.1f mm");
    scaledSlider("lowest beam", spec.lowest, 1000.0F, 2.0F, 100.0F, "%.0f mm above the belt");
    scaledSlider("scan cycle", spec.scanTime, 1000.0, 1.0F, 50.0F, "%.0f ms");
    ImGui::TextColored(kDim, "covers %.0f to %.0f mm", spec.lowest * 1000.0F,
                       (spec.lowest + static_cast<float>(spec.beams - 1) * spec.pitch) * 1000.0F);
}

} // namespace fsim
