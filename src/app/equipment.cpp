// A conveyor's line equipment in the inspector: accumulation zones and a
// carriage (turntable or lift).
#include "app/app.hpp"
#include "app/widgets.hpp"
#include <imgui.h>
#include <imgui_stdlib.h>
#include <string>

namespace fsim {

namespace {

const char* stateName(ZoneState state) {
    switch (state) {
    case ZoneState::Arriving:
        return "arriving";
    case ZoneState::Full:
        return "full";
    case ZoneState::Empty:
        break;
    }
    return "empty";
}

} // namespace

void App::drawConveyorEquipment(std::size_t index) {
    auto& s = scene();
    auto& spec = s.conveyors[index].spec;
    const Conveyor* running = index < world_->conveyors().size() ? &world_->conveyors()[index] : nullptr;

    ImGui::SeparatorText("Accumulation zones");
    int zones = spec.zones;
    ImGui::SetNextItemWidth(fieldWidth());
    if (ImGui::SliderInt("zones", &zones, 1, 12)) {
        setZones(s, index, zones, spec.zoneLogic);
    }
    ImGui::SetItemTooltip("1: one belt. More: each zone its own motor and photo-eye (a motor-driven-roller\n"
                          "conveyor), so boxes can queue without pushing. Changing it restarts the run.");
    if (spec.zones > 1) {
        int logic = spec.zoneLogic == ZoneLogic::Local ? 0 : 1;
        ImGui::SetNextItemWidth(fieldWidth());
        if (ImGui::Combo("run by", &logic, "the zone cards (zero pressure)\0the PLC, one output per zone\0")) {
            setZones(s, index, spec.zones, logic == 0 ? ZoneLogic::Local : ZoneLogic::Plc);
        }
        if (spec.zoneLogic == ZoneLogic::Local) {
            addressField("release", spec.releaseOutput, IoAddress::Area::Output, IoAddress::Size::Bit);
            ImGui::SetItemTooltip("While this output is on, the last zone passes its box on. Empty: always.");
        }
        if (ImGui::BeginTable("zones", spec.zoneLogic == ZoneLogic::Plc ? 4 : 3,
                              ImGuiTableFlags_Borders | ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("zone");
            ImGui::TableSetupColumn("eye");
            if (spec.zoneLogic == ZoneLogic::Plc) {
                ImGui::TableSetupColumn("motor");
            }
            ImGui::TableSetupColumn("now");
            ImGui::TableHeadersRow();
            for (std::size_t z = 0; z < spec.zoneIo.size(); ++z) {
                ImGui::PushID(static_cast<int>(z));
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("%zu", z + 1);
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(-1.0F);
                ImGui::InputText("##eye", &spec.zoneIo[z].eyeInput);
                if (spec.zoneLogic == ZoneLogic::Plc) {
                    ImGui::TableNextColumn();
                    ImGui::SetNextItemWidth(-1.0F);
                    ImGui::InputText("##motor", &spec.zoneIo[z].motorOutput);
                }
                ImGui::TableNextColumn();
                if (running != nullptr && z < running->zoneRun.size()) {
                    const bool eye = running->eyes[z].output.value();
                    ImGui::TextColored(eye ? ImVec4(1.0F, 0.85F, 0.1F, 1.0F) : kDim, "%s%s, %s",
                                       spec.zoneLogic == ZoneLogic::Local ? stateName(running->zoneState[z]) : "",
                                       spec.zoneLogic == ZoneLogic::Local ? "" : (eye ? "box" : "clear"),
                                       running->zoneRun[z] ? "running" : "stopped");
                }
                ImGui::PopID();
            }
            ImGui::EndTable();
        }
    }

    ImGui::SeparatorText("Carriage");
    int carriage = static_cast<int>(spec.carriage);
    ImGui::SetNextItemWidth(fieldWidth());
    if (ImGui::Combo("on", &carriage, "nothing (fixed)\0a turntable\0a lift\0")) {
        setCarriage(s, index, static_cast<Carriage>(carriage));
    }
    ImGui::SetItemTooltip("A turntable turns the belt about its middle; a lift raises it.\n"
                          "Changing it restarts the run.");
    if (spec.carriage == Carriage::None) {
        return;
    }
    if (spec.carriage == Carriage::Turntable) {
        scaledSlider("turns", spec.turnAngle, kDegrees, -180.0F, 180.0F, "%.0f deg (left +)");
        const float corner = std::sqrt(spec.length * spec.length + spec.width * spec.width) / 2.0F;
        ImGui::TextColored(kDim, "its corners sweep a circle of %.0f mm: keep %.0f mm clear of its middle",
                           corner * 1000.0F, corner * 1000.0F);
    } else {
        scaledSlider("rises", spec.liftHeight, 1000.0F, 50.0F, 2000.0F, "%.0f mm");
    }
    scaledSlider("dead time", spec.carriageStroke.deadTime, 1000.0, 0.0F, 500.0F, "%.0f ms");
    scaledSlider("out in", spec.carriageStroke.extendTime, 1000.0, 100.0F, 10000.0F, "%.0f ms");
    scaledSlider("back in", spec.carriageStroke.retractTime, 1000.0, 100.0F, 10000.0F, "%.0f ms");
    addressField("move", spec.carriageOutput, IoAddress::Area::Output, IoAddress::Size::Bit);
    ImGui::SetItemTooltip(spec.carriage == Carriage::Turntable ? "On: turn. Off: back home." : "On: up. Off: down.");
    addressField("at the end", spec.carriageEndInput, IoAddress::Area::Input, IoAddress::Size::Bit);
    addressField("at home", spec.carriageHomeInput, IoAddress::Area::Input, IoAddress::Size::Bit);
    if (running != nullptr) {
        drawStrokeControls(world_->conveyors()[index].carriage, Pick{Pick::Kind::Conveyor, index});
    }
}

} // namespace fsim
