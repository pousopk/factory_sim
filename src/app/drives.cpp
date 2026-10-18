// A conveyor's drive in the inspector: its state, the telegram as the PLC
// and the drive see it, bit by bit, and its settings.
#include "app/app.hpp"
#include "app/widgets.hpp"
#include <cmath>
#include <imgui.h>
#include <imgui_stdlib.h>

namespace fsim {

namespace {

// The bits of a word, named, lit when set, as in a drive's commissioning tool.
void bits(const char* id, std::uint16_t word, const char* const names[16]) {
    if (ImGui::BeginTable(id, 2, ImGuiTableFlags_SizingStretchSame)) {
        for (unsigned bit = 0; bit < 16; ++bit) {
            if (names[bit] == nullptr) {
                continue;
            }
            ImGui::TableNextColumn();
            const bool on = ((static_cast<unsigned>(word) >> bit) & 1U) != 0;
            ImGui::TextColored(on ? ImVec4(1.0F, 0.85F, 0.1F, 1.0F) : kDim, "%2u %s", bit, names[bit]);
        }
        ImGui::EndTable();
    }
}

const char* const kStw1Names[16] = {"ON (OFF1 when 0)",
                                    "no coast stop (OFF2)",
                                    "no quick stop (OFF3)",
                                    "enable operation",
                                    "ramp enable",
                                    "ramp start",
                                    "setpoint enable",
                                    "acknowledge fault",
                                    nullptr,
                                    nullptr,
                                    "control by PLC",
                                    nullptr,
                                    nullptr,
                                    nullptr,
                                    nullptr,
                                    nullptr};
const char* const kZsw1Names[16] = {"ready to switch on",
                                    "ready to operate",
                                    "operation enabled",
                                    "fault",
                                    "no coast stop",
                                    "no quick stop",
                                    "switching on inhibited",
                                    nullptr,
                                    "at speed",
                                    "control requested",
                                    nullptr,
                                    "no current limit",
                                    "brake open",
                                    "no motor overtemp",
                                    "forward",
                                    "no overload"};

} // namespace

void App::drawDrive(std::size_t index) {
    auto& spec = scene().conveyors[index].spec;
    auto& drive = spec.drive;
    if (index < world_->conveyors().size()) {
        const auto& running = world_->conveyors()[index].drive;
        const bool faulted = running.state() == Drive::State::Fault;
        ImGui::TextColored(faulted ? kProblem : ImVec4(1.0F, 1.0F, 1.0F, 1.0F), "%s", stateName(running.state()));
        ImGui::SameLine();
        ImGui::TextColored(kDim, "  %.1f %% (%.3f m/s)", running.speed() * 100.0,
                           running.speed() * drive.referenceSpeed);
        if (faulted) {
            ImGui::TextColored(kProblem, "%s", running.fault().c_str());
            ImGui::TextColored(kDim, "The PLC acknowledges it with a rising edge of STW1 bit 7.");
        } else if (ImGui::SmallButton("Trip it (overload)")) {
            world_->tripDrive(index, "F30005: power unit overload I2t (tripped by hand)");
        }
        if (ImGui::TreeNode("STW1 from the PLC", "STW1 16#%04X from the PLC", running.controlWord())) {
            bits("stw1", running.controlWord(), kStw1Names);
            ImGui::TreePop();
        }
        ImGui::TextColored(kDim, "NSOLL_A %d = %.1f %%", running.setpoint(), 100.0 * running.setpoint() / kFullScale);
        if (ImGui::TreeNode("ZSW1 to the PLC", "ZSW1 16#%04X to the PLC", running.statusWord())) {
            bits("zsw1", running.statusWord(), kZsw1Names);
            ImGui::TreePop();
        }
        ImGui::TextColored(kDim, "NIST_A %d", running.actualSpeed());
        if (world_->program() == nullptr) {
            ImGui::TextColored(kDim, "No control program: it runs at the target speed, as if told to.");
        }
    }
    scaledSlider("reference speed", drive.referenceSpeed, 1.0F, 0.1F, 3.0F, "%.2f m/s = 100 %%");
    ImGui::SetItemTooltip("The belt speed at 16#4000 (100 %%) in NSOLL_A and NIST_A");
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::SliderFloat("quick stop (OFF3)", &drive.quickStop, 0.5F, 20.0F, "%.1f m/s²", ImGuiSliderFlags_Logarithmic);
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::SliderFloat("coasting", &drive.coast, 0.1F, 10.0F, "%.1f m/s²", ImGuiSliderFlags_Logarithmic);
    ImGui::SetItemTooltip("How fast the loaded belt slows down by itself with the motor off (OFF2, a fault)");
    ImGui::TextColored(kDim, "The ramp is the conveyor's acceleration.");
    addressField("STW1", drive.controlOutput, IoAddress::Area::Output, IoAddress::Size::Word);
    ImGui::SetItemTooltip("Control word, from the PLC");
    addressField("NSOLL_A", drive.setpointOutput, IoAddress::Area::Output, IoAddress::Size::Word);
    ImGui::SetItemTooltip("Speed setpoint, from the PLC: 16#4000 = 100 %%");
    addressField("ZSW1", drive.statusInput, IoAddress::Area::Input, IoAddress::Size::Word);
    ImGui::SetItemTooltip("Status word, to the PLC");
    addressField("NIST_A", drive.actualInput, IoAddress::Area::Input, IoAddress::Size::Word);
    ImGui::SetItemTooltip("Actual speed, to the PLC");
    if (const auto status = parseAddress(drive.statusInput); status && status->size == IoAddress::Size::Word) {
        // The trap every S7 programmer meets once: a word's bit 0 is in its *second* byte.
        ImGui::TextColored(kDim, "ZSW1 bit 0 is %%IX%zu.0, bit 8 is %%IX%zu.0 (big-endian)", status->byte + 1,
                           status->byte);
    }
}

} // namespace fsim
