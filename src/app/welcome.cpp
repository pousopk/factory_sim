// The start page: what factory_sim shows when it starts without a scene
// named, and after File > Close scene. A new, empty line, a scene to open
// (recent ones, the untitled one left last time), or a demo.
//
// It fills the window under the menu bar: no 3D view, toolbar or panels
// until there's a line to show in them.
#include "app/app.hpp"
#include "app/widgets.hpp"
#include <algorithm>
#include <imgui.h>
#include <optional>

namespace fsim {

const std::vector<Demo>& demos() {
    static const std::vector<Demo> list = {
        {"Sorting",
         "A conveyor, a third of the boxes bad, an inspection sensor, a pusher, a bin, and the built-in sorter "
         "wired to them.",
         [] { return sortingDemo(); }},
        {"Tins and cameras", "Tins at 1 m/s, one photocell triggering four cameras: which camera misses which tin.",
         [] { return tinDemo(); }},
        {"Conveyor (soft_plc)",
         "ladder_editor's conveyor program, run by soft_plc. Press Start on the operator panel; ladder_editor can "
         "go online with --connect=5020.",
         [] {
             auto demo = conveyorDemo(FACTORY_SIM_EXAMPLES_DIR "/conveyor.st");
             demo.monitorPort = 5020;
             return demo;
         }},
        {"Conveyor with a drive (soft_plc)",
         "A belt driven over PROFIdrive telegram 1 by examples/drive.st: press Start, turn the Speed potentiometer.",
         [] { return driveDemo(FACTORY_SIM_EXAMPLES_DIR "/drive.st"); }},
        {"Line equipment (soft_plc)",
         "An accumulation conveyor, a turntable and a lift, sequenced by examples/equipment.st.",
         [] { return equipmentDemo(FACTORY_SIM_EXAMPLES_DIR "/equipment.st"); }},
        {"Merge (soft_plc)", "A side conveyor joining a main line, waiting for gaps (examples/merge.st).",
         [] { return mergeDemo(FACTORY_SIM_EXAMPLES_DIR "/merge.st"); }},
        {"Sensors",
         "Boxes of different heights and tins past an inductive and a capacitive proximity switch, a laser "
         "measuring heights, and a light curtain.",
         [] { return sensorsDemo(); }},
        {"Line-scan and area cameras",
         "A line-scan camera on the encoder and an area camera over one belt: see View > Camera view.",
         [] { return cameraDemo(); }},
        {"Odd shapes",
         "Frozen fish, bottles, bags, balls, brackets and an STL part past a photocell, a laser, a line-scan camera "
         "and a light curtain, which all see their real outlines.",
         [] { return shapesDemo(); }},
        {"Rejecting: pusher and air jet",
         "The same products, a third of them bad, on two lines: one rejects with a pusher, the other with an air "
         "jet. Watch what each throws into its bin, and what it can't move.",
         [] { return rejectDemo(); }},
    };
    return list;
}

void App::drawStartPage() {
    const auto* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    constexpr auto kFlags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings |
                            ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoBringToFrontOnFocus;
    if (!ImGui::Begin("##start", nullptr, kFlags)) {
        ImGui::End();
        return;
    }
    // One column of choices and the demos beside it, together no wider than
    // reads well, in the middle of the window.
    const float em = ImGui::GetFontSize();
    const float width = std::min(ImGui::GetContentRegionAvail().x, em * 56.0F);
    const float left = em * 16.0F;
    const float gap = em * 3.0F;
    ImGui::SetCursorPos(ImVec2(std::max(ImGui::GetCursorPosX(), (ImGui::GetWindowWidth() - width) / 2.0F),
                               std::max(ImGui::GetCursorPosY(), ImGui::GetWindowHeight() * 0.10F)));
    ImGui::BeginGroup();

    ImGui::PushFont(nullptr, em * 2.0F);
    ImGui::TextUnformatted("factory_sim");
    ImGui::PopFont();
    ImGui::TextColored(kDim, "Conveyors, sensors and actuators in 3D, controlled by a PLC");
    ImGui::Dummy(ImVec2(0.0F, em));

    // What to do: a new line, a scene to open, one to go back to. Acting
    // after drawing, since each one replaces what's being drawn.
    std::optional<std::filesystem::path> chosenFile;
    bool newLine = false;
    bool open = false;
    bool resume = false;
    ImGui::BeginChild("start", ImVec2(left, 0.0F), ImGuiChildFlags_AutoResizeY);
    const ImVec2 button(-FLT_MIN, em * 2.2F);
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.26F, 0.49F, 0.86F, 1.0F)); // the theme's accent
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.33F, 0.56F, 0.92F, 1.0F));
    newLine = ImGui::Button("New empty line", button);
    ImGui::PopStyleColor(2);
    ImGui::SetItemTooltip("An empty floor, untitled until Save as names it (Ctrl+N)");
    open = ImGui::Button("Open...", button);
    ImGui::SetItemTooltip("A scene file (Ctrl+O)");

    ImGui::Dummy(ImVec2(0.0F, em * 0.5F));
    ImGui::SeparatorText("Recent");
    if (untitledLeft_) {
        resume = ImGui::Selectable("Continue untitled");
        ImGui::SetItemTooltip("The untitled scene, as you left it (saved in the config folder)");
    }
    for (const auto& path : settings_.recent) {
        std::error_code ignored;
        const bool gone = !std::filesystem::exists(path, ignored); // deleted since it was opened
        ImGui::BeginDisabled(gone);
        if (ImGui::Selectable(path.filename().string().c_str())) {
            chosenFile = path; // not while iterating: opening changes the list
        }
        ImGui::EndDisabled();
        ImGui::SetItemTooltip("%s%s", path.string().c_str(), gone ? " (no longer there)" : "");
    }
    if (settings_.recent.empty()) {
        ImGui::TextColored(kDim, "No scenes opened yet.");
    }
    ImGui::EndChild();

    // The demos, as tiles two by two: a name, and what it shows.
    ImGui::SameLine(0.0F, gap);
    std::optional<std::size_t> chosenDemo;
    ImGui::BeginChild("demos", ImVec2(width - left - gap, 0.0F), ImGuiChildFlags_AutoResizeY);
    ImGui::SeparatorText("Start from a demo");
    const auto& style = ImGui::GetStyle();
    const float tileWidth = (ImGui::GetContentRegionAvail().x - style.ItemSpacing.x) / 2.0F;
    const ImVec2 padding(em * 0.6F, em * 0.5F);
    const float wrap = tileWidth - 2.0F * padding.x;
    float tileHeight = 0.0F; // the tallest, so the rows line up
    for (const auto& demo : demos()) {
        tileHeight = std::max(tileHeight, ImGui::CalcTextSize(demo.description, nullptr, false, wrap).y);
    }
    tileHeight += 2.0F * padding.y + ImGui::GetTextLineHeightWithSpacing();
    auto* draw = ImGui::GetWindowDrawList();
    for (std::size_t i = 0; i < demos().size(); ++i) {
        const auto& demo = demos()[i];
        if (i % 2 == 1) {
            ImGui::SameLine();
        }
        const ImVec2 at = ImGui::GetCursorScreenPos();
        ImGui::PushID(static_cast<int>(i));
        if (ImGui::Selectable("##demo", false, ImGuiSelectableFlags_None, ImVec2(tileWidth, tileHeight))) {
            chosenDemo = i;
        }
        ImGui::PopID();
        const ImVec2 to(at.x + tileWidth, at.y + tileHeight);
        draw->AddRect(at, to, ImGui::GetColorU32(ImGuiCol_Border), style.FrameRounding);
        draw->AddText(ImVec2(at.x + padding.x, at.y + padding.y), ImGui::GetColorU32(ImGuiCol_Text), demo.name);
        draw->AddText(ImGui::GetFont(), ImGui::GetFontSize(),
                      ImVec2(at.x + padding.x, at.y + padding.y + ImGui::GetTextLineHeightWithSpacing()),
                      ImGui::GetColorU32(kDim), demo.description, nullptr, wrap);
    }
    ImGui::EndChild();

    ImGui::EndGroup();
    ImGui::End();

    if (newLine) {
        startNew(Scene{});
    } else if (open) {
        dialog_ = Dialog::Open;
        browser_.emplace(std::filesystem::current_path());
        dialogName_.clear();
        dialogError_.clear();
    } else if (resume) {
        continueUntitled();
    } else if (chosenFile) {
        openScene(*chosenFile);
    } else if (chosenDemo) {
        startNew(demos()[*chosenDemo].make());
    }
}

} // namespace fsim
