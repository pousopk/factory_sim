// The window's layout: a toolbar at the top, a status bar at the bottom, and
// between them a dockspace, with the 3D view in its middle (the central node,
// which the panels don't cover) and the panels docked around it as tabs.
// The arrangement is remembered with the window layout; View > Reset layout
// puts it back to this default.
#include "app/app.hpp"
#include "app/widgets.hpp"
#include <imgui.h>
#include <imgui_internal.h> // the dock builder and viewport side bars

namespace fsim {

namespace {

// The default arrangement: a column on the left for building (Control, Line,
// Inspector), one on the right for watching (I/O, Triggers, PLC variables,
// Cameras), a strip under the view for time (Logic analyser, Camera view,
// Operator panel).
void buildDefaultLayout(ImGuiID dockspace, ImVec2 size) {
    ImGui::DockBuilderRemoveNode(dockspace);
    ImGui::DockBuilderAddNode(dockspace, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(dockspace, size);
    ImGuiID left = 0;
    ImGuiID rest = 0;
    ImGui::DockBuilderSplitNode(dockspace, ImGuiDir_Left, 0.22F, &left, &rest);
    ImGuiID right = 0;
    ImGuiID middle = 0;
    ImGui::DockBuilderSplitNode(rest, ImGuiDir_Right, 0.27F, &right, &middle);
    ImGuiID bottom = 0;
    ImGuiID center = 0;
    ImGui::DockBuilderSplitNode(middle, ImGuiDir_Down, 0.34F, &bottom, &center);
    ImGuiID leftTop = 0;
    ImGuiID leftBottom = 0;
    ImGui::DockBuilderSplitNode(left, ImGuiDir_Up, 0.42F, &leftTop, &leftBottom);
    for (const char* name : {"Control", "Line"}) {
        ImGui::DockBuilderDockWindow(name, leftTop);
    }
    ImGui::DockBuilderDockWindow("Inspector", leftBottom);
    for (const char* name : {"I/O", "Triggers", "PLC variables", "Cameras"}) {
        ImGui::DockBuilderDockWindow(name, right);
    }
    for (const char* name : {"Logic analyser", "Camera view", "Operator panel"}) {
        ImGui::DockBuilderDockWindow(name, bottom);
    }
    ImGui::DockBuilderFinish(dockspace);
}

} // namespace

void App::drawDockspace() {
    auto* viewport = ImGui::GetMainViewport();
    const ImGuiID dockspace = ImGui::GetID("dockspace");
    // A first run (no saved layout) or View > Reset layout: the default.
    if (resetLayout_ || ImGui::DockBuilderGetNode(dockspace) == nullptr) {
        buildDefaultLayout(dockspace, viewport->WorkSize);
        resetLayout_ = false;
    }
    // Pass-through: where no panel is docked (the middle), the 3D view shows
    // and gets the mouse.
    ImGui::DockSpaceOverViewport(dockspace, viewport, ImGuiDockNodeFlags_PassthruCentralNode);
    if (const auto* central = ImGui::DockBuilderGetCentralNode(dockspace)) {
        viewMin_ = central->Pos;
        viewSize_ = central->Size;
    }
}

// Running the simulation, always in reach.
void App::drawToolbar() {
    const float height = ImGui::GetFrameHeight() + ImGui::GetStyle().WindowPadding.y * 1.2F;
    constexpr ImGuiWindowFlags kFlags = ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings;
    if (ImGui::BeginViewportSideBar("##toolbar", ImGui::GetMainViewport(), ImGuiDir_Up, height, kFlags)) {
        if (ImGui::Button(paused_ ? "  Run  " : " Pause ")) {
            paused_ = !paused_;
        }
        ImGui::SetItemTooltip("Space");
        ImGui::SameLine();
        ImGui::BeginDisabled(!paused_);
        if (ImGui::Button("Step 1 ms")) {
            world_->step();
        }
        ImGui::SameLine();
        if (ImGui::Button("Step 100 ms")) {
            world_->run(0.1);
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button("Restart")) {
            restartRun();
        }
        ImGui::SetItemTooltip("Start the run again: no items, time 0. (A change to the layout does this too.)");
        ImGui::SameLine();
        if (ImGui::Button("Clear items")) {
            world_->clearItems();
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(ImGui::GetFontSize() * 9.0F);
        ImGui::SliderFloat("##scale", &timeScale_, 0.05F, 2.0F, "time scale %.2fx", ImGuiSliderFlags_Logarithmic);
        ImGui::SetItemTooltip("How fast simulated time runs compared with real time.\n"
                              "The physics is the same at any scale: always 1 ms steps.");
        ImGui::SameLine();
        ImGui::Text("   %.3f s", world_->time());
        ImGui::SetItemTooltip("Simulated time: %llu steps of 1 ms",
                              static_cast<unsigned long long>(world_->stepCount()));
        ImGui::SameLine();
        ImGui::TextColored(kDim, "  %zu items", world_->itemCount());
    }
    ImGui::End();
}

// How things are going, at a glance.
void App::drawStatusBar() {
    constexpr ImGuiWindowFlags kFlags =
        ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_MenuBar;
    if (ImGui::BeginViewportSideBar("##status", ImGui::GetMainViewport(), ImGuiDir_Down, ImGui::GetFrameHeight(),
                                    kFlags)) {
        if (ImGui::BeginMenuBar()) {
            ImGui::Text("%s", paused_ ? "paused" : "running");
            ImGui::Separator();
            if (behind_) {
                ImGui::TextColored(kProblem, "behind: %.2fx real time", realTimeFactor_);
                ImGui::SetItemTooltip("The line is too busy to simulate in real time (%.2f ms a step on %d threads):\n"
                                      "it runs slower instead. The physics is the same, only slower than the clock.",
                                      stepMilliseconds_, world_->physicsThreads());
            } else {
                ImGui::TextColored(kDim, "%.2fx real time", paused_ ? 0.0 : realTimeFactor_);
            }
            ImGui::Separator();
            const auto& drawn = renderer_.stats();
            ImGui::TextColored(kDim, "%.0f fps, %d draw calls, GPU %.1f ms",
                               frameSeconds_ > 0.0 ? 1.0 / frameSeconds_ : 0.0, drawn.drawCalls, drawn.gpuMilliseconds);
            ImGui::SetItemTooltip("%zu boxes drawn; CPU %.2f ms to hand them to OpenGL", drawn.boxes,
                                  drawn.cpuMilliseconds);
            ImGui::Separator();
            const bool problem =
                fileStatus_.find("can't") != std::string::npos || fileStatus_.find("isn't") != std::string::npos;
            ImGui::TextColored(problem ? kProblem : kDim, "%s",
                               fileStatus_.empty() ? savePath().string().c_str() : fileStatus_.c_str());
            if (!programError_.empty()) {
                ImGui::Separator();
                ImGui::TextColored(kProblem, "program: %s", programError_.c_str());
            }
            ImGui::EndMenuBar();
        }
    }
    ImGui::End();
}

// The control program and what it's doing.
void App::drawControlWindow() {
    if (!showControl_) {
        return;
    }
    if (ImGui::Begin("Control", &showControl_)) {
        drawControl();
    }
    ImGui::End();
}

// The mouse and the keys, in one place.
void App::drawHelp() {
    if (!showHelp_) {
        return;
    }
    ImGui::SetNextWindowSize(ImVec2(ImGui::GetFontSize() * 30.0F, 0.0F), ImGuiCond_Appearing);
    if (ImGui::Begin("Mouse and keys", &showHelp_, ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_AlwaysAutoResize)) {
        const struct {
            const char* what;
            const char* how;
        } rows[] = {{"Turn the view", "left drag (on nothing selected)"},
                    {"Move the view", "right or middle drag"},
                    {"Zoom", "wheel"},
                    {"Select", "click"},
                    {"Move the selection", "drag it: along its belt, or a conveyor across the floor"},
                    {"Fine steps", "hold Shift while dragging"},
                    {"Move or turn the selection anywhere", "drag its arrows (W) or rings (E)"},
                    {"The gizmo's axes", "T: the selection's own, or the world's"},
                    {"Turn a conveyor", "R, Shift+R"},
                    {"Copy, paste, duplicate", "Ctrl+C, Ctrl+V, Ctrl+D"},
                    {"Delete", "Del"},
                    {"Deselect, cancel placing", "Esc"},
                    {"Undo, redo", "Ctrl+Z, Ctrl+Y"},
                    {"Run or pause", "Space"},
                    {"Open, save as, new", "Ctrl+O, Ctrl+Shift+S, Ctrl+N"},
                    {"This window", "F1"}};
        if (ImGui::BeginTable("keys", 2, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit)) {
            for (const auto& row : rows) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(row.what);
                ImGui::TableNextColumn();
                ImGui::TextColored(kDim, "%s", row.how);
            }
            ImGui::EndTable();
        }
    }
    ImGui::End();
}

} // namespace fsim
