#include "app/app.hpp"
#include "app/widgets.hpp"
#include "plc/programs.hpp"
#include "plc/s7_link.hpp"
#include "plc/soft_plc_program.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fmt/format.h>
#include <imgui.h>
#include <imgui_stdlib.h>
#include <numbers>

namespace fsim {

namespace {

const char* turnName(Turn turn) {
    switch (turn) {
    case Turn::Left:
        return "turned left";
    case Turn::Right:
        return "turned right";
    case Turn::Straight:
        break;
    }
    return "straight on";
}

} // namespace

App::App(std::optional<std::filesystem::path> scenePath)
    : settings_(loadSettings(configFolder() / "settings.json")), history_(defaultScene()) {
    if (scenePath && std::filesystem::exists(*scenePath)) {
        if (!openScene(*scenePath)) {
            // Keep the unreadable file aside rather than overwrite it with the next save.
            auto aside = *scenePath;
            aside += ".broken";
            std::error_code ignored;
            std::filesystem::copy_file(*scenePath, aside, std::filesystem::copy_options::overwrite_existing, ignored);
            fileStatus_ += " (kept as " + aside.filename().string() + "; starting a new scene)";
            scenePath_ = *scenePath;
        }
    } else if (scenePath) {
        // A new scene, in the file named: it exists from the start.
        scenePath_ = std::filesystem::absolute(*scenePath).lexically_normal();
        if (auto saved = saveScene(scene(), scenePath_); saved) {
            remember(scenePath_);
            fileStatus_ = "a new scene in " + scenePath_.filename().string();
        } else {
            fileStatus_ = saved.error();
        }
    } else if (!(!settings_.recent.empty() && openScene(settings_.recent.front()))) {
        // Nothing named, nothing recent: the untitled scene, as it was left.
        if (auto untitled = loadScene(savePath())) {
            history_ = SceneHistory(std::move(*untitled));
        }
        fileStatus_ = "untitled";
    }
    if (!world_) {
        rebuildWorld();
    }
}

void App::rebuildWorld() {
    world_.reset();
    programError_.clear();
    world_ = buildWorld(scene(), [this](const Scene& s) {
        auto made = makeAnyProgram(s, sceneFolder());
        programError_ = made.error;
        return std::move(made.program);
    });
    applied_ = scene();
}

void App::replaceProgram() {
    world_->setProgram(nullptr);
    auto made = makeAnyProgram(scene(), sceneFolder());
    programError_ = made.error;
    world_->setProgram(std::move(made.program));
}

void App::frame(double realSeconds, int width, int height) {
    handleMouse(width, height);
    handleKeys();
    drawMenuBar();
    drawMainPanel();
    drawLinePanel();
    drawInspector();
    drawTriggers();
    drawIoTable();
    drawControlPanel();
    drawPlcVariables();
    drawAnalyser();
    drawCameras();
    drawFileDialog();
    drawNewConfirmation();

    // An edit is finished when no widget is being used: then it's one undo step, saved at once.
    if (!ImGui::IsAnyItemActive() && history_.commit()) {
        if (auto saved = saveScene(scene(), savePath()); saved) {
            fileStatus_ = scenePath_.empty() ? "untitled (saved in the config folder)"
                                             : "saved " + scenePath_.filename().string();
        } else {
            fileStatus_ = saved.error();
        }
    }
    keepSelectionValid();
    followScene();

    // Long pauses (the window was dragged, the computer was busy) shouldn't
    // turn into a burst of simulation: at most a quarter of a second at once.
    realSeconds = std::min(realSeconds, 0.25);
    frameSeconds_ = realSeconds;
    // A time budget for the simulation: as many steps as fit in about 25 ms
    // (judged by what steps have cost lately). A line too busy to simulate in
    // real time then runs slower than real time, instead of freezing the
    // window for a quarter of a second every frame trying to catch up.
    constexpr double kBudgetMilliseconds = 25.0;
    const int maxSteps = std::clamp(static_cast<int>(kBudgetMilliseconds / std::max(stepMilliseconds_, 1e-3)), 10, 250);
    const auto stepping = std::chrono::steady_clock::now();
    stepsLastFrame_ = paused_ ? 0 : world_->advance(realSeconds * timeScale_, maxSteps);
    if (stepsLastFrame_ > 0) {
        const double spent =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - stepping).count();
        stepMilliseconds_ = 0.8 * stepMilliseconds_ + 0.2 * spent / stepsLastFrame_; // smoothed
    }
    behind_ = !paused_ && stepsLastFrame_ == maxSteps;
    finishFiring();
    renderer_.draw(sceneBoxes(*world_, 1.0, selected_), camera_, width, height);
    if (showNames_) {
        drawNames(width, height);
    }
}

// Each device's name, at its place in the view: projected through the
// camera like a vertex, then written by ImGui behind its windows.
void App::drawNames(int width, int height) {
    const auto& io = ImGui::GetIO();
    const float aspect = height > 0 ? static_cast<float>(width) / static_cast<float>(height) : 1.0F;
    const auto viewProjection = camera_.projection(aspect) * camera_.view();
    auto* draw = ImGui::GetBackgroundDrawList();
    for (const auto& label : sceneLabels(*world_)) {
        const glm::vec4 clip = viewProjection * glm::vec4(label.position, 1.0F);
        if (clip.w <= 0.01F) {
            continue; // behind the camera
        }
        const glm::vec2 ndc = glm::vec2(clip) / clip.w;
        if (std::abs(ndc.x) > 1.1F || std::abs(ndc.y) > 1.1F) {
            continue;
        }
        // In ImGui's window points, y down (the framebuffer may have more pixels).
        const ImVec2 size = ImGui::CalcTextSize(label.text.c_str());
        const ImVec2 at((ndc.x * 0.5F + 0.5F) * io.DisplaySize.x - size.x / 2.0F,
                        (0.5F - ndc.y * 0.5F) * io.DisplaySize.y - size.y);
        const bool selected = selected_ == label.pick;
        draw->AddRectFilled(ImVec2(at.x - 3.0F, at.y - 1.0F), ImVec2(at.x + size.x + 3.0F, at.y + size.y + 1.0F),
                            selected ? IM_COL32(40, 90, 170, 200) : IM_COL32(0, 0, 0, 120), 3.0F);
        draw->AddText(at, IM_COL32(235, 238, 245, 255), label.text.c_str());
    }
}

void App::followScene() {
    if (scene() == applied_) {
        return;
    }
    if (sameLayout(applied_, scene())) {
        applyLive(*world_, scene()); // the boxes keep running
        // A new program (or a sorter set up differently) starts from scratch;
        // the boxes keep running meanwhile.
        if (!sameControl(scene(), applied_)) {
            replaceProgram();
        }
        applied_ = scene();
    } else {
        rebuildWorld(); // something moved: the run starts again
    }
}

void App::restartRun() {
    rebuildWorld();
}

// ---------------------------------------------------------------- Input

// Left drag turns around the target, right or middle drag moves it, the
// wheel zooms; a left click (without dragging) selects what's under the
// mouse. Only when the mouse isn't over a panel.
void App::handleMouse(int width, int height) {
    const auto& io = ImGui::GetIO();
    if (io.WantCaptureMouse) {
        return;
    }
    const auto windowHeight = static_cast<float>(height);
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Left, 3.0F)) {
        camera_.rotate(io.MouseDelta.x, io.MouseDelta.y, windowHeight);
    }
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Right, 0.0F) || ImGui::IsMouseDragging(ImGuiMouseButton_Middle, 0.0F)) {
        camera_.pan(io.MouseDelta.x, io.MouseDelta.y, windowHeight);
    }
    if (io.MouseWheel != 0.0F) {
        camera_.zoom(io.MouseWheel);
    }
    // A click is a press and release that didn't move far (otherwise it was a drag).
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Left) && io.MouseDragMaxDistanceSqr[ImGuiMouseButton_Left] < 9.0F) {
        // ImGui's mouse is in window points; the framebuffer may have more pixels (HiDPI).
        const float scaleX = static_cast<float>(width) / std::max(io.DisplaySize.x, 1.0F);
        const float scaleY = static_cast<float>(height) / std::max(io.DisplaySize.y, 1.0F);
        const auto ray = camera_.rayThrough(io.MousePos.x * scaleX, io.MousePos.y * scaleY, static_cast<float>(width),
                                            static_cast<float>(height));
        selected_ = pickAt(sceneBoxes(*world_), ray.origin, ray.direction);
    }
}

void App::handleKeys() {
    const auto& io = ImGui::GetIO();
    if (io.WantTextInput) {
        return; // the keys belong to the text field
    }
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Z)) {
        io.KeyShift ? history_.redo() : history_.undo();
    }
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Y)) {
        history_.redo();
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Delete) && !io.WantCaptureKeyboard) {
        deleteSelected();
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
        selected_.reset();
    }
}

void App::deleteSelected() {
    if (!selected_) {
        return;
    }
    switch (selected_->kind) {
    case Pick::Kind::Conveyor:
        removeConveyor(scene(), selected_->index);
        break;
    case Pick::Kind::Photocell:
        removePhotocell(scene(), selected_->index);
        break;
    case Pick::Kind::Spawner:
        removeSpawner(scene(), selected_->index);
        break;
    case Pick::Kind::Pusher:
        removePusher(scene(), selected_->index);
        break;
    case Pick::Kind::Diverter:
        removeDiverter(scene(), selected_->index);
        break;
    case Pick::Kind::Stopper:
        removeStopper(scene(), selected_->index);
        break;
    case Pick::Kind::Bin:
        removeBin(scene(), selected_->index);
        break;
    case Pick::Kind::Camera:
        removeCamera(scene(), selected_->index);
        break;
    case Pick::Kind::Proximity:
        removeProximity(scene(), selected_->index);
        break;
    case Pick::Kind::Laser:
        removeLaser(scene(), selected_->index);
        break;
    case Pick::Kind::LightCurtain:
        removeLightCurtain(scene(), selected_->index);
        break;
    case Pick::Kind::None:
        break;
    }
    selected_.reset();
}

// After an undo, the selected thing may not exist any more.
void App::keepSelectionValid() {
    if (!selected_) {
        return;
    }
    const auto& s = scene();
    std::size_t count = 0;
    switch (selected_->kind) {
    case Pick::Kind::Conveyor:
        count = s.conveyors.size();
        break;
    case Pick::Kind::Photocell:
        count = s.photocells.size();
        break;
    case Pick::Kind::Spawner:
        count = s.spawners.size();
        break;
    case Pick::Kind::Pusher:
        count = s.pushers.size();
        break;
    case Pick::Kind::Diverter:
        count = s.diverters.size();
        break;
    case Pick::Kind::Stopper:
        count = s.stoppers.size();
        break;
    case Pick::Kind::Bin:
        count = s.bins.size();
        break;
    case Pick::Kind::Camera:
        count = s.cameras.size();
        break;
    case Pick::Kind::Proximity:
        count = s.proximities.size();
        break;
    case Pick::Kind::Laser:
        count = s.lasers.size();
        break;
    case Pick::Kind::LightCurtain:
        count = s.curtains.size();
        break;
    case Pick::Kind::None:
        break;
    }
    if (selected_->index >= count) {
        selected_.reset();
    }
}

std::optional<std::size_t> App::selectedConveyor() const {
    if (!selected_) {
        return std::nullopt;
    }
    const auto& s = history_.current();
    switch (selected_->kind) {
    case Pick::Kind::Conveyor:
        return selected_->index;
    case Pick::Kind::Photocell:
        return s.photocells[selected_->index].conveyor;
    case Pick::Kind::Spawner:
        return s.spawners[selected_->index].conveyor;
    case Pick::Kind::Pusher:
        return s.pushers[selected_->index].conveyor;
    case Pick::Kind::Diverter:
        return s.diverters[selected_->index].conveyor;
    case Pick::Kind::Stopper:
        return s.stoppers[selected_->index].conveyor;
    case Pick::Kind::Bin:
        return s.bins[selected_->index].conveyor;
    case Pick::Kind::Camera:
        return s.cameras[selected_->index].conveyor;
    case Pick::Kind::Proximity:
        return s.proximities[selected_->index].conveyor;
    case Pick::Kind::Laser:
        return s.lasers[selected_->index].conveyor;
    case Pick::Kind::LightCurtain:
        return s.curtains[selected_->index].conveyor;
    case Pick::Kind::None:
        break;
    }
    return std::nullopt;
}

// ---------------------------------------------------------------- Panels

void App::drawMainPanel() {
    ImGui::SetNextWindowPos(ImVec2(10, 32), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(340, 0), ImGuiCond_FirstUseEver);
    ImGui::Begin("factory_sim");

    ImGui::SeparatorText("Simulation");
    ImGui::Text("Time %.3f s", world_->time());
    ImGui::SameLine();
    ImGui::TextColored(kDim, "(%llu steps of 1 ms)", static_cast<unsigned long long>(world_->stepCount()));
    if (ImGui::Button(paused_ ? "Run" : "Pause")) {
        paused_ = !paused_;
    }
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
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::SliderFloat("time scale", &timeScale_, 0.05F, 2.0F, "%.2fx", ImGuiSliderFlags_Logarithmic);
    ImGui::SetItemTooltip("How fast simulated time runs compared with real time.\n"
                          "The physics is the same at any scale: always 1 ms steps.");
    ImGui::TextColored(kDim, "%d steps this frame, %.1f ms per frame", stepsLastFrame_, frameSeconds_ * 1000.0);
    if (behind_) {
        ImGui::TextColored(kProblem, "behind: simulating at most %.0f ms per frame (%.2f ms a step, %d threads)", 25.0,
                           stepMilliseconds_, world_->physicsThreads());
        ImGui::SetItemTooltip("The line is too busy to simulate in real time: it runs slower instead.\n"
                              "The physics is the same, only slower than the clock on the wall.");
    }
    const auto& drawn = renderer_.stats();
    ImGui::TextColored(kDim, "drawing: %zu boxes, %d draw calls, CPU %.2f ms, GPU %.2f ms", drawn.boxes,
                       drawn.drawCalls, drawn.cpuMilliseconds, drawn.gpuMilliseconds);
    ImGui::SetItemTooltip("CPU: handing the frame to OpenGL. GPU: drawing it (a timer query, a frame late).");
    ImGui::Text("%zu boxes", world_->itemCount());
    ImGui::SameLine();
    if (ImGui::SmallButton("Clear boxes")) {
        world_->clearItems();
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Restart")) {
        restartRun();
    }
    ImGui::SetItemTooltip("Start the run again: no boxes, time 0. (A change to the layout does this too.)");

    drawControl();

    ImGui::SeparatorText("Scene");
    ImGui::BeginDisabled(!history_.canUndo());
    if (ImGui::Button("Undo")) {
        history_.undo();
    }
    ImGui::EndDisabled();
    ImGui::SetItemTooltip("Ctrl+Z");
    ImGui::SameLine();
    ImGui::BeginDisabled(!history_.canRedo());
    if (ImGui::Button("Redo")) {
        history_.redo();
    }
    ImGui::EndDisabled();
    ImGui::SetItemTooltip("Ctrl+Y");
    ImGui::SameLine();
    ImGui::TextColored(kDim, "saved after every change");
    ImGui::PushTextWrapPos(0.0F);
    ImGui::TextColored(fileStatus_.find("can't") != std::string::npos || fileStatus_.find("isn't") != std::string::npos
                           ? kProblem
                           : kDim,
                       "%s", fileStatus_.empty() ? savePath().string().c_str() : fileStatus_.c_str());
    ImGui::PopTextWrapPos();

    ImGui::SeparatorText("Camera");
    ImGui::TextColored(kDim,
                       "Left drag: turn. Right drag: move. Wheel: zoom.\nClick: select. Del: delete. Esc: deselect.");
    ImGui::End();
}

// Everything in the line, to select, and buttons to add and remove things.
void App::drawLinePanel() {
    if (!showLine_) {
        return; // hidden (View menu)
    }
    ImGui::SetNextWindowPos(ImVec2(10, 472), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(340, 330), ImGuiCond_FirstUseEver);
    ImGui::Begin("Line", &showLine_);
    auto& s = scene();
    const auto selectable = [&](const std::string& label, Pick pick) {
        if (ImGui::Selectable(label.c_str(), selected_ == pick)) {
            selected_ = pick;
        }
    };
    ImGui::SeparatorText("Conveyors");
    for (std::size_t i = 0; i < s.conveyors.size(); ++i) {
        const auto& c = s.conveyors[i];
        const auto detail = c.follows && *c.follows < s.conveyors.size()
                                ? fmt::format("  after {}, {}", s.conveyors[*c.follows].spec.name, turnName(c.turn))
                                : std::string("  on its own");
        selectable(fmt::format("{}  {:.2f} m{}###c{}", c.spec.name, c.spec.length, detail, i),
                   Pick{Pick::Kind::Conveyor, i});
    }
    const auto conveyorName = [&](std::size_t c) {
        return c < s.conveyors.size() ? s.conveyors[c].spec.name : std::string("?");
    };
    ImGui::SeparatorText("Sensors and spawners");
    for (std::size_t i = 0; i < s.photocells.size(); ++i) {
        const auto& p = s.photocells[i];
        const auto on = p.conveyor < s.conveyors.size() ? s.conveyors[p.conveyor].spec.name : std::string("?");
        selectable(fmt::format("{}  photocell on {} at {:.2f} m###p{}", p.name, on, p.along, i),
                   Pick{Pick::Kind::Photocell, i});
    }
    for (std::size_t i = 0; i < s.proximities.size(); ++i) {
        const auto& p = s.proximities[i];
        selectable(fmt::format("{}  {} switch on {} at {:.2f} m###px{}", p.name,
                               p.inductive ? "inductive" : "capacitive", conveyorName(p.conveyor), p.along, i),
                   Pick{Pick::Kind::Proximity, i});
    }
    for (std::size_t i = 0; i < s.lasers.size(); ++i) {
        const auto& l = s.lasers[i];
        selectable(fmt::format("{}  laser over {} at {:.2f} m###la{}", l.name, conveyorName(l.conveyor), l.along, i),
                   Pick{Pick::Kind::Laser, i});
    }
    for (std::size_t i = 0; i < s.curtains.size(); ++i) {
        const auto& c = s.curtains[i];
        selectable(
            fmt::format("{}  light curtain on {} at {:.2f} m###lc{}", c.name, conveyorName(c.conveyor), c.along, i),
            Pick{Pick::Kind::LightCurtain, i});
    }
    for (std::size_t i = 0; i < s.spawners.size(); ++i) {
        const auto& sp = s.spawners[i];
        const auto on = sp.conveyor < s.conveyors.size() ? s.conveyors[sp.conveyor].spec.name : std::string("?");
        selectable(fmt::format("spawner on {}, every {:.2f} s###s{}", on, sp.interval, i),
                   Pick{Pick::Kind::Spawner, i});
    }
    ImGui::SeparatorText("Actuators and bins");
    for (std::size_t i = 0; i < s.pushers.size(); ++i) {
        const auto& a = s.pushers[i];
        selectable(fmt::format("{}  pusher on {} at {:.2f} m###pu{}", a.name, conveyorName(a.conveyor), a.along, i),
                   Pick{Pick::Kind::Pusher, i});
    }
    for (std::size_t i = 0; i < s.diverters.size(); ++i) {
        const auto& a = s.diverters[i];
        selectable(fmt::format("{}  diverter on {} at {:.2f} m###di{}", a.name, conveyorName(a.conveyor), a.along, i),
                   Pick{Pick::Kind::Diverter, i});
    }
    for (std::size_t i = 0; i < s.stoppers.size(); ++i) {
        const auto& a = s.stoppers[i];
        selectable(fmt::format("{}  stopper on {} at {:.2f} m###st{}", a.name, conveyorName(a.conveyor), a.along, i),
                   Pick{Pick::Kind::Stopper, i});
    }
    for (std::size_t i = 0; i < s.bins.size(); ++i) {
        const auto& b = s.bins[i];
        const auto count = i < world_->bins().size() ? world_->bins()[i].count : 0;
        selectable(fmt::format("{}  bin beside {}: {} boxes###bi{}", b.name, conveyorName(b.conveyor), count, i),
                   Pick{Pick::Kind::Bin, i});
    }

    if (!s.cameras.empty()) {
        ImGui::SeparatorText("Cameras");
    }
    for (std::size_t i = 0; i < s.cameras.size(); ++i) {
        const auto& c = s.cameras[i];
        selectable(fmt::format("{}  camera over {} at {:.3f} m###ca{}", c.name, conveyorName(c.conveyor), c.along, i),
                   Pick{Pick::Kind::Camera, i});
    }

    ImGui::SeparatorText("Add");
    const auto conveyor = selectedConveyor();
    const bool conveyorSelected = selected_ && selected_->kind == Pick::Kind::Conveyor;
    ImGui::BeginDisabled(!conveyorSelected);
    ImGui::TextUnformatted("After the selected conveyor:");
    for (const auto turn : {Turn::Straight, Turn::Left, Turn::Right}) {
        if (turn != Turn::Straight) {
            ImGui::SameLine();
        }
        const char* label = turn == Turn::Straight ? "Straight" : turn == Turn::Left ? "Left" : "Right";
        if (ImGui::SmallButton(label)) {
            selected_ = Pick{Pick::Kind::Conveyor, addConveyorAfter(s, selected_->index, turn)};
        }
    }
    ImGui::EndDisabled();
    if (ImGui::SmallButton("New conveyor")) {
        selected_ = Pick{Pick::Kind::Conveyor, addConveyor(s)};
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(!conveyor);
    if (ImGui::SmallButton("Photocell")) {
        selected_ = Pick{Pick::Kind::Photocell, addPhotocell(s, *conveyor)};
    }
    ImGui::SetItemTooltip("Halfway along the selected conveyor");
    ImGui::SameLine();
    if (ImGui::SmallButton("Spawner")) {
        selected_ = Pick{Pick::Kind::Spawner, addSpawner(s, *conveyor)};
    }
    ImGui::SetItemTooltip("At the start of the selected conveyor");
    if (ImGui::SmallButton("Pusher")) {
        selected_ = Pick{Pick::Kind::Pusher, addPusher(s, *conveyor)};
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Diverter")) {
        selected_ = Pick{Pick::Kind::Diverter, addDiverter(s, *conveyor)};
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Stopper")) {
        selected_ = Pick{Pick::Kind::Stopper, addStopper(s, *conveyor)};
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Bin")) {
        selected_ = Pick{Pick::Kind::Bin, addBin(s, *conveyor)};
    }
    ImGui::SetItemTooltip("Beside the selected conveyor, across from its pusher if it has one");
    ImGui::SameLine();
    if (ImGui::SmallButton("Camera")) {
        selected_ = Pick{Pick::Kind::Camera, addCamera(s, *conveyor)};
    }
    ImGui::SetItemTooltip("Over the selected conveyor, looking down");
    if (ImGui::SmallButton("Proximity")) {
        selected_ = Pick{Pick::Kind::Proximity, addProximity(s, *conveyor)};
    }
    ImGui::SetItemTooltip("An inductive proximity switch on the left of the selected conveyor");
    ImGui::SameLine();
    if (ImGui::SmallButton("Laser")) {
        selected_ = Pick{Pick::Kind::Laser, addLaser(s, *conveyor)};
    }
    ImGui::SetItemTooltip("A laser distance sensor over the selected conveyor, looking down");
    ImGui::SameLine();
    if (ImGui::SmallButton("Light curtain")) {
        selected_ = Pick{Pick::Kind::LightCurtain, addLightCurtain(s, *conveyor)};
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(!selected_);
    if (ImGui::SmallButton("Delete")) {
        deleteSelected();
    }
    ImGui::EndDisabled();
    ImGui::End();
}

void App::drawInspector() {
    if (!showInspector_) {
        return; // hidden (View menu)
    }
    ImGui::SetNextWindowPos(ImVec2(1040, 262), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(350, 0), ImGuiCond_FirstUseEver);
    ImGui::Begin("Inspector", &showInspector_);
    if (!selected_) {
        ImGui::TextColored(kDim, "Click something in the view, or in the Line panel.");
    } else {
        switch (selected_->kind) {
        case Pick::Kind::Conveyor:
            drawConveyorInspector(selected_->index);
            break;
        case Pick::Kind::Photocell:
            drawPhotocellInspector(selected_->index);
            break;
        case Pick::Kind::Spawner:
            drawSpawnerInspector(selected_->index);
            break;
        case Pick::Kind::Pusher:
            drawPusherInspector(selected_->index);
            break;
        case Pick::Kind::Diverter:
            drawDiverterInspector(selected_->index);
            break;
        case Pick::Kind::Stopper:
            drawStopperInspector(selected_->index);
            break;
        case Pick::Kind::Bin:
            drawBinInspector(selected_->index);
            break;
        case Pick::Kind::Camera:
            drawCameraInspector(selected_->index);
            break;
        case Pick::Kind::Proximity:
            drawProximityInspector(selected_->index);
            break;
        case Pick::Kind::Laser:
            drawLaserInspector(selected_->index);
            break;
        case Pick::Kind::LightCurtain:
            drawCurtainInspector(selected_->index);
            break;
        case Pick::Kind::None:
            break;
        }
    }
    ImGui::End();
}

void App::drawConveyorInspector(std::size_t index) {
    auto& s = scene();
    auto& placed = s.conveyors[index];
    auto& spec = placed.spec;
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::InputText("name", &spec.name);

    ImGui::SeparatorText("Place");
    if (placed.follows && *placed.follows < s.conveyors.size()) {
        ImGui::Text("After %s, %s", s.conveyors[*placed.follows].spec.name.c_str(), turnName(placed.turn));
        int turn = static_cast<int>(placed.turn);
        ImGui::SetNextItemWidth(kFieldWidth);
        if (ImGui::Combo("turn", &turn, "straight on\0turned left\0turned right\0")) {
            placed.turn = static_cast<Turn>(turn);
        }
        if (ImGui::SmallButton("Detach")) {
            // Stays where it is, but on its own: moving the one before won't move it any more.
            const auto where = placeConveyors(s)[index];
            spec.start = where.start;
            spec.heading = where.heading;
            placed.follows.reset();
            placed.turn = Turn::Straight;
        }
        ImGui::SetItemTooltip("Keep it where it is, but no longer attached to the conveyor before it");
    } else {
        ImGui::SetNextItemWidth(kFieldWidth);
        ImGui::DragFloat("start x", &spec.start.x, 0.01F, -50.0F, 50.0F, "%.3f m");
        ImGui::SetNextItemWidth(kFieldWidth);
        ImGui::DragFloat("start z", &spec.start.z, 0.01F, -50.0F, 50.0F, "%.3f m");
        float degrees = spec.heading * kDegrees;
        ImGui::SetNextItemWidth(kFieldWidth);
        if (ImGui::SliderFloat("heading", &degrees, -180.0F, 180.0F, "%.0f deg")) {
            spec.heading = degrees / kDegrees;
        }
        ImGui::SetItemTooltip("Direction of travel: 0 = along +X, 90 = along -Z");
    }

    ImGui::Checkbox("guide left", &spec.guideLeft);
    ImGui::SameLine();
    ImGui::Checkbox("guide right", &spec.guideRight);
    ImGui::SetItemTooltip(
        "Rails along the belt's edges that keep boxes on it (left and right looking along the travel)");

    ImGui::SeparatorText("Size");
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::SliderFloat("length", &spec.length, 0.5F, 10.0F, "%.2f m");
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::SliderFloat("width", &spec.width, 0.15F, 1.2F, "%.2f m");
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::SliderFloat("belt height", &spec.beltHeight, 0.3F, 1.5F, "%.2f m");
    ImGui::TextColored(kDim, "Changing the place or size restarts the run.");

    ImGui::SeparatorText("Speed");
    if (spec.drive.fitted) {
        ImGui::TextColored(kDim, "With a drive, the PLC sets the speed: the target speed is only\n"
                                 "used without a control program, and the run output not at all.");
    }
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::SliderFloat("target speed", &spec.targetSpeed, 0.0F, 3.0F, "%.2f m/s");
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::SliderFloat("acceleration", &spec.acceleration, 0.1F, 10.0F, "%.1f m/s²", ImGuiSliderFlags_Logarithmic);
    ImGui::SetItemTooltip("Like a VFD's ramp: 1 m/s² takes 1 s from 0 to 1 m/s.");
    if (index < world_->conveyors().size()) {
        const auto& running = world_->conveyors()[index];
        const float fraction = spec.targetSpeed > 0.0F ? running.speed / spec.targetSpeed : 0.0F;
        ImGui::ProgressBar(std::clamp(fraction, 0.0F, 1.0F), ImVec2(kFieldWidth, 0.0F), "");
        ImGui::SameLine();
        ImGui::Text("%.3f m/s", running.speed);
        ImGui::TextColored(kDim, "at this speed a box moves %.2f mm per 1 ms step", running.speed); // m/s = mm/ms
    }

    ImGui::SeparatorText("I/O");
    addressField("run output", spec.runOutput, IoAddress::Area::Output, IoAddress::Size::Bit);
    ImGui::SetItemTooltip(
        "The motor runs while this output is on (only with a control program).\nEmpty: it always runs.");
    addressField("encoder count", spec.encoderInput, IoAddress::Area::Input, IoAddress::Size::DoubleWord);

    ImGui::SeparatorText("Drive on the bus");
    if (ImGui::Checkbox("drive on the bus (PROFIdrive telegram 1)", &spec.drive.fitted) && spec.drive.fitted) {
        spec.drive.fitted = false;
        fitDrive(s, index); // with free addresses
    }
    if (spec.drive.fitted) {
        drawDrive(index);
    } else {
        ImGui::TextColored(kDim, "Without one, the motor is on or off (run output) at the target speed.");
    }

    ImGui::SeparatorText("Encoder");
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::SliderInt("pulses/rev", &spec.encoder.pulsesPerRevolution, 100, 5000);
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::SliderFloat("roller", &spec.encoder.rollerDiameter, 0.02F, 0.2F, "%.3f m");
    const double perMetre = spec.encoder.pulsesPerMetre();
    if (index < world_->conveyors().size()) {
        const auto& running = world_->conveyors()[index];
        ImGui::Text("count %lld", static_cast<long long>(running.encoderCount()));
        ImGui::TextColored(kDim, "%.0f pulses/m, %.3f mm per pulse, %.0f Hz now", perMetre, 1000.0 / perMetre,
                           running.speed * perMetre);
    }
}

void App::drawPhotocellInspector(std::size_t index) {
    auto& s = scene();
    auto& spec = s.photocells[index];
    if (index < world_->photocells().size()) {
        const auto& running = world_->photocells()[index];
        const auto on = running.output.value();
        ImGui::ColorButton("##led", on ? ImVec4(1.0F, 0.85F, 0.1F, 1.0F) : ImVec4(0.25F, 0.22F, 0.12F, 1.0F),
                           ImGuiColorEditFlags_NoTooltip, ImVec2(14, 14));
        ImGui::SameLine();
        ImGui::Text("output %s", on ? "ON" : "off");
        ImGui::SameLine();
        ImGui::TextColored(kDim, "  beam %.0f %% covered", running.covered * 100.0F);
    }
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::InputText("name", &spec.name);
    addressField("input", spec.input, IoAddress::Area::Input, IoAddress::Size::Bit);
    ImGui::Checkbox("sees only bad (NOK) boxes", &spec.onlyNok);
    ImGui::SetItemTooltip("A stand-in for an inspection system (a camera, a colour sensor)\n"
                          "whose output says \"this one is bad\".");
    conveyorCombo("on conveyor", spec.conveyor, s);
    const float length = spec.conveyor < s.conveyors.size() ? s.conveyors[spec.conveyor].spec.length : 1.0F;
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::SliderFloat("position", &spec.along, 0.0F, length, "%.3f m");
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::SliderFloat("height", &spec.height, 0.005F, 0.3F, "%.3f m above belt");
    scaledSlider("response", spec.responseTime, 1000.0, 0.0F, 20.0F, "%.2f ms");
    ImGui::SetItemTooltip("From the beam being blocked to the output switching.\n"
                          "Typical photocells: 0.25 to 2 ms.");
    scaledSlider("beam", spec.beamDiameter, 1000.0F, 1.0F, 30.0F, "%.1f mm");
    if (scaledSlider("on above", spec.onFraction, 100.0F, 10.0F, 95.0F, "%.0f %% covered")) {
        spec.offFraction = std::min(spec.offFraction, spec.onFraction);
    }
    if (scaledSlider("off below", spec.offFraction, 100.0F, 5.0F, 90.0F, "%.0f %% covered")) {
        spec.offFraction = std::min(spec.offFraction, spec.onFraction);
    }
    ImGui::TextColored(kDim, "smallest object it sees: %.1f mm long", spec.onFraction * spec.beamDiameter * 1000.0F);
}

void App::drawSpawnerInspector(std::size_t index) {
    auto& s = scene();
    auto& spawner = s.spawners[index];
    conveyorCombo("on conveyor", spawner.conveyor, s);
    ImGui::Checkbox("spawn boxes", &spawner.enabled);
    scaledSlider("bad boxes", spawner.nokFraction, 100.0F, 0.0F, 100.0F, "%.0f %%");
    ImGui::SetItemTooltip("The share of boxes that are bad (NOK, red), chosen at random");
    scaledSlider("every", spawner.interval, 1.0, 0.2F, 5.0F, "%.2f s");
    scaledSlider("jitter", spawner.intervalJitter, 1.0, 0.0F, 1.0F, "± %.2f s");
    int shape = spawner.shape == Shape::Tin ? 1 : 0;
    ImGui::SetNextItemWidth(kFieldWidth);
    if (ImGui::Combo("makes", &shape, "boxes\0tins\0")) {
        spawner.shape = shape == 1 ? Shape::Tin : Shape::Box;
    }
    ImGui::Checkbox("arriving at belt speed", &spawner.atBeltSpeed);
    ImGui::SetItemTooltip("As from an infeed conveyor. Off: put down standing still, so the belt has to\n"
                          "speed them up first, and the spawner waits until there's room.");
    ImGui::SetNextItemWidth(kFieldWidth);
    if (spawner.shape == Shape::Tin) {
        float size[2] = {spawner.size.x, spawner.size.y};
        if (ImGui::SliderFloat2("size", size, 0.02F, 0.35F, "%.3f m")) {
            spawner.size = {size[0], size[1], size[0]};
        }
        ImGui::SetItemTooltip("Diameter, height");
    } else {
        ImGui::SliderFloat3("size", &spawner.size.x, 0.02F, 0.35F, "%.2f m");
        ImGui::SetItemTooltip("Length along the belt, height, width");
    }
    const float length = spawner.conveyor < s.conveyors.size() ? s.conveyors[spawner.conveyor].spec.length : 1.0F;
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::SliderFloat("where", &spawner.offset, 0.0F, length, "%.2f m from the start");
    if (spawner.conveyor < s.conveyors.size()) {
        const float speed = s.conveyors[spawner.conveyor].spec.targetSpeed;
        ImGui::TextColored(kDim, "gap between them at full speed: %.3f m",
                           speed * static_cast<float>(spawner.interval) - spawner.size.x);
    }
    if (ImGui::Button("Spawn one now") && spawner.conveyor < world_->conveyors().size()) {
        const auto& c = world_->conveyors()[spawner.conveyor];
        world_->spawnItem(c.beltPoint(spawner.offset) + glm::vec3(0.0F, spawner.size.y / 2.0F + 0.002F, 0.0F),
                          spawner.size, {0.85F, 0.30F, 0.30F}, c.spec.heading, false, spawner.shape,
                          spawner.atBeltSpeed ? c.direction() * c.speed : glm::vec3(0.0F));
    }
}

// The last triggers of the selected photocell (or the first one): when the
// beam was blocked, when the output switched, and how far past the beam the
// box's front was by then, also as a PLC scanning every few ms would see it.
void App::drawTriggers() {
    if (!showTriggers_) {
        return; // hidden (View menu)
    }
    if (world_->photocells().empty()) {
        return;
    }
    const auto index =
        selected_ && selected_->kind == Pick::Kind::Photocell && selected_->index < world_->photocells().size()
            ? selected_->index
            : 0;
    const auto& photocell = world_->photocells()[index];
    ImGui::SetNextWindowPos(ImVec2(1040, 32), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(350, 220), ImGuiCond_FirstUseEver);
    ImGui::Begin("Triggers", &showTriggers_);
    ImGui::Text("%s", photocell.spec.name.c_str());
    ImGui::SameLine();
    ImGui::SetNextItemWidth(140.0F);
    ImGui::SliderFloat("PLC scan", &scanMilliseconds_, 1.0F, 50.0F, "%.0f ms");
    ImGui::SetItemTooltip("A PLC reads its inputs once per cycle: the trigger reaches the program\n"
                          "at the next cycle start after the output switches.");
    const double scan = scanMilliseconds_ / 1000.0;
    if (ImGui::BeginTable("triggers", 6, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY)) {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("box");
        ImGui::TableSetupColumn("beam s");
        ImGui::TableSetupColumn("output s");
        ImGui::TableSetupColumn("error mm");
        ImGui::TableSetupColumn("at scan mm");
        ImGui::TableSetupColumn("encoder");
        ImGui::TableHeadersRow();
        for (const auto& trigger : photocell.triggers) {
            // The first PLC cycle starting at or after the output time.
            const double seen = std::ceil(trigger.outputTime / scan - 1e-9) * scan;
            const double atScan = trigger.positionError + trigger.speed * (seen - trigger.outputTime);
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%llu", static_cast<unsigned long long>(trigger.item));
            ImGui::TableNextColumn();
            ImGui::Text("%.5f", trigger.beamTime);
            ImGui::TableNextColumn();
            ImGui::Text("%.5f", trigger.outputTime);
            ImGui::TableNextColumn();
            ImGui::Text("%.2f", trigger.positionError * 1000.0F);
            ImGui::TableNextColumn();
            ImGui::Text("%.2f", atScan * 1000.0);
            ImGui::TableNextColumn();
            ImGui::Text("%lld", static_cast<long long>(trigger.encoderCount));
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

// ---------------------------------------------------------------- Actuators

namespace {

void led(const char* label, bool on) {
    ImGui::ColorButton(label, on ? ImVec4(1.0F, 0.85F, 0.1F, 1.0F) : ImVec4(0.25F, 0.22F, 0.12F, 1.0F),
                       ImGuiColorEditFlags_NoTooltip, ImVec2(14, 14));
}

void sideCombo(Side& side) {
    int value = side == Side::Left ? 0 : 1;
    ImGui::SetNextItemWidth(kFieldWidth);
    if (ImGui::Combo("side", &value, "left\0right\0")) {
        side = value == 0 ? Side::Left : Side::Right;
    }
    ImGui::SetItemTooltip("Left or right, looking along the belt's travel");
}

void strokeTimes(StrokeSpec& stroke) {
    scaledSlider("dead time", stroke.deadTime, 1000.0, 0.0F, 100.0F, "%.0f ms");
    ImGui::SetItemTooltip("From the valve switching to the movement starting: air flowing, pressure building");
    scaledSlider("out in", stroke.extendTime, 1000.0, 20.0F, 2000.0F, "%.0f ms");
    scaledSlider("back in", stroke.retractTime, 1000.0, 20.0F, 2000.0F, "%.0f ms");
}

template <class Spec> void ioFields(Spec& spec) {
    ImGui::SeparatorText("I/O");
    addressField("valve", spec.valveOutput, IoAddress::Area::Output, IoAddress::Size::Bit);
    addressField("extended", spec.extendedInput, IoAddress::Area::Input, IoAddress::Size::Bit);
    addressField("retracted", spec.retractedInput, IoAddress::Area::Input, IoAddress::Size::Bit);
}

} // namespace

// The valve, by hand: Extend and Retract switch it; Fire once extends and
// retracts again as soon as the cylinder is fully out (as a reject would).
// The LEDs are the cylinder's end position sensors.
void App::drawStrokeControls(Stroke& stroke, Pick pick) {
    ImGui::SeparatorText("By hand");
    const double now = world_->time();
    if (ImGui::Button("Extend")) {
        stroke.command(now, true);
    }
    ImGui::SameLine();
    if (ImGui::Button("Retract")) {
        stroke.command(now, false);
    }
    ImGui::SameLine();
    if (ImGui::Button("Fire once")) {
        stroke.command(now, true);
        firing_.push_back(pick);
    }
    ImGui::Text("valve %s", stroke.commanded() ? "OUT" : "back");
    ImGui::SameLine();
    led("##retracted", stroke.retracted().value());
    ImGui::SameLine();
    ImGui::TextUnformatted("retracted");
    ImGui::SameLine();
    led("##extended", stroke.extended().value());
    ImGui::SameLine();
    ImGui::TextUnformatted("extended");
    ImGui::ProgressBar(static_cast<float>(stroke.position()), ImVec2(kFieldWidth, 0.0F), "");
    ImGui::SameLine();
    ImGui::TextColored(kDim, "stroke");
    if (world_->program() != nullptr) {
        ImGui::TextColored(kDim, "A control program is running: it sets the valve every scan, over these buttons.");
    }
}

void App::finishFiring() {
    std::erase_if(firing_, [&](const Pick& pick) {
        Stroke* stroke = nullptr;
        if (pick.kind == Pick::Kind::Pusher && pick.index < world_->pushers().size()) {
            stroke = &world_->pushers()[pick.index].stroke;
        } else if (pick.kind == Pick::Kind::Diverter && pick.index < world_->diverters().size()) {
            stroke = &world_->diverters()[pick.index].stroke;
        } else if (pick.kind == Pick::Kind::Stopper && pick.index < world_->stoppers().size()) {
            stroke = &world_->stoppers()[pick.index].stroke;
        }
        if (stroke == nullptr) {
            return true; // gone (the run restarted)
        }
        if (stroke->extended().value()) {
            stroke->command(world_->time(), false);
            return true;
        }
        return false;
    });
}

void App::drawPusherInspector(std::size_t index) {
    auto& s = scene();
    auto& spec = s.pushers[index];
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::InputText("name", &spec.name);
    conveyorCombo("on conveyor", spec.conveyor, s);
    const float length = spec.conveyor < s.conveyors.size() ? s.conveyors[spec.conveyor].spec.length : 1.0F;
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::SliderFloat("position", &spec.along, 0.0F, length, "%.3f m");
    sideCombo(spec.side);
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::SliderFloat("paddle", &spec.paddleLength, 0.05F, 1.0F, "%.2f m long");
    strokeTimes(spec.stroke);
    ioFields(spec);
    ImGui::TextColored(kDim, "Pushes boxes off the other side: put a bin there.");
    if (index < world_->pushers().size()) {
        drawStrokeControls(world_->pushers()[index].stroke, Pick{Pick::Kind::Pusher, index});
    }
}

void App::drawDiverterInspector(std::size_t index) {
    auto& s = scene();
    auto& spec = s.diverters[index];
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::InputText("name", &spec.name);
    conveyorCombo("on conveyor", spec.conveyor, s);
    const float length = spec.conveyor < s.conveyors.size() ? s.conveyors[spec.conveyor].spec.length : 1.0F;
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::SliderFloat("pivot at", &spec.along, 0.0F, length, "%.3f m");
    sideCombo(spec.side);
    scaledSlider("angle", spec.angle, kDegrees, 15.0F, 70.0F, "%.0f deg");
    strokeTimes(spec.stroke);
    ioFields(spec);
    ImGui::TextColored(kDim, "Swung out, it guides boxes off the other side.");
    if (index < world_->diverters().size()) {
        drawStrokeControls(world_->diverters()[index].stroke, Pick{Pick::Kind::Diverter, index});
    }
}

void App::drawStopperInspector(std::size_t index) {
    auto& s = scene();
    auto& spec = s.stoppers[index];
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::InputText("name", &spec.name);
    conveyorCombo("on conveyor", spec.conveyor, s);
    const float length = spec.conveyor < s.conveyors.size() ? s.conveyors[spec.conveyor].spec.length : 1.0F;
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::SliderFloat("position", &spec.along, 0.0F, length, "%.3f m");
    scaledSlider("rises", spec.height, 1000.0F, 10.0F, 150.0F, "%.0f mm above the belt");
    strokeTimes(spec.stroke);
    ioFields(spec);
    ImGui::TextColored(kDim, "Raised, boxes stop against it while the belt slides under them.");
    if (index < world_->stoppers().size()) {
        drawStrokeControls(world_->stoppers()[index].stroke, Pick{Pick::Kind::Stopper, index});
    }
}

void App::drawBinInspector(std::size_t index) {
    auto& s = scene();
    auto& spec = s.bins[index];
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::InputText("name", &spec.name);
    conveyorCombo("beside conveyor", spec.conveyor, s);
    const float length = spec.conveyor < s.conveyors.size() ? s.conveyors[spec.conveyor].spec.length : 1.0F;
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::SliderFloat("position", &spec.along, 0.0F, length, "%.3f m");
    sideCombo(spec.side);
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::SliderFloat3("size", &spec.size.x, 0.2F, 1.0F, "%.2f m");
    ImGui::SetItemTooltip("Along the belt, height, out from the belt");
    if (index < world_->bins().size()) {
        auto& bin = world_->bins()[index];
        ImGui::Text("%zu boxes landed in it: %zu bad, %zu good", bin.count, bin.nokCount, bin.count - bin.nokCount);
        ImGui::SameLine();
        if (ImGui::SmallButton("Reset count")) {
            bin.count = 0;
            bin.nokCount = 0;
        }
    }
}

// ---------------------------------------------------------------- Control

void App::drawControl() {
    ImGui::SeparatorText("Control");
    auto& s = scene();
    // In the order of the enum.
    int control = static_cast<int>(s.control);
    ImGui::SetNextItemWidth(kFieldWidth);
    if (ImGui::Combo("program", &control,
                     "none (by hand)\0built-in sorting\0soft_plc (lockstep)\0S7 PLC (live)\0"
                     "built-in camera triggering\0")) {
        s.control = static_cast<Control>(control);
    }
    ImGui::SetItemTooltip("What drives the valves and motors, scan by scan (every 10 ms), through the I/O:\n"
                          "soft_plc runs an ST program in step with the simulation; an S7 PLC runs on its own.");
    ImGui::TextColored(kDim, "%zu boxes made, %zu bad", world_->spawned(), world_->spawnedNok());
    ImGui::TextColored(kDim, "Demos: File > New from a demo");
    if (!programError_.empty()) {
        ImGui::PushTextWrapPos(0.0F);
        ImGui::TextColored(kProblem, "%s", programError_.c_str());
        ImGui::PopTextWrapPos();
    }
    if (s.control == Control::SoftPlc || s.control == Control::S7Plc) {
        drawPlcStatus();
        return;
    }

    if (s.control == Control::BuiltInCameras) {
        drawCameraTriggering();
        return;
    }
    if (s.control != Control::BuiltInSorting) {
        return;
    }
    const auto* sorting = dynamic_cast<const BuiltInSorting*>(world_->program());
    const auto status = sorting != nullptr ? sorting->status() : std::vector<BuiltInSorting::Status>{};
    std::optional<std::size_t> remove;
    for (std::size_t i = 0; i < s.sorters.size(); ++i) {
        auto& sorter = s.sorters[i];
        ImGui::PushID(static_cast<int>(i));
        const auto label = i < status.size() ? fmt::format("{}: fired {}, tracking {}###sorter", sorter.name,
                                                           status[i].fired, status[i].waiting)
                                             : sorter.name + "###sorter";
        if (ImGui::TreeNode(label.c_str())) {
            ImGui::SetNextItemWidth(kFieldWidth);
            ImGui::InputText("name", &sorter.name);
            addressField("NOK sensor", sorter.nokInput, IoAddress::Area::Input, IoAddress::Size::Bit);
            addressField("encoder count", sorter.encoderInput, IoAddress::Area::Input, IoAddress::Size::DoubleWord);
            addressField("valve", sorter.valveOutput, IoAddress::Area::Output, IoAddress::Size::Bit);
            addressField("extended", sorter.extendedInput, IoAddress::Area::Input, IoAddress::Size::Bit);
            int distance = static_cast<int>(sorter.distance);
            ImGui::SetNextItemWidth(kFieldWidth);
            if (ImGui::DragInt("distance", &distance, 10.0F, 0, 1000000, "%d pulses")) {
                sorter.distance = distance;
            }
            ImGui::SetItemTooltip("Encoder pulses from the sensor seeing a bad box to firing the pusher");
            if (ImGui::SmallButton("Remove")) {
                remove = i;
            }
            ImGui::TreePop();
        }
        ImGui::PopID();
    }
    if (remove) {
        s.sorters.erase(s.sorters.begin() + static_cast<std::ptrdiff_t>(*remove));
    }
    // A new sorter, wired to a sensor and a pusher chosen here.
    int& sensor = newSorterSensor_;
    int& pusher = newSorterPusher_;
    if (!s.photocells.empty() && !s.pushers.empty()) {
        sensor = std::clamp(sensor, 0, static_cast<int>(s.photocells.size()) - 1);
        pusher = std::clamp(pusher, 0, static_cast<int>(s.pushers.size()) - 1);
        ImGui::SetNextItemWidth(90.0F);
        if (ImGui::BeginCombo("##sensor", s.photocells[static_cast<std::size_t>(sensor)].name.c_str())) {
            for (std::size_t i = 0; i < s.photocells.size(); ++i) {
                if (ImGui::Selectable(s.photocells[i].name.c_str(), static_cast<int>(i) == sensor)) {
                    sensor = static_cast<int>(i);
                }
            }
            ImGui::EndCombo();
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(90.0F);
        if (ImGui::BeginCombo("##pusher", s.pushers[static_cast<std::size_t>(pusher)].name.c_str())) {
            for (std::size_t i = 0; i < s.pushers.size(); ++i) {
                if (ImGui::Selectable(s.pushers[i].name.c_str(), static_cast<int>(i) == pusher)) {
                    pusher = static_cast<int>(i);
                }
            }
            ImGui::EndCombo();
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Add sorter")) {
            s.sorters.push_back(sorterFor(s, static_cast<std::size_t>(sensor), static_cast<std::size_t>(pusher)));
        }
        ImGui::SetItemTooltip("Wired to this sensor and pusher, with the distance worked out from the layout\n"
                              "and the belt's speed");
    }
}

// Every address in the scene, with what it is and its value now: the PLC's
// view of the line, like the I/O table of a PLC project.
void App::drawIoTable() {
    if (!showIo_) {
        return; // hidden (View menu)
    }
    ImGui::SetNextWindowPos(ImVec2(1040, 662), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(350, 250), ImGuiCond_FirstUseEver);
    ImGui::Begin("I/O", &showIo_);
    struct Row {
        std::string address;
        std::string what;
    };
    std::vector<Row> rows;
    const auto& s = scene();
    const auto add = [&](const std::string& address, std::string what) {
        if (!address.empty()) {
            rows.push_back(Row{address, std::move(what)});
        }
    };
    for (const auto& c : s.conveyors) {
        add(c.spec.encoderInput, c.spec.name + " encoder count");
        add(c.spec.runOutput, c.spec.name + " motor");
        if (c.spec.drive.fitted) {
            add(c.spec.drive.controlOutput, c.spec.name + " drive STW1 (control word)");
            add(c.spec.drive.setpointOutput, c.spec.name + " drive NSOLL_A (setpoint)");
            add(c.spec.drive.statusInput, c.spec.name + " drive ZSW1 (status word)");
            add(c.spec.drive.actualInput, c.spec.name + " drive NIST_A (actual speed)");
        }
    }
    for (const auto& p : s.potentiometers) {
        add(p.address, p.name + " potentiometer");
    }
    for (const auto& p : s.photocells) {
        add(p.input, p.name + (p.onlyNok ? " inspection (bad box)" : " photocell"));
    }
    const auto actuator = [&](const auto& a, const char* kind) {
        add(a.valveOutput, a.name + " " + kind + " valve");
        add(a.extendedInput, a.name + " extended");
        add(a.retractedInput, a.name + " retracted");
    };
    for (const auto& a : s.pushers)
        actuator(a, "pusher");
    for (const auto& a : s.diverters)
        actuator(a, "diverter");
    for (const auto& a : s.stoppers)
        actuator(a, "stopper");
    for (const auto& p : s.proximities) {
        add(p.input, p.name + (p.inductive ? " inductive switch" : " capacitive switch"));
    }
    for (const auto& l : s.lasers) {
        add(l.output, l.name + " laser distance");
    }
    for (const auto& c : s.curtains) {
        add(c.clearInput, c.name + " light curtain clear");
        add(c.countInput, c.name + " light curtain beams broken");
    }
    for (const auto& c : s.cameras) {
        add(c.triggerOutput, c.name + " camera trigger");
        add(c.readyInput, c.name + " camera ready");
    }
    for (const auto& b : s.buttons)
        add(b.address, b.name + (b.normallyClosed ? " button (NC)" : " button"));
    for (const auto& l : s.lamps)
        add(l.address, l.name + " lamp");
    std::sort(rows.begin(), rows.end(), [](const Row& a, const Row& b) {
        const auto pa = parseAddress(a.address);
        const auto pb = parseAddress(b.address);
        if (!pa || !pb)
            return a.address < b.address;
        return std::tie(pa->area, pa->byte, pa->bit) < std::tie(pb->area, pb->byte, pb->bit);
    });
    if (ImGui::BeginTable("io", 3, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY)) {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("address", ImGuiTableColumnFlags_WidthFixed, 70.0F);
        ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthFixed, 80.0F); // 16#047F fits
        ImGui::TableSetupColumn("what");
        ImGui::TableHeadersRow();
        for (const auto& row : rows) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(row.address.c_str());
            ImGui::TableNextColumn();
            const auto address = parseAddress(row.address);
            if (!address || !IoImage::fits(*address)) {
                ImGui::TextColored(kProblem, "invalid");
            } else if (address->size == IoAddress::Size::Bit) {
                const bool on = world_->io().bit(*address);
                ImGui::TextColored(on ? ImVec4(1.0F, 0.85F, 0.1F, 1.0F) : kDim, "%s", on ? "1" : "0");
            } else if (address->size == IoAddress::Size::Word) {
                // As TIA's watch table shows a word: hex, and as a signed INT.
                const auto word = static_cast<std::uint16_t>(world_->io().value(*address));
                ImGui::Text("16#%04X", word);
                ImGui::SetItemTooltip("%d as an INT, %u as a WORD", static_cast<std::int16_t>(word), word);
            } else {
                ImGui::Text("%u", world_->io().value(*address));
            }
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(row.what.c_str());
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

// The operator's panel: buttons to press (momentary ones while the mouse
// holds them, latching ones toggle) and lamps the program lights. Below,
// the list of them to edit.
void App::drawControlPanel() {
    if (!showControlPanel_) {
        return; // hidden (View menu)
    }
    ImGui::SetNextWindowPos(ImVec2(360, 792), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(560, 120), ImGuiCond_FirstUseEver);
    ImGui::Begin("Control panel", &showControlPanel_);
    auto& s = scene();
    for (std::size_t i = 0; i < world_->buttons().size(); ++i) {
        const auto& button = world_->buttons()[i];
        ImGui::PushID(static_cast<int>(i));
        if (i > 0) {
            ImGui::SameLine();
        }
        if (button.latching) {
            bool down = world_->pressed(i);
            if (ImGui::Checkbox(button.name.c_str(), &down)) {
                world_->press(i, down);
            }
        } else {
            ImGui::Button(button.name.c_str(), ImVec2(80.0F, 40.0F));
            world_->press(i, ImGui::IsItemActive()); // pressed while the mouse holds it
        }
        ImGui::PopID();
    }
    for (std::size_t i = 0; i < world_->potentiometers().size(); ++i) {
        ImGui::PushID(500 + static_cast<int>(i));
        int value = world_->potentiometer(i);
        ImGui::SetNextItemWidth(220.0F);
        if (ImGui::SliderInt(world_->potentiometers()[i].name.c_str(), &value, 0, kAnalogFullScale, "%d")) {
            world_->turn(i, value);
        }
        ImGui::SameLine();
        ImGui::TextColored(kDim, "%.0f %%", 100.0 * value / kAnalogFullScale);
        ImGui::PopID();
    }
    if (!world_->lamps().empty()) {
        ImGui::Spacing();
    }
    for (std::size_t i = 0; i < world_->lamps().size(); ++i) {
        ImGui::PushID(1000 + static_cast<int>(i));
        if (i > 0) {
            ImGui::SameLine();
        }
        led("##lamp", world_->lit(i));
        ImGui::SameLine();
        ImGui::TextUnformatted(world_->lamps()[i].name.c_str());
        ImGui::PopID();
    }
    if (world_->buttons().empty() && world_->lamps().empty() && world_->potentiometers().empty()) {
        ImGui::TextColored(kDim, "No buttons or lamps yet.");
    }

    if (ImGui::CollapsingHeader("Edit buttons and lamps")) {
        std::optional<std::size_t> removeButton;
        for (std::size_t i = 0; i < s.buttons.size(); ++i) {
            auto& b = s.buttons[i];
            ImGui::PushID(2000 + static_cast<int>(i));
            ImGui::SetNextItemWidth(90.0F);
            ImGui::InputText("##name", &b.name);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(70.0F);
            ImGui::InputText("##address", &b.address);
            ImGui::SameLine();
            ImGui::Checkbox("NC", &b.normallyClosed);
            ImGui::SetItemTooltip("Normally closed: the input is on while the button is NOT pressed (stop buttons)");
            ImGui::SameLine();
            ImGui::Checkbox("latching", &b.latching);
            ImGui::SameLine();
            if (ImGui::SmallButton("x")) {
                removeButton = i;
            }
            ImGui::PopID();
        }
        if (removeButton) {
            s.buttons.erase(s.buttons.begin() + static_cast<std::ptrdiff_t>(*removeButton));
        }
        std::optional<std::size_t> removeLamp;
        for (std::size_t i = 0; i < s.lamps.size(); ++i) {
            auto& l = s.lamps[i];
            ImGui::PushID(3000 + static_cast<int>(i));
            ImGui::SetNextItemWidth(90.0F);
            ImGui::InputText("##name", &l.name);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(70.0F);
            ImGui::InputText("##address", &l.address);
            ImGui::SameLine();
            ImGui::TextColored(kDim, "lamp");
            ImGui::SameLine();
            if (ImGui::SmallButton("x")) {
                removeLamp = i;
            }
            ImGui::PopID();
        }
        if (removeLamp) {
            s.lamps.erase(s.lamps.begin() + static_cast<std::ptrdiff_t>(*removeLamp));
        }
        std::optional<std::size_t> removePot;
        for (std::size_t i = 0; i < s.potentiometers.size(); ++i) {
            auto& p = s.potentiometers[i];
            ImGui::PushID(4000 + static_cast<int>(i));
            ImGui::SetNextItemWidth(90.0F);
            ImGui::InputText("##name", &p.name);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(70.0F);
            ImGui::InputText("##address", &p.address);
            ImGui::SameLine();
            ImGui::TextColored(kDim, "potentiometer (input word)");
            ImGui::SameLine();
            if (ImGui::SmallButton("x")) {
                removePot = i;
            }
            ImGui::PopID();
        }
        if (removePot) {
            s.potentiometers.erase(s.potentiometers.begin() + static_cast<std::ptrdiff_t>(*removePot));
        }
        if (ImGui::SmallButton("+ Button")) {
            s.buttons.push_back(ButtonSpec{fmt::format("S{}", s.buttons.size() + 1),
                                           freeAddress(s, IoAddress::Area::Input, IoAddress::Size::Bit), false, false});
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("+ Lamp")) {
            s.lamps.push_back(LampSpec{fmt::format("H{}", s.lamps.size() + 1),
                                       freeAddress(s, IoAddress::Area::Output, IoAddress::Size::Bit)});
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("+ Potentiometer")) {
            s.potentiometers.push_back(
                PotentiometerSpec{fmt::format("R{}", s.potentiometers.size() + 1),
                                  freeAddress(s, IoAddress::Area::Input, IoAddress::Size::Word)});
        }
        ImGui::SetItemTooltip("An analog input word, 0 to 27648, like a Siemens analog input module");
    }
    ImGui::End();
}

// ---------------------------------------------------------------- PLC

void App::drawPlcStatus() {
    auto& s = scene();
    if (s.control == Control::SoftPlc) {
        ImGui::SetNextItemWidth(kFieldWidth);
        ImGui::InputTextWithHint("ST file", "conveyor.st", &s.plcProgram);
        ImGui::SetItemTooltip("The Structured Text soft_plc runs (ladder_editor's Export ST writes one).\n"
                              "A relative path is relative to the scene file's folder.");
        ImGui::SetNextItemWidth(kFieldWidth);
        ImGui::InputInt("monitor port", &s.monitorPort, 0);
        s.monitorPort = std::clamp(s.monitorPort, 0, 65535);
        ImGui::SetItemTooltip("0: none. Otherwise ladder_editor --connect=PORT (or soft_plc_monitor) goes online\n"
                              "with the program running here: values, forcing, Download.");
        if (ImGui::SmallButton("Reload the file")) {
            replaceProgram(); // a fresh start of the program; the line keeps running
        }
        ImGui::SetItemTooltip("Read the ST file again and start it afresh (its values start over).\n"
                              "To keep them, Download from ladder_editor instead: that's an online change.");
        if (const auto* plc = dynamic_cast<const SoftPlcProgram*>(world_->program())) {
            const auto status = plc->status();
            ImGui::TextColored(status.running ? ImVec4(0.45F, 0.85F, 0.50F, 1.0F) : kProblem, "%s",
                               status.running ? "RUN" : "STOP");
            ImGui::SameLine();
            ImGui::TextColored(kDim, "%llu cycles, last %.0f µs", static_cast<unsigned long long>(status.cycles),
                               status.lastCycleMicroseconds);
            if (!status.error.empty()) {
                ImGui::PushTextWrapPos(0.0F);
                ImGui::TextColored(kProblem, "%s", status.error.c_str());
                ImGui::PopTextWrapPos();
            }
            if (status.monitorPort != 0) {
                ImGui::TextColored(kDim, "online: ladder_editor --connect=%u", status.monitorPort);
            }
        }
    } else {
        ImGui::SetNextItemWidth(kFieldWidth);
        ImGui::InputText("host", &s.s7Host);
        ImGui::SetNextItemWidth(kFieldWidth);
        ImGui::InputInt("S7 port", &s.s7Port, 0);
        s.s7Port = std::clamp(s.s7Port, 1, 65535);
        ImGui::SetItemTooltip("102 for a real S7; soft_plc --s7=PORT for another (no rights needed above 1023)");
        ImGui::SetNextItemWidth(kFieldWidth);
        ImGui::SliderInt("bytes", &s.s7Bytes, 2, static_cast<int>(IoImage::kBytes));
        ImGui::SetItemTooltip("How many input and output bytes to exchange each scan. soft_plc --s7 has 16;\n"
                              "a real S7 has far more. Addresses past them don't reach the PLC.");
        if (const auto* link = dynamic_cast<const S7Link*>(world_->program())) {
            const auto status = link->status();
            ImGui::TextColored(status.connected ? ImVec4(0.45F, 0.85F, 0.50F, 1.0F) : kProblem, "%s",
                               status.connected ? "connected" : "not connected");
            ImGui::SameLine();
            ImGui::TextColored(kDim, "%llu exchanges, last %.0f µs", static_cast<unsigned long long>(status.exchanges),
                               status.lastRoundTripMicroseconds);
            if (!status.connected && !status.error.empty()) {
                ImGui::TextColored(kProblem, "%s", status.error.c_str());
            }
        }
        if (timeScale_ != 1.0F || paused_) {
            ImGui::TextColored(kProblem, "A live PLC runs in real time: set the time scale to 1x.");
        }
    }
}

// soft_plc's variables, live: everything the program has, as it has it.
void App::drawPlcVariables() {
    if (!showPlcVariables_) {
        return; // hidden (View menu)
    }
    const auto* plc = dynamic_cast<const SoftPlcProgram*>(world_->program());
    if (plc == nullptr) {
        return;
    }
    ImGui::SetNextWindowPos(ImVec2(360, 32), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(330, 300), ImGuiCond_FirstUseEver);
    ImGui::Begin("PLC variables", &showPlcVariables_);
    ImGui::SetNextItemWidth(-1.0F);
    ImGui::InputTextWithHint("##filter", "filter: part of a name", &variableFilter_);
    if (ImGui::BeginTable("variables", 3, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY)) {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("variable");
        ImGui::TableSetupColumn("type", ImGuiTableColumnFlags_WidthFixed, 45.0F);
        ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthFixed, 80.0F);
        ImGui::TableHeadersRow();
        for (const auto& variable : plc->variables()) {
            if (!variableFilter_.empty() && variable.path.find(variableFilter_) == std::string::npos) {
                continue;
            }
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(variable.path.c_str());
            ImGui::TableNextColumn();
            ImGui::TextColored(kDim, "%s", variable.type.c_str());
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(variable.value.c_str());
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

} // namespace fsim
