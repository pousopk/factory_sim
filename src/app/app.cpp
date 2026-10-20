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
    : settings_(loadSettings(configFolder() / "settings.json")), history_(Scene{}) {
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
    } else {
        startPage_ = true; // nothing named: choose what to do
        untitledLeft_ = untitledWork(untitledFile()).has_value();
    }
    if (!world_) {
        rebuildWorld();
    }
}

void App::rebuildWorld() {
    world_.reset();
    programError_.clear();
    // STL files named without a folder: next to the scene, or in the examples.
    setShapeFolders({sceneFolder(), FACTORY_SIM_EXAMPLES_DIR});
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
    if (startPage_) {
        // No line yet: nothing to run, draw or save, only what to do.
        drawMenuBar();
        drawStartPage();
        drawHelp();
        drawFileDialog();
        drawNewConfirmation();
        return;
    }
    handleMouse(width, height);
    handleKeys();
    drawMenuBar();
    drawToolbar();
    drawStatusBar();
    drawDockspace();
    drawControlWindow();
    drawHelp();
    drawLinePanel();
    drawInspector();
    drawTriggers();
    drawIoTable();
    drawControlPanel();
    drawPlcVariables();
    drawAnalyser();
    drawCameras();
    drawCameraView();
    drawFileDialog();
    drawNewConfirmation();

    // An edit is finished when no widget is being used: then it's one undo step, saved at once.
    if (!ImGui::IsAnyItemActive() && !drag_ && history_.commit()) {
        if (auto saved = saveScene(scene(), savePath()); saved) {
            fileStatus_ = scenePath_.empty() ? "untitled (saved in the config folder)"
                                             : "saved " + scenePath_.filename().string();
        } else {
            fileStatus_ = saved.error();
        }
    }
    keepSelectionValid();
    if (!drag_) {
        followScene(); // (while dragging, a preview is drawn instead)
    }

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
    if (realSeconds > 0.0) {
        const double factor = stepsLastFrame_ * World::kStep / realSeconds;
        realTimeFactor_ = 0.9 * realTimeFactor_ + 0.1 * factor; // smoothed over about ten frames
    }
    finishFiring();
    // The 3D view fills the dockspace's middle: in framebuffer pixels, with
    // OpenGL's y going up from the bottom.
    const auto& io = ImGui::GetIO();
    const float scaleX = static_cast<float>(width) / std::max(io.DisplaySize.x, 1.0F);
    const float scaleY = static_cast<float>(height) / std::max(io.DisplaySize.y, 1.0F);
    const int viewWidth = std::max(1, static_cast<int>(viewSize_.x * scaleX));
    const int viewHeight = std::max(1, static_cast<int>(viewSize_.y * scaleY));
    renderer_.draw(boxesToDraw(), camera_, static_cast<int>(viewMin_.x * scaleX),
                   height - static_cast<int>((viewMin_.y + viewSize_.y) * scaleY), viewWidth, viewHeight);
    if (showNames_) {
        drawNames();
    }
}

// Each device's name, at its place in the view: projected through the
// camera like a vertex, then written by ImGui behind its windows.
void App::drawNames() {
    const float aspect = viewSize_.y > 0.0F ? viewSize_.x / viewSize_.y : 1.0F;
    const auto viewProjection = camera_.projection(aspect) * camera_.view();
    auto* draw = ImGui::GetBackgroundDrawList();
    draw->PushClipRect(viewMin_, ImVec2(viewMin_.x + viewSize_.x, viewMin_.y + viewSize_.y));
    for (const auto& label : sceneLabels(drag_ && preview_ ? *preview_ : *world_)) {
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
        const ImVec2 at(viewMin_.x + (ndc.x * 0.5F + 0.5F) * viewSize_.x - size.x / 2.0F,
                        viewMin_.y + (0.5F - ndc.y * 0.5F) * viewSize_.y - size.y);
        const bool selected = selected_ == label.pick;
        draw->AddRectFilled(ImVec2(at.x - 3.0F, at.y - 1.0F), ImVec2(at.x + size.x + 3.0F, at.y + size.y + 1.0F),
                            selected ? IM_COL32(40, 90, 170, 200) : IM_COL32(0, 0, 0, 120), 3.0F);
        draw->AddText(at, IM_COL32(235, 238, 245, 255), label.text.c_str());
    }
    draw->PopClipRect();
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
        hovered_.reset();
        hoverPick_.reset();
        return;
    }
    const bool clicked =
        ImGui::IsMouseReleased(ImGuiMouseButton_Left) && io.MouseDragMaxDistanceSqr[ImGuiMouseButton_Left] < 9.0F;
    if (const auto ray = mouseRay(width, height)) {
        editInView(*ray, ImGui::IsMouseClicked(ImGuiMouseButton_Left), ImGui::IsMouseDown(ImGuiMouseButton_Left),
                   clicked);
        // What's under the mouse: lighter, with a tooltip saying what it is.
        // A click (a press and release that didn't move far) selects it.
        hoverPick_.reset();
        if (!placing_ && !drag_) {
            hoverPick_ = pickAt(sceneBoxes(*world_), ray->origin, ray->direction);
            if (hoverPick_ && !ImGui::IsMouseDown(ImGuiMouseButton_Left) &&
                !ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
                if (const auto text = describe(*hoverPick_); !text.empty()) {
                    ImGui::SetTooltip("%s", text.c_str());
                }
            }
            if (clicked) {
                selected_ = hoverPick_;
            }
        }
    }
    const auto windowHeight = static_cast<float>(height);
    if (!drag_ && ImGui::IsMouseDragging(ImGuiMouseButton_Left, 3.0F)) {
        camera_.rotate(io.MouseDelta.x, io.MouseDelta.y, windowHeight);
    }
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Right, 0.0F) || ImGui::IsMouseDragging(ImGuiMouseButton_Middle, 0.0F)) {
        camera_.pan(io.MouseDelta.x, io.MouseDelta.y, windowHeight);
    }
    if (io.MouseWheel != 0.0F) {
        camera_.zoom(io.MouseWheel);
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
        if (placing_) {
            placing_.reset(); // Esc first cancels placing, then the selection
            hovered_.reset();
        } else {
            selected_.reset();
        }
    }
    if (io.WantCaptureKeyboard) {
        return;
    }
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_C)) {
        copySelection();
    }
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_V)) {
        pasteClip(std::nullopt);
    }
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_D)) {
        copySelection();
        pasteClip(std::nullopt);
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Space, false)) {
        paused_ = !paused_;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_F1, false)) {
        showHelp_ = !showHelp_;
    }
    if (!io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_R)) {

        turnSelection(io.KeyShift ? -1.0F : 1.0F);
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
    case Pick::Kind::Item:
        world_->removeItem(selected_->index); // not the scene's: just this one off the line
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
    case Pick::Kind::Item: {
        // An item lives in the world, not the scene: still there?
        const auto items = world_->items();
        if (std::none_of(items.begin(), items.end(), [&](const ItemView& i) { return i.id == selected_->index; })) {
            selected_.reset();
        }
        return;
    }
    case Pick::Kind::None:
        break;
    }
    if (selected_->index >= count) {
        selected_.reset();
    }
}

// ---------------------------------------------------------------- Panels

// Everything in the line, to select, and buttons to add and remove things.
void App::drawLinePanel() {
    if (!showLine_) {
        return; // hidden (View menu)
    }
    ImGui::SetNextWindowPos(ImVec2(10, 472), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(340, 330), ImGuiCond_FirstUseEver);
    ImGui::Begin("Line", &showLine_);
    auto& s = scene();
    drawLineTree();

    ImGui::SeparatorText("Add");
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
    ImGui::SameLine();
    ImGui::TextUnformatted(" Join it:");
    for (const auto side : {Side::Left, Side::Right}) {
        ImGui::SameLine();
        if (ImGui::SmallButton(side == Side::Left ? "from the left" : "from the right")) {
            selected_ = Pick{Pick::Kind::Conveyor, addConveyorJoining(s, selected_->index, side)};
        }
    }
    ImGui::SetItemTooltip("A conveyor merging into the selected one from the side, halfway along it");
    ImGui::EndDisabled();
    if (ImGui::SmallButton("New conveyor")) {
        selected_ = Pick{Pick::Kind::Conveyor, addConveyor(s)};
    }
    // Devices: the button picks what, a click on a belt says where.
    ImGui::TextUnformatted("Then click on a belt to place:");
    const struct {
        const char* label;
        Pick::Kind kind;
    } devices[] = {{"Photocell", Pick::Kind::Photocell}, {"Spawner", Pick::Kind::Spawner},
                   {"Pusher", Pick::Kind::Pusher},       {"Diverter", Pick::Kind::Diverter},
                   {"Stopper", Pick::Kind::Stopper},     {"Bin", Pick::Kind::Bin},
                   {"Camera", Pick::Kind::Camera},       {"Proximity", Pick::Kind::Proximity},
                   {"Laser", Pick::Kind::Laser},         {"Light curtain", Pick::Kind::LightCurtain}};
    for (std::size_t i = 0; i < std::size(devices); ++i) {
        if (i % 4 != 0) {
            ImGui::SameLine();
        }
        const bool armed = placing_ == devices[i].kind;
        if (armed) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.25F, 0.55F, 0.95F, 1.0F));
        }
        if (ImGui::SmallButton(devices[i].label)) {
            placing_ = armed ? std::nullopt : std::optional<Pick::Kind>(devices[i].kind);
        }
        if (armed) {
            ImGui::PopStyleColor();
        }
    }
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
        case Pick::Kind::Item:
            drawItemInspector(selected_->index);
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
    ImGui::SetNextItemWidth(fieldWidth());
    ImGui::InputText("name", &spec.name);

    ImGui::SeparatorText("Place");
    if (placed.follows && *placed.follows < s.conveyors.size()) {
        ImGui::Text("After %s, %s", s.conveyors[*placed.follows].spec.name.c_str(), turnName(placed.turn));
        int turn = static_cast<int>(placed.turn);
        ImGui::SetNextItemWidth(fieldWidth());
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
    } else if (placed.joins && *placed.joins < s.conveyors.size()) {
        ImGui::Text("Joins %s from the %s", s.conveyors[*placed.joins].spec.name.c_str(),
                    placed.joinSide == Side::Left ? "left" : "right");
        ImGui::SetNextItemWidth(fieldWidth());
        ImGui::SliderFloat("at", &placed.joinAt, 0.0F, s.conveyors[*placed.joins].spec.length, "%.2f m along it");
        if (ImGui::SmallButton("Detach")) {
            const auto where = placeConveyors(s)[index];
            spec.start = where.start;
            spec.heading = where.heading;
            spec.beltHeight = where.beltHeight;
            placed.joins.reset();
        }
        ImGui::SetItemTooltip("Keep it where it is, but no longer attached to the conveyor it joins");
    } else {
        ImGui::SetNextItemWidth(fieldWidth());
        ImGui::DragFloat("start x", &spec.start.x, 0.01F, -50.0F, 50.0F, "%.3f m");
        ImGui::SetNextItemWidth(fieldWidth());
        ImGui::DragFloat("start z", &spec.start.z, 0.01F, -50.0F, 50.0F, "%.3f m");
        float degrees = spec.heading * kDegrees;
        ImGui::SetNextItemWidth(fieldWidth());
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
    ImGui::SetNextItemWidth(fieldWidth());
    ImGui::SliderFloat("length", &spec.length, 0.5F, 10.0F, "%.2f m");
    ImGui::SetNextItemWidth(fieldWidth());
    ImGui::SliderFloat("width", &spec.width, 0.15F, 1.2F, "%.2f m");
    ImGui::SetNextItemWidth(fieldWidth());
    ImGui::SliderFloat("belt height", &spec.beltHeight, 0.3F, 1.5F, "%.2f m");
    ImGui::TextColored(kDim, "Changing the place or size restarts the run.");

    ImGui::SeparatorText("Speed");
    if (spec.drive.fitted) {
        ImGui::TextColored(kDim, "With a drive, the PLC sets the speed: the target speed is only\n"
                                 "used without a control program, and the run output not at all.");
    }
    ImGui::SetNextItemWidth(fieldWidth());
    ImGui::SliderFloat("target speed", &spec.targetSpeed, 0.0F, 3.0F, "%.2f m/s");
    ImGui::SetNextItemWidth(fieldWidth());
    ImGui::SliderFloat("acceleration", &spec.acceleration, 0.1F, 10.0F, "%.1f m/s²", ImGuiSliderFlags_Logarithmic);
    ImGui::SetItemTooltip("Like a VFD's ramp: 1 m/s² takes 1 s from 0 to 1 m/s.");
    if (index < world_->conveyors().size()) {
        const auto& running = world_->conveyors()[index];
        const float fraction = spec.targetSpeed > 0.0F ? running.speed / spec.targetSpeed : 0.0F;
        ImGui::ProgressBar(std::clamp(fraction, 0.0F, 1.0F), ImVec2(fieldWidth(), 0.0F), "");
        ImGui::SameLine();
        ImGui::Text("%.3f m/s", running.speed);
        ImGui::TextColored(kDim, "at this speed a box moves %.2f mm per 1 ms step", running.speed); // m/s = mm/ms
    }

    ImGui::SeparatorText("I/O");
    addressField("run output", spec.runOutput, IoAddress::Area::Output, IoAddress::Size::Bit);
    ImGui::SetItemTooltip(
        "The motor runs while this output is on (only with a control program).\nEmpty: it always runs.");
    addressField("encoder count", spec.encoderInput, IoAddress::Area::Input, IoAddress::Size::DoubleWord);

    drawConveyorEquipment(index);

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
    ImGui::SetNextItemWidth(fieldWidth());
    ImGui::SliderInt("pulses/rev", &spec.encoder.pulsesPerRevolution, 100, 5000);
    ImGui::SetNextItemWidth(fieldWidth());
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
    ImGui::SetNextItemWidth(fieldWidth());
    ImGui::InputText("name", &spec.name);
    addressField("input", spec.input, IoAddress::Area::Input, IoAddress::Size::Bit);
    ImGui::Checkbox("sees only bad (NOK) boxes", &spec.onlyNok);
    ImGui::SetItemTooltip("A stand-in for an inspection system (a camera, a colour sensor)\n"
                          "whose output says \"this one is bad\".");
    conveyorCombo("on conveyor", spec.conveyor, s);
    const float length = spec.conveyor < s.conveyors.size() ? s.conveyors[spec.conveyor].spec.length : 1.0F;
    ImGui::SetNextItemWidth(fieldWidth());
    ImGui::SliderFloat("position", &spec.along, 0.0F, length, "%.3f m");
    ImGui::SetNextItemWidth(fieldWidth());
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
    ImGui::Checkbox("spawn items", &spawner.enabled);
    scaledSlider("bad ones", spawner.nokFraction, 100.0F, 0.0F, 100.0F, "%.0f %%");
    ImGui::SetItemTooltip("The share of items that are bad (NOK, red), chosen at random");
    scaledSlider("every", spawner.interval, 1.0, 0.2F, 8.0F, "%.2f s");
    scaledSlider("jitter", spawner.intervalJitter, 1.0, 0.0F, 1.0F, "± %.2f s");
    drawShapeSettings(spawner);
    ImGui::Checkbox("arriving at belt speed", &spawner.atBeltSpeed);
    ImGui::SetItemTooltip("As from an infeed conveyor. Off: put down standing still, so the belt has to\n"
                          "speed them up first, and the spawner waits until there's room.");
    const float length = spawner.conveyor < s.conveyors.size() ? s.conveyors[spawner.conveyor].spec.length : 1.0F;
    ImGui::SetNextItemWidth(fieldWidth());
    ImGui::SliderFloat("where", &spawner.offset, 0.0F, length, "%.2f m from the start");
    const auto geometry = itemGeometry(spawner.shape, spawner.size, spawner.stl);
    const auto size = geometry && *geometry != nullptr ? (*geometry)->size() : spawner.size;
    if (spawner.conveyor < s.conveyors.size()) {
        const float speed = s.conveyors[spawner.conveyor].spec.targetSpeed;
        ImGui::TextColored(kDim, "gap between them at full speed: %.3f m",
                           speed * static_cast<float>(spawner.interval) - size.x);
    }
    ImGui::BeginDisabled(!geometry);
    if (ImGui::Button("Spawn one now") && spawner.conveyor < world_->conveyors().size()) {
        const auto& c = world_->conveyors()[spawner.conveyor];
        world_->spawnItem(c.beltPoint(spawner.offset) + glm::vec3(0.0F, size.y / 2.0F + 0.002F, 0.0F), size,
                          world_->itemColor(spawner.shape), c.spec.heading, false, spawner.shape,
                          spawner.atBeltSpeed ? c.direction() * c.speed : glm::vec3(0.0F), geometry.value_or(nullptr));
    }
    ImGui::EndDisabled();
}

// What a spawner makes: its shape and size, or its STL file, and what
// that comes to (its weight, its material, the physics' pieces).
void App::drawShapeSettings(SpawnerSpec& spawner) {
    static const char* const names[] = {"boxes", "tins",           "bottles",     "bags of crisps",
                                        "balls", "angle brackets", "frozen fish", "a part from an STL file"};
    int shape = static_cast<int>(spawner.shape);
    ImGui::SetNextItemWidth(fieldWidth());
    if (ImGui::Combo("makes", &shape, names, IM_ARRAYSIZE(names))) {
        spawner.shape = static_cast<Shape>(shape);
        if (spawner.shape != Shape::Stl) {
            spawner.size = typicalSize(spawner.shape);
        }
    }
    ImGui::SetNextItemWidth(fieldWidth());
    switch (spawner.shape) {
    case Shape::Tin:
    case Shape::Bottle: {
        float size[2] = {spawner.size.x, spawner.size.y};
        if (ImGui::SliderFloat2("size", size, 0.02F, 0.4F, "%.3f m")) {
            spawner.size = {size[0], size[1], size[0]};
        }
        ImGui::SetItemTooltip("Diameter, height");
        break;
    }
    case Shape::Ball:
        if (ImGui::SliderFloat("diameter", &spawner.size.x, 0.02F, 0.3F, "%.3f m")) {
            spawner.size = glm::vec3(spawner.size.x);
        }
        break;
    case Shape::Stl:
        drawStlSettings(spawner);
        break;
    default:
        ImGui::SliderFloat3("size", &spawner.size.x, 0.02F, 0.6F, "%.3f m");
        ImGui::SetItemTooltip(spawner.shape == Shape::Fish ? "Length, thickness (lying on its side), depth"
                                                           : "Length along the belt, height, width");
        break;
    }
    if (spawner.shape != Shape::Stl) {
        scaledSlider("height jitter", spawner.heightJitter, 1000.0F, 0.0F, 50.0F, "± %.0f mm");
        ImGui::SetItemTooltip("Each item's height varies at random by up to this much");
    }
    const auto geometry = itemGeometry(spawner.shape, spawner.size, spawner.stl);
    if (!geometry) {
        ImGui::TextColored(kProblem, "%s", geometry.error().c_str());
        return;
    }
    if (*geometry == nullptr) {
        ImGui::TextColored(kDim, "%s", spawner.shape == Shape::Tin ? "tinplate" : "cardboard");
        return;
    }
    const auto& g = **geometry;
    const auto& material = propertiesOf(g.material());
    ImGui::TextColored(kDim, "%.3f kg of %s, %.0f x %.0f x %.0f mm", g.mass(), material.name, g.size().x * 1000.0F,
                       g.size().y * 1000.0F, g.size().z * 1000.0F);
    ImGui::SetItemTooltip("Friction %.2f, bounce %.2f, %.0f kg/m³%s\n"
                          "Its centre of mass is %.0f mm %s of its middle.",
                          material.friction, material.restitution, material.density, material.metal ? ", metal" : "",
                          std::abs(g.centerOfMass().x) * 1000.0F, g.centerOfMass().x >= 0.0F ? "ahead" : "behind");
    ImGui::TextColored(kDim, "%zu convex piece%s for the physics, %zu triangles drawn",
                       std::max<std::size_t>(g.pieceCount(), 1), g.pieceCount() == 1 ? "" : "s",
                       g.surface().positions.size() / 3);
}

void App::drawStlSettings(SpawnerSpec& spawner) {
    auto& stl = spawner.stl;
    ImGui::SetNextItemWidth(fieldWidth() - ImGui::GetFontSize() * 4.5F);
    ImGui::InputText("##file", &stl.file);
    ImGui::SameLine();
    if (ImGui::Button("Choose...")) {
        dialog_ = Dialog::Stl;
        dialogSpawner_ = static_cast<std::size_t>(&spawner - scene().spawners.data());
        const auto found = findShapeFile(stl.file);
        browser_.emplace(found.empty() ? sceneFolder() : found.parent_path(), ".stl");
        dialogName_ = found.empty() ? std::string() : found.filename().string();
        dialogError_.clear();
    }
    ImGui::SameLine();
    ImGui::TextUnformatted("file");
    ImGui::SetItemTooltip("Looked for next to the scene, then in the examples folder, unless it's a full path");
    // STL files have no unit: the one it was drawn in.
    const std::pair<const char*, float> units[] = {{"mm", 0.001F}, {"cm", 0.01F}, {"inch", 0.0254F}, {"m", 1.0F}};
    int unit = 0;
    for (int i = 0; i < 4; ++i) {
        if (std::abs(units[i].second - stl.unit) < 1e-6F) {
            unit = i;
        }
    }
    ImGui::SetNextItemWidth(fieldWidth());
    if (ImGui::Combo("drawn in", &unit, "mm\0cm\0inch\0m\0")) {
        stl.unit = units[unit].second;
    }
    ImGui::SameLine();
    ImGui::Checkbox("z up", &stl.zUp);
    ImGui::SetItemTooltip("Most CAD programs have z up; here y is. Its x runs along the belt.");
    ImGui::SetNextItemWidth(fieldWidth());
    if (ImGui::BeginCombo("made of", propertiesOf(stl.material).name)) {
        for (const auto m : kMaterials) {
            if (ImGui::Selectable(propertiesOf(m).name, m == stl.material)) {
                stl.material = m;
            }
        }
        ImGui::EndCombo();
    }
    ImGui::SetNextItemWidth(fieldWidth());
    ImGui::SliderInt("pieces", &stl.pieces, 1, 32, "up to %d");
    ImGui::SetItemTooltip("How many convex pieces the physics may split it into (with V-HACD).\n"
                          "1: its convex hull, which fills any hollow; more follow its shape better\n"
                          "and cost more to simulate.");
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
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10.8F);
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
    ImGui::SetNextItemWidth(fieldWidth());
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
    ImGui::ProgressBar(static_cast<float>(stroke.position()), ImVec2(fieldWidth(), 0.0F), "");
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
    ImGui::SetNextItemWidth(fieldWidth());
    ImGui::InputText("name", &spec.name);
    conveyorCombo("on conveyor", spec.conveyor, s);
    const float length = spec.conveyor < s.conveyors.size() ? s.conveyors[spec.conveyor].spec.length : 1.0F;
    ImGui::SetNextItemWidth(fieldWidth());
    ImGui::SliderFloat("position", &spec.along, 0.0F, length, "%.3f m");
    sideCombo(spec.side);
    ImGui::SetNextItemWidth(fieldWidth());
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
    ImGui::SetNextItemWidth(fieldWidth());
    ImGui::InputText("name", &spec.name);
    conveyorCombo("on conveyor", spec.conveyor, s);
    const float length = spec.conveyor < s.conveyors.size() ? s.conveyors[spec.conveyor].spec.length : 1.0F;
    ImGui::SetNextItemWidth(fieldWidth());
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
    ImGui::SetNextItemWidth(fieldWidth());
    ImGui::InputText("name", &spec.name);
    conveyorCombo("on conveyor", spec.conveyor, s);
    const float length = spec.conveyor < s.conveyors.size() ? s.conveyors[spec.conveyor].spec.length : 1.0F;
    ImGui::SetNextItemWidth(fieldWidth());
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
    ImGui::SetNextItemWidth(fieldWidth());
    ImGui::InputText("name", &spec.name);
    conveyorCombo("beside conveyor", spec.conveyor, s);
    const float length = spec.conveyor < s.conveyors.size() ? s.conveyors[spec.conveyor].spec.length : 1.0F;
    ImGui::SetNextItemWidth(fieldWidth());
    ImGui::SliderFloat("position", &spec.along, 0.0F, length, "%.3f m");
    sideCombo(spec.side);
    ImGui::SetNextItemWidth(fieldWidth());
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
    auto& s = scene();
    // In the order of the enum.
    int control = static_cast<int>(s.control);
    ImGui::SetNextItemWidth(fieldWidth());
    if (ImGui::Combo("program", &control,
                     "none (by hand)\0built-in sorting\0soft_plc (lockstep)\0S7 PLC (live)\0"
                     "built-in camera triggering\0")) {
        s.control = static_cast<Control>(control);
    }
    ImGui::SetItemTooltip("What drives the valves and motors, scan by scan (every 10 ms), through the I/O:\n"
                          "soft_plc runs an ST program in step with the simulation; an S7 PLC runs on its own.");
    ImGui::TextColored(kDim, "%zu items made, %zu bad", world_->spawned(), world_->spawnedNok());
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
            ImGui::SetNextItemWidth(fieldWidth());
            ImGui::InputText("name", &sorter.name);
            addressField("NOK sensor", sorter.nokInput, IoAddress::Area::Input, IoAddress::Size::Bit);
            addressField("encoder count", sorter.encoderInput, IoAddress::Area::Input, IoAddress::Size::DoubleWord);
            addressField("valve", sorter.valveOutput, IoAddress::Area::Output, IoAddress::Size::Bit);
            addressField("extended", sorter.extendedInput, IoAddress::Area::Input, IoAddress::Size::Bit);
            int distance = static_cast<int>(sorter.distance);
            ImGui::SetNextItemWidth(fieldWidth());
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
        ImGui::SetNextItemWidth(ImGui::GetFontSize() * 6.9F);
        if (ImGui::BeginCombo("##sensor", s.photocells[static_cast<std::size_t>(sensor)].name.c_str())) {
            for (std::size_t i = 0; i < s.photocells.size(); ++i) {
                if (ImGui::Selectable(s.photocells[i].name.c_str(), static_cast<int>(i) == sensor)) {
                    sensor = static_cast<int>(i);
                }
            }
            ImGui::EndCombo();
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(ImGui::GetFontSize() * 6.9F);
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
        ImGui::TableSetupColumn("address", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFontSize() * 5.4F);
        ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFontSize() * 6.2F); // 16#047F fits
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
    ImGui::Begin("Operator panel", &showControlPanel_);
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
        ImGui::SetNextItemWidth(ImGui::GetFontSize() * 16.9F);
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
            ImGui::SetNextItemWidth(ImGui::GetFontSize() * 6.9F);
            ImGui::InputText("##name", &b.name);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(ImGui::GetFontSize() * 5.4F);
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
            ImGui::SetNextItemWidth(ImGui::GetFontSize() * 6.9F);
            ImGui::InputText("##name", &l.name);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(ImGui::GetFontSize() * 5.4F);
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
            ImGui::SetNextItemWidth(ImGui::GetFontSize() * 6.9F);
            ImGui::InputText("##name", &p.name);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(ImGui::GetFontSize() * 5.4F);
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
        ImGui::SetNextItemWidth(fieldWidth());
        ImGui::InputTextWithHint("ST file", "conveyor.st", &s.plcProgram);
        ImGui::SetItemTooltip("The Structured Text soft_plc runs (ladder_editor's Export ST writes one).\n"
                              "A relative path is relative to the scene file's folder.");
        ImGui::SetNextItemWidth(fieldWidth());
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
        ImGui::SetNextItemWidth(fieldWidth());
        ImGui::InputText("host", &s.s7Host);
        ImGui::SetNextItemWidth(fieldWidth());
        ImGui::InputInt("S7 port", &s.s7Port, 0);
        s.s7Port = std::clamp(s.s7Port, 1, 65535);
        ImGui::SetItemTooltip("102 for a real S7; soft_plc --s7=PORT for another (no rights needed above 1023)");
        ImGui::SetNextItemWidth(fieldWidth());
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
        ImGui::TableSetupColumn("type", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFontSize() * 3.5F);
        ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFontSize() * 6.2F);
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
