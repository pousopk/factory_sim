#include "app/app.hpp"
#include <algorithm>
#include <cmath>
#include <fmt/format.h>
#include <imgui.h>
#include <imgui_stdlib.h>
#include <numbers>

namespace fsim {

namespace {

const ImVec4 kDim{0.65F, 0.67F, 0.72F, 1.0F};
const ImVec4 kProblem{0.95F, 0.45F, 0.40F, 1.0F};
constexpr float kFieldWidth = 180.0F;
constexpr float kDegrees = 180.0F / std::numbers::pi_v<float>;

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

} // namespace

App::App(std::filesystem::path scenePath) : scenePath_(std::move(scenePath)), history_(defaultScene()) {
    if (std::filesystem::exists(scenePath_)) {
        if (auto loaded = loadScene(scenePath_)) {
            history_ = SceneHistory(std::move(*loaded));
            fileStatus_ = "loaded " + scenePath_.filename().string();
        } else {
            // Keep the unreadable file aside rather than overwrite it with the next save.
            auto aside = scenePath_;
            aside += ".broken";
            std::error_code ignored;
            std::filesystem::copy_file(scenePath_, aside, std::filesystem::copy_options::overwrite_existing, ignored);
            fileStatus_ = loaded.error() + " (kept as " + aside.filename().string() + "; starting a new scene)";
        }
    }
    applied_ = scene();
    world_ = buildWorld(applied_);
}

void App::frame(double realSeconds, int width, int height) {
    handleMouse(width, height);
    handleKeys();
    drawMainPanel();
    drawLinePanel();
    drawInspector();
    drawTriggers();

    // An edit is finished when no widget is being used: then it's one undo step, saved at once.
    if (!ImGui::IsAnyItemActive() && history_.commit()) {
        if (auto saved = saveScene(scene(), scenePath_); saved) {
            fileStatus_ = "saved " + scenePath_.filename().string();
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
    stepsLastFrame_ = paused_ ? 0 : world_->advance(realSeconds * timeScale_);
    finishFiring();
    renderer_.draw(sceneBoxes(*world_, 1.0, selected_), camera_, width, height);
}

void App::followScene() {
    if (scene() == applied_) {
        return;
    }
    if (sameLayout(applied_, scene())) {
        applyLive(*world_, scene()); // the boxes keep running
    } else {
        world_ = buildWorld(scene()); // something moved: the run starts again
    }
    applied_ = scene();
}

void App::restartRun() {
    world_ = buildWorld(scene());
    applied_ = scene();
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
    case Pick::Kind::None:
        break;
    }
    return std::nullopt;
}

// ---------------------------------------------------------------- Panels

void App::drawMainPanel() {
    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
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
                       "%s", fileStatus_.empty() ? scenePath_.string().c_str() : fileStatus_.c_str());
    ImGui::PopTextWrapPos();

    ImGui::SeparatorText("Camera");
    ImGui::TextColored(kDim,
                       "Left drag: turn. Right drag: move. Wheel: zoom.\nClick: select. Del: delete. Esc: deselect.");
    ImGui::End();
}

// Everything in the line, to select, and buttons to add and remove things.
void App::drawLinePanel() {
    ImGui::SetNextWindowPos(ImVec2(10, 420), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(340, 330), ImGuiCond_FirstUseEver);
    ImGui::Begin("Line");
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
    ImGui::SeparatorText("Sensors and spawners");
    for (std::size_t i = 0; i < s.photocells.size(); ++i) {
        const auto& p = s.photocells[i];
        const auto on = p.conveyor < s.conveyors.size() ? s.conveyors[p.conveyor].spec.name : std::string("?");
        selectable(fmt::format("{}  photocell on {} at {:.2f} m###p{}", p.name, on, p.along, i),
                   Pick{Pick::Kind::Photocell, i});
    }
    for (std::size_t i = 0; i < s.spawners.size(); ++i) {
        const auto& sp = s.spawners[i];
        const auto on = sp.conveyor < s.conveyors.size() ? s.conveyors[sp.conveyor].spec.name : std::string("?");
        selectable(fmt::format("spawner on {}, every {:.2f} s###s{}", on, sp.interval, i),
                   Pick{Pick::Kind::Spawner, i});
    }
    const auto conveyorName = [&](std::size_t c) {
        return c < s.conveyors.size() ? s.conveyors[c].spec.name : std::string("?");
    };
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
    ImGui::SetNextWindowPos(ImVec2(1010, 380), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(380, 0), ImGuiCond_FirstUseEver);
    ImGui::Begin("Inspector");
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

    ImGui::SeparatorText("Drive");
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
    scaledSlider("every", spawner.interval, 1.0, 0.2F, 5.0F, "%.2f s");
    scaledSlider("jitter", spawner.intervalJitter, 1.0, 0.0F, 1.0F, "± %.2f s");
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::SliderFloat3("size", &spawner.size.x, 0.02F, 0.35F, "%.2f m");
    ImGui::SetItemTooltip("Length along the belt, height, width");
    const float length = spawner.conveyor < s.conveyors.size() ? s.conveyors[spawner.conveyor].spec.length : 1.0F;
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::SliderFloat("where", &spawner.offset, 0.0F, length, "%.2f m from the start");
    if (spawner.conveyor < s.conveyors.size()) {
        const float speed = s.conveyors[spawner.conveyor].spec.targetSpeed;
        ImGui::TextColored(kDim, "gap between boxes at full speed: %.3f m",
                           speed * static_cast<float>(spawner.interval) - spawner.size.x);
    }
    if (ImGui::Button("Spawn one now") && spawner.conveyor < world_->conveyors().size()) {
        const auto& c = world_->conveyors()[spawner.conveyor];
        world_->spawnItem(c.beltPoint(spawner.offset) + glm::vec3(0.0F, spawner.size.y / 2.0F + 0.002F, 0.0F),
                          spawner.size, {0.85F, 0.30F, 0.30F}, c.spec.heading);
    }
}

// The last triggers of the selected photocell (or the first one): when the
// beam was blocked, when the output switched, and how far past the beam the
// box's front was by then, also as a PLC scanning every few ms would see it.
void App::drawTriggers() {
    if (world_->photocells().empty()) {
        return;
    }
    const auto index =
        selected_ && selected_->kind == Pick::Kind::Photocell && selected_->index < world_->photocells().size()
            ? selected_->index
            : 0;
    const auto& photocell = world_->photocells()[index];
    ImGui::SetNextWindowPos(ImVec2(940, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(450, 330), ImGuiCond_FirstUseEver);
    ImGui::Begin("Triggers");
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
    ImGui::TextColored(kDim, "Milestone 5 will drive these from control logic; here, by hand.");
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
        ImGui::Text("%zu boxes landed in it", bin.count);
        ImGui::SameLine();
        if (ImGui::SmallButton("Reset count")) {
            bin.count = 0;
        }
    }
}

} // namespace fsim
