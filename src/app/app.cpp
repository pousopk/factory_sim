#include "app/app.hpp"
#include "render/scene.hpp"
#include <algorithm>
#include <imgui.h>

namespace fsim {

namespace {

const ImVec4 kDim{0.65F, 0.67F, 0.72F, 1.0F};

} // namespace

App::App() {
    resetScene();
}

// The starting scene: one 3 m conveyor along +X, boxes every 1.2 s.
void App::resetScene() {
    world_ = std::make_unique<World>();
    ConveyorSpec conveyor;
    conveyor.length = 3.0F;
    conveyor.targetSpeed = 0.5F;
    world_->addConveyor(conveyor);
    SpawnerSpec spawner;
    spawner.interval = 1.2;
    world_->addSpawner(spawner);
}

void App::frame(double realSeconds, int width, int height) {
    // Long pauses (the window was dragged, the computer was busy) shouldn't
    // turn into a burst of simulation: at most a quarter of a second at once.
    realSeconds = std::min(realSeconds, 0.25);
    frameSeconds_ = realSeconds;
    stepsLastFrame_ = paused_ ? 0 : world_->advance(realSeconds * timeScale_);

    handleMouse(height);
    renderer_.draw(sceneBoxes(*world_), camera_, width, height);
    drawPanels();
}

// Left drag turns around the target, right or middle drag moves it, the
// wheel zooms. Only when the mouse isn't over a panel.
void App::handleMouse(int height) {
    const auto& io = ImGui::GetIO();
    if (io.WantCaptureMouse) {
        return;
    }
    const auto windowHeight = static_cast<float>(height);
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Left, 0.0F)) {
        camera_.rotate(io.MouseDelta.x, io.MouseDelta.y, windowHeight);
    }
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Right, 0.0F) || ImGui::IsMouseDragging(ImGuiMouseButton_Middle, 0.0F)) {
        camera_.pan(io.MouseDelta.x, io.MouseDelta.y, windowHeight);
    }
    if (io.MouseWheel != 0.0F) {
        camera_.zoom(io.MouseWheel);
    }
}

void App::drawPanels() {
    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(340, 0), ImGuiCond_FirstUseEver);
    ImGui::Begin("factory_sim");

    // --- Time ---
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
    ImGui::SetNextItemWidth(180.0F);
    ImGui::SliderFloat("time scale", &timeScale_, 0.05F, 2.0F, "%.2fx", ImGuiSliderFlags_Logarithmic);
    ImGui::SetItemTooltip("How fast simulated time runs compared with real time.\n"
                          "The physics is the same at any scale: always 1 ms steps.");
    ImGui::TextColored(kDim, "%d steps this frame, %.1f ms per frame", stepsLastFrame_, frameSeconds_ * 1000.0);

    // --- The conveyor ---
    auto& conveyor = world_->conveyors().front();
    ImGui::SeparatorText(conveyor.spec.name.c_str());
    ImGui::SetNextItemWidth(180.0F);
    ImGui::SliderFloat("target speed", &conveyor.spec.targetSpeed, 0.0F, 3.0F, "%.2f m/s");
    ImGui::SetNextItemWidth(180.0F);
    ImGui::SliderFloat("acceleration", &conveyor.spec.acceleration, 0.1F, 10.0F, "%.1f m/s²",
                       ImGuiSliderFlags_Logarithmic);
    ImGui::SetItemTooltip("Like a VFD's ramp: 1 m/s² takes 1 s from 0 to 1 m/s.");
    const float fraction = conveyor.spec.targetSpeed > 0.0F ? conveyor.speed / conveyor.spec.targetSpeed : 0.0F;
    ImGui::ProgressBar(std::clamp(fraction, 0.0F, 1.0F), ImVec2(180.0F, 0.0F), "");
    ImGui::SameLine();
    ImGui::Text("%.3f m/s", conveyor.speed);
    ImGui::TextColored(kDim, "%.2f m long, %.2f m wide, belt at %.2f m", conveyor.spec.length, conveyor.spec.width,
                       conveyor.spec.beltHeight);
    ImGui::TextColored(kDim, "at this speed a box moves %.2f mm per 1 ms step", conveyor.speed); // m/s = mm/ms

    // --- Boxes ---
    auto& spawner = world_->spawners().front();
    ImGui::SeparatorText("Boxes");
    ImGui::Checkbox("spawn boxes", &spawner.enabled);
    ImGui::SetNextItemWidth(180.0F);
    float interval = static_cast<float>(spawner.interval);
    if (ImGui::SliderFloat("every", &interval, 0.2F, 5.0F, "%.2f s")) {
        spawner.interval = interval;
    }
    ImGui::SetNextItemWidth(180.0F);
    float jitter = static_cast<float>(spawner.intervalJitter);
    if (ImGui::SliderFloat("jitter", &jitter, 0.0F, 1.0F, "± %.2f s")) {
        spawner.intervalJitter = jitter;
    }
    ImGui::SetNextItemWidth(180.0F);
    ImGui::SliderFloat3("size", &spawner.size.x, 0.05F, 0.35F, "%.2f m");
    ImGui::SetItemTooltip("Length along the belt, height, width");
    const float gap = conveyor.speed * static_cast<float>(spawner.interval) - spawner.size.x;
    ImGui::TextColored(kDim, "gap between boxes at this speed: %.3f m", gap);
    if (ImGui::Button("Spawn one now")) {
        const auto& c = world_->conveyors().front();
        world_->spawnItem(c.beltPoint(spawner.offset) + glm::vec3(0.0F, spawner.size.y / 2.0F + 0.002F, 0.0F),
                          spawner.size, {0.85F, 0.30F, 0.30F}, c.spec.heading);
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear boxes")) {
        world_->clearItems();
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset")) {
        resetScene();
    }
    ImGui::Text("%zu boxes", world_->itemCount());

    ImGui::SeparatorText("Camera");
    ImGui::TextColored(kDim, "Left drag: turn. Right drag: move. Wheel: zoom.");
    ImGui::End();
}

} // namespace fsim
