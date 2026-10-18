#include "app/app.hpp"
#include "render/scene.hpp"
#include <algorithm>
#include <cmath>
#include <imgui.h>

namespace fsim {

namespace {

const ImVec4 kDim{0.65F, 0.67F, 0.72F, 1.0F};

} // namespace

App::App() {
    resetScene();
}

// The starting scene: one 3 m conveyor along +X, boxes every 1.2 s, a
// photocell halfway along.
void App::resetScene() {
    world_ = std::make_unique<World>();
    ConveyorSpec conveyor;
    conveyor.length = 3.0F;
    conveyor.targetSpeed = 0.5F;
    world_->addConveyor(conveyor);
    SpawnerSpec spawner;
    spawner.interval = 1.2;
    world_->addSpawner(spawner);
    PhotocellSpec photocell;
    photocell.name = "B1";
    photocell.along = 1.5F;
    world_->addPhotocell(photocell);
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
    drawSensorPanel();
    drawTriggers();
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

    // --- The encoder on its drive roller ---
    ImGui::SeparatorText("Encoder");
    auto& encoder = conveyor.spec.encoder;
    ImGui::SetNextItemWidth(180.0F);
    ImGui::SliderInt("pulses/rev", &encoder.pulsesPerRevolution, 100, 5000);
    ImGui::SetNextItemWidth(180.0F);
    ImGui::SliderFloat("roller", &encoder.rollerDiameter, 0.02F, 0.2F, "%.3f m");
    const double perMetre = encoder.pulsesPerMetre();
    ImGui::Text("count %lld", static_cast<long long>(conveyor.encoderCount()));
    ImGui::TextColored(kDim, "%.0f pulses/m, %.3f mm per pulse, %.0f Hz now", perMetre, 1000.0 / perMetre,
                       conveyor.speed * perMetre);

    ImGui::SeparatorText("Camera");
    ImGui::TextColored(kDim, "Left drag: turn. Right drag: move. Wheel: zoom.");
    ImGui::End();
}

void App::drawSensorPanel() {
    if (world_->photocells().empty()) {
        return;
    }
    auto& photocell = world_->photocells().front();
    auto& spec = photocell.spec;
    ImGui::SetNextWindowPos(ImVec2(10, 560), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(340, 0), ImGuiCond_FirstUseEver);
    ImGui::Begin("Photocell");
    // The output, as an LED, and how much of the beam is covered right now.
    const auto on = photocell.output.value();
    ImGui::ColorButton("##led", on ? ImVec4(1.0F, 0.85F, 0.1F, 1.0F) : ImVec4(0.25F, 0.22F, 0.12F, 1.0F),
                       ImGuiColorEditFlags_NoTooltip, ImVec2(14, 14));
    ImGui::SameLine();
    ImGui::Text("%s output %s", spec.name.c_str(), on ? "ON" : "off");
    ImGui::SameLine();
    ImGui::TextColored(kDim, "  beam %.0f %% covered", photocell.covered * 100.0F);

    const float length = world_->conveyors()[spec.conveyor].spec.length;
    ImGui::SetNextItemWidth(180.0F);
    ImGui::SliderFloat("position", &spec.along, 0.1F, length - 0.1F, "%.3f m");
    ImGui::SetNextItemWidth(180.0F);
    ImGui::SliderFloat("height", &spec.height, 0.005F, 0.3F, "%.3f m above belt");
    ImGui::SetNextItemWidth(180.0F);
    float response = static_cast<float>(spec.responseTime * 1000.0);
    if (ImGui::SliderFloat("response", &response, 0.0F, 20.0F, "%.2f ms")) {
        spec.responseTime = response / 1000.0;
    }
    ImGui::SetItemTooltip("From the beam being blocked to the output switching.\n"
                          "Typical photocells: 0.25 to 2 ms.");
    ImGui::SetNextItemWidth(180.0F);
    float diameter = spec.beamDiameter * 1000.0F;
    if (ImGui::SliderFloat("beam", &diameter, 1.0F, 30.0F, "%.1f mm")) {
        spec.beamDiameter = diameter / 1000.0F;
    }
    ImGui::SetNextItemWidth(180.0F);
    float on100 = spec.onFraction * 100.0F;
    float off100 = spec.offFraction * 100.0F;
    if (ImGui::SliderFloat("on above", &on100, 10.0F, 95.0F, "%.0f %% covered")) {
        spec.onFraction = on100 / 100.0F;
        spec.offFraction = std::min(spec.offFraction, spec.onFraction);
    }
    ImGui::SetNextItemWidth(180.0F);
    if (ImGui::SliderFloat("off below", &off100, 5.0F, 90.0F, "%.0f %% covered")) {
        spec.offFraction = std::min(off100 / 100.0F, spec.onFraction);
    }
    ImGui::TextColored(kDim, "smallest object it sees: %.1f mm long", spec.onFraction * spec.beamDiameter * 1000.0F);
    ImGui::End();
}

// The last triggers: when the beam was blocked, when the output switched,
// and how far past the beam the box's front was by then, also as a PLC
// scanning every few ms would see it.
void App::drawTriggers() {
    if (world_->photocells().empty()) {
        return;
    }
    const auto& photocell = world_->photocells().front();
    ImGui::SetNextWindowPos(ImVec2(940, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(450, 330), ImGuiCond_FirstUseEver);
    ImGui::Begin("Triggers");
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

} // namespace fsim
