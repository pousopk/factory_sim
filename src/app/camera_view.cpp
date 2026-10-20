// What the cameras see. An area camera: its live view (as if it took a
// picture every frame) and the last picture it took when triggered, drawn
// from its own position and lens. A line-scan camera: its lines so far,
// stacked into an image. Exposure, as in brightness, isn't modelled; what
// moving does during the exposure (blur) and noise are.
#include "app/app.hpp"
#include "app/widgets.hpp"
#include <algorithm>
#include <cmath>
#include <fmt/format.h>
#include <imgui.h>

namespace fsim {

namespace {

ImTextureRef textureOf(GLuint texture) {
    return ImTextureRef(static_cast<ImTextureID>(texture));
}

} // namespace

// The scene at moments spread over an exposure centred on `middle`: the
// fixed parts as they are, the items moved back (or on) along their
// velocities. Enough moments for a smooth smear (two per pixel moved, at
// most 12); one when nothing moves enough to blur.
std::vector<std::vector<DrawBox>> App::exposureMoments(const Camera& camera, double middle) const {
    const auto& spec = camera.spec;
    std::vector<DrawBox> fixed;
    for (const auto& box : sceneBoxes(*world_)) {
        // Not the items (added below), ghosts and see-through helpers (a
        // field of view on the belt, a sensor's reach), or the cameras.
        if (box.pick.kind != Pick::Kind::None && box.pick.kind != Pick::Kind::Item &&
            box.pick.kind != Pick::Kind::Camera && box.alpha >= 1.0F) {
            fixed.push_back(box);
        }
    }
    const auto items = world_->items();
    float fastest = 0.0F;
    for (const auto& item : items) {
        fastest = std::max(fastest, glm::length(item.velocity));
    }
    const float pixel = spec.fieldOfView / static_cast<float>(spec.pixels);
    const float smear = fastest * static_cast<float>(spec.exposure) / pixel;
    const int count = std::clamp(static_cast<int>(std::ceil(smear * 2.0F)), 1, 12);
    std::vector<std::vector<DrawBox>> moments;
    for (int k = 0; k < count; ++k) {
        const double t = middle + spec.exposure * ((static_cast<double>(k) + 0.5) / count - 0.5);
        const auto dt = static_cast<float>(t - world_->time());
        auto boxes = fixed;
        for (const auto& item : items) {
            auto box = itemBox(item);
            box.center += item.velocity * dt;
            boxes.push_back(box);
        }
        moments.push_back(std::move(boxes));
    }
    return moments;
}

void App::drawCameraView() {
    const auto& cameras = world_->cameras();
    if (!showCameraView_ || cameras.empty()) {
        return;
    }
    ImGui::SetNextWindowPos(ImVec2(700, 32), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(520, 460), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Camera view", &showCameraView_)) {
        ImGui::End();
        return;
    }
    viewedCamera_ = std::clamp(viewedCamera_, 0, static_cast<int>(cameras.size()) - 1);
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 9.2F);
    if (ImGui::BeginCombo("camera", cameras[static_cast<std::size_t>(viewedCamera_)].spec.name.c_str())) {
        for (std::size_t i = 0; i < cameras.size(); ++i) {
            if (ImGui::Selectable(cameras[i].spec.name.c_str(), static_cast<int>(i) == viewedCamera_)) {
                viewedCamera_ = static_cast<int>(i);
                lastPictureAt_ = -1.0;
                linesShown_ = 0;
            }
        }
        ImGui::EndCombo();
    }
    const auto& camera = cameras[static_cast<std::size_t>(viewedCamera_)];
    const auto& spec = camera.spec;
    const bool onBelt = spec.place.parent == Parent::Conveyor;
    if (onBelt && spec.conveyor >= world_->conveyors().size()) {
        ImGui::End();
        return;
    }
    static const Conveyor none; // one standing on its own looks from its placement, not a belt
    const auto& conveyor = spec.conveyor < world_->conveyors().size() ? world_->conveyors()[spec.conveyor] : none;
    const float available = ImGui::GetContentRegionAvail().x;

    if (spec.kind == CameraKind::Line) {
        // The lines, oldest at the top, one screen row per line where it fits:
        // the image's proportions are the camera's, pixel for pixel.
        if (camera.image.taken != linesShown_) {
            lineTexture_.upload(camera.image);
            linesShown_ = camera.image.taken;
        }
        const double across = spec.lineWidth / spec.pixelsAcross;
        const double along = spec.lineTrigger == LineTrigger::FreeRun
                                 ? conveyor.speed / spec.lineRate
                                 : spec.pulsesPerLine / conveyor.spec.encoder.pulsesPerMetre();
        ImGui::SameLine();
        ImGui::TextColored(kDim, "%llu lines; pixel %.2f mm across, %.2f mm along",
                           static_cast<unsigned long long>(camera.image.taken), across * 1000.0, along * 1000.0);
        if (lineTexture_.texture() != 0 && lineTexture_.width() > 0) {
            // As wide as the panel, but small enough that the last 600 lines
            // (half a metre or so: a whole product) fit its height; the same
            // scale both ways, so the image keeps its proportions.
            const float height = std::max(ImGui::GetContentRegionAvail().y, 50.0F);
            const float lines = static_cast<float>(std::min(lineTexture_.height(), 600));
            const float width = static_cast<float>(lineTexture_.width());
            const float scale = std::min((available - ImGui::GetStyle().ScrollbarSize) / width, height / lines);
            ImGui::BeginChild("lines", ImVec2(0, 0));
            ImGui::SetCursorPosX((available - width * scale) / 2.0F);
            ImGui::Image(textureOf(lineTexture_.texture()),
                         ImVec2(width * scale, static_cast<float>(lineTexture_.height()) * scale));
            ImGui::SetScrollHereY(1.0F); // follow the newest line
            ImGui::EndChild();
        }
        ImGui::End();
        return;
    }

    // An area camera: its view, as a picture taken now, and its last
    // triggered picture, taken at its exposure.
    const auto view = cameraView(spec, conveyor);
    const auto viewProjection = view.projection * view.view;
    renderer_.drawPicture(livePicture_, exposureMoments(camera, world_->time() - spec.exposure / 2.0), viewProjection,
                          view.eye, view.width, view.height, spec.noise, static_cast<unsigned>(world_->stepCount()));
    const Shot* last = nullptr;
    for (const auto& shot : camera.shots) {
        if (shot.taken) {
            last = &shot;
            break;
        }
    }
    if (last != nullptr && last->exposure != lastPictureAt_) {
        renderer_.drawPicture(lastPicture_, exposureMoments(camera, last->exposure), viewProjection, view.eye,
                              view.width, view.height, spec.noise, static_cast<unsigned>(last->exposure * 1000.0));
        lastPictureAt_ = last->exposure;
    }
    const float pixel = spec.fieldOfView / static_cast<float>(spec.pixels);
    ImGui::SameLine();
    ImGui::TextColored(kDim, "%d x %d, pixel %.3f mm, blur %.2f px at %.2f m/s", spec.pixels, spec.pixelsAcross,
                       pixel * 1000.0F, conveyor.speed * spec.exposure / pixel, conveyor.speed);
    const ImVec2 size(available, available * static_cast<float>(view.height) / static_cast<float>(view.width));
    if (ImGui::BeginTabBar("pictures")) {
        if (ImGui::BeginTabItem("Live")) {
            // OpenGL's rows go up from the bottom: flipped to show.
            ImGui::Image(textureOf(livePicture_.texture()), size, ImVec2(0, 1), ImVec2(1, 0));
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Last picture")) {
            if (lastPicture_.texture() != 0 && last != nullptr) {
                ImGui::TextColored(kDim, "taken at %.4f s: item %llu, %+.1f mm from the middle", last->exposure,
                                   static_cast<unsigned long long>(last->item), last->offset * 1000.0F);
                ImGui::Image(textureOf(lastPicture_.texture()), size, ImVec2(0, 1), ImVec2(1, 0));
            } else {
                ImGui::TextColored(kDim,
                                   "No picture yet: it takes one when triggered (or Trigger now in the inspector).");
            }
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    ImGui::End();
}

} // namespace fsim
