// The cameras: a camera's settings, the built-in camera triggering, and the
// Cameras window, which answers the tin line's two questions: which camera
// missed which tin, and how close tins can come before one does.
#include "analysis/tin_study.hpp"
#include "app/app.hpp"
#include "app/widgets.hpp"
#include "plc/programs.hpp"
#include <algorithm>
#include <chrono>
#include <fmt/format.h>
#include <imgui.h>
#include <imgui_stdlib.h>
#include <implot.h>
#include <map>

namespace fsim {

namespace {

const ImVec4 kGood{0.45F, 0.85F, 0.50F, 1.0F};
const ImVec4 kWarn{0.95F, 0.80F, 0.30F, 1.0F};

// The speed and item size on a camera's conveyor, as the scene sets them.
struct LineFacts {
    float speed = 0.0F;
    float itemLength = 0.0F;
};

LineFacts lineFacts(const Scene& scene, std::size_t conveyor) {
    LineFacts facts;
    if (conveyor < scene.conveyors.size()) {
        facts.speed = scene.conveyors[conveyor].spec.targetSpeed;
    }
    for (const auto& spawner : scene.spawners) {
        if (spawner.conveyor == conveyor) {
            facts.itemLength = spawner.size.x;
        }
    }
    return facts;
}

} // namespace

void App::drawCameraInspector(std::size_t index) {
    auto& s = scene();
    auto& spec = s.cameras[index];
    if (index < world_->cameras().size()) {
        const auto& camera = world_->cameras()[index];
        const bool busy = camera.timing.busyAt(world_->time());
        ImGui::ColorButton("##ready", busy ? ImVec4(1.0F, 0.85F, 0.1F, 1.0F) : ImVec4(0.25F, 0.22F, 0.12F, 1.0F),
                           ImGuiColorEditFlags_NoTooltip, ImVec2(14, 14));
        ImGui::SameLine();
        ImGui::TextUnformatted(busy ? "busy" : "ready for a trigger");
        ImGui::SameLine();
        if (ImGui::SmallButton("Trigger now")) {
            world_->triggerCamera(index);
        }
    }
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::InputText("name", &spec.name);
    conveyorCombo("over conveyor", spec.conveyor, s);
    const float length = spec.conveyor < s.conveyors.size() ? s.conveyors[spec.conveyor].spec.length : 1.0F;
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::SliderFloat("looks at", &spec.along, 0.0F, length, "%.3f m");
    ImGui::SetItemTooltip("The middle of its picture, along the belt");
    scaledSlider("from", spec.tilt, kDegrees, -80.0F, 80.0F, "%.0f deg");
    ImGui::SetItemTooltip(
        "0: straight down; positive from the left, negative from the right (looking along the travel)");
    scaledSlider("field of view", spec.fieldOfView, 1000.0F, 20.0F, 600.0F, "%.0f mm");
    ImGui::SetNextItemWidth(kFieldWidth);
    ImGui::SliderInt("pixels", &spec.pixels, 320, 5000);
    ImGui::SetItemTooltip("Across the field of view, along the travel");

    ImGui::SeparatorText("Timing");
    scaledSlider("trigger delay", spec.triggerDelay, 1000.0, 0.0F, 300.0F, "%.1f ms");
    ImGui::SetItemTooltip("The camera's own delay from the trigger to the exposure. It's busy meanwhile.");
    scaledSlider("exposure", spec.exposure, 1000.0, 0.02F, 10.0F, "%.2f ms");
    scaledSlider("readout", spec.readout, 1000.0, 1.0F, 200.0F, "%.0f ms");
    ImGui::SetItemTooltip("Getting the picture off the sensor and over the network: a 5 MP picture over\n"
                          "Gigabit Ethernet takes about 40 ms. It can't take another one meanwhile.");
    const auto facts = lineFacts(s, spec.conveyor);
    ImGui::TextColored(kDim, "busy %.1f ms per picture: at most %.0f pictures/s", spec.busyTime() * 1000.0,
                       1.0 / spec.busyTime());
    if (facts.speed > 0.0F) {
        const double blur = facts.speed * spec.exposure;
        ImGui::TextColored(kDim, "at %.2f m/s: items at least %.0f mm apart (centre to centre)", facts.speed,
                           facts.speed * spec.busyTime() * 1000.0);
        const double pixelsPerMetre = static_cast<double>(spec.pixels) / spec.fieldOfView;
        ImGui::TextColored(blur * pixelsPerMetre > 1.0 ? kWarn : kDim, "motion blur %.2f mm = %.1f pixels",
                           blur * 1000.0, blur * pixelsPerMetre);
    }

    ImGui::SeparatorText("I/O");
    addressField("trigger", spec.triggerOutput, IoAddress::Area::Output, IoAddress::Size::Bit);
    ImGui::SetItemTooltip("The PLC output wired to its trigger line: a rising edge takes a picture");
    addressField("ready", spec.readyInput, IoAddress::Area::Input, IoAddress::Size::Bit);
    ImGui::SetItemTooltip("The PLC input its \"ready for a trigger\" line is wired to (off while busy)");
}

void App::drawCameraTriggering() {
    auto& s = scene();
    const auto* triggering = dynamic_cast<const CameraTriggering*>(world_->program());
    const auto status = triggering != nullptr ? triggering->status() : std::vector<CameraTriggering::Status>{};
    std::optional<std::size_t> remove;
    for (std::size_t i = 0; i < s.cameraTriggers.size(); ++i) {
        auto& t = s.cameraTriggers[i];
        ImGui::PushID(static_cast<int>(i));
        const auto label = i < status.size() ? fmt::format("{}: {} triggers, {} lost###trigger", t.name,
                                                           status[i].fired, status[i].lost)
                                             : t.name + "###trigger";
        if (ImGui::TreeNode(label.c_str())) {
            ImGui::SetNextItemWidth(kFieldWidth);
            ImGui::InputText("name", &t.name);
            addressField("sensor", t.sensorInput, IoAddress::Area::Input, IoAddress::Size::Bit);
            addressField("trigger", t.triggerOutput, IoAddress::Area::Output, IoAddress::Size::Bit);
            scaledSlider("delay", t.delay, 1000.0, 0.0F, 500.0F, "%.0f ms");
            ImGui::SetItemTooltip("From the sensor's rising edge (as the PLC sees it) to the trigger");
            int method = t.method == DelayMethod::Timer ? 0 : 1;
            ImGui::SetNextItemWidth(kFieldWidth);
            if (ImGui::Combo("delayed by", &method, "one timer\0a queue, per item\0")) {
                t.method = method == 0 ? DelayMethod::Timer : DelayMethod::PerItem;
            }
            ImGui::SetItemTooltip("One timer: a second item arriving while it runs gets no trigger.\n"
                                  "A queue: every item gets its own trigger time.");
            if (ImGui::SmallButton("Remove")) {
                remove = i;
            }
            ImGui::TreePop();
        }
        ImGui::PopID();
    }
    if (remove) {
        s.cameraTriggers.erase(s.cameraTriggers.begin() + static_cast<std::ptrdiff_t>(*remove));
    }
    if (!s.photocells.empty() && !s.cameras.empty()) {
        int& sensor = newTriggerSensor_;
        int& camera = newTriggerCamera_;
        sensor = std::clamp(sensor, 0, static_cast<int>(s.photocells.size()) - 1);
        camera = std::clamp(camera, 0, static_cast<int>(s.cameras.size()) - 1);
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
        if (ImGui::BeginCombo("##camera", s.cameras[static_cast<std::size_t>(camera)].name.c_str())) {
            for (std::size_t i = 0; i < s.cameras.size(); ++i) {
                if (ImGui::Selectable(s.cameras[i].name.c_str(), static_cast<int>(i) == camera)) {
                    camera = static_cast<int>(i);
                }
            }
            ImGui::EndCombo();
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Add trigger")) {
            s.cameraTriggers.push_back(
                triggerFor(s, static_cast<std::size_t>(sensor), static_cast<std::size_t>(camera)));
        }
        ImGui::SetItemTooltip("From this sensor to this camera, with the delay that puts an item in the middle\n"
                              "of its picture at the belt's speed");
    }
}

void App::drawCameras() {
    if (!showCameras_) {
        return; // hidden (View menu)
    }
    if (scene().cameras.empty()) {
        return;
    }
    ImGui::SetNextWindowPos(ImVec2(360, 542), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(670, 370), ImGuiCond_FirstUseEver);
    ImGui::Begin("Cameras", &showCameras_);
    if (ImGui::BeginTabBar("cameras")) {
        if (ImGui::BeginTabItem("Which camera took which item")) {
            drawTinTable();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Gap study")) {
            drawGapStudy();
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    ImGui::End();
}

// The last items to pass, one row each, and for each camera: taken (and
// where the item was in the picture), ignored because the camera was busy,
// never triggered, or not there yet.
void App::drawTinTable() {
    const auto& cameras = world_->cameras();
    // Per camera: pictures, ignored triggers.
    for (const auto& camera : cameras) {
        const auto taken =
            std::count_if(camera.shots.begin(), camera.shots.end(), [](const Shot& s) { return s.taken; });
        const auto ignored = static_cast<long>(camera.shots.size()) - taken;
        ImGui::TextUnformatted(camera.spec.name.c_str());
        ImGui::SameLine();
        ImGui::TextColored(ignored > 0 ? kProblem : kDim, "%ld pictures, %ld triggers ignored", taken, ignored);
        ImGui::SameLine();
    }
    ImGui::NewLine();

    // The items the cameras saw or should have seen, newest first.
    std::map<ItemId, int, std::greater<>> rows;
    for (const auto& camera : cameras) {
        for (const auto& shot : camera.shots) {
            if (shot.item != 0) {
                rows[shot.item] = 0;
            }
        }
    }
    for (const auto& photocell : world_->photocells()) {
        for (const auto& trigger : photocell.triggers) {
            if (!cameras.empty() && photocell.spec.conveyor == cameras.front().spec.conveyor && trigger.item != 0) {
                rows[trigger.item] = 0;
            }
        }
    }
    const auto items = world_->items();
    const auto columns = static_cast<int>(cameras.size()) + 1;
    if (ImGui::BeginTable("tins", columns, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY)) {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("item", ImGuiTableColumnFlags_WidthFixed, 50.0F);
        for (const auto& camera : cameras) {
            ImGui::TableSetupColumn(camera.spec.name.c_str());
        }
        ImGui::TableHeadersRow();
        int shown = 0;
        for (const auto& [id, unused] : rows) {
            if (++shown > 60) {
                break;
            }
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%llu", static_cast<unsigned long long>(id));
            const auto item = std::find_if(items.begin(), items.end(), [&](const ItemView& v) { return v.id == id; });
            for (const auto& camera : cameras) {
                ImGui::TableNextColumn();
                const auto taken = std::find_if(camera.shots.begin(), camera.shots.end(),
                                                [&](const Shot& s) { return s.taken && s.item == id; });
                if (taken != camera.shots.end()) {
                    ImGui::TextColored(taken->whole ? kGood : kWarn, "%s %+.1f mm", taken->whole ? "ok" : "part",
                                       taken->offset * 1000.0F);
                    if (ImGui::IsItemHovered()) {
                        ImGui::SetTooltip("Its centre %.1f mm from the middle of the picture%s, blurred by %.2f mm",
                                          taken->offset * 1000.0F, taken->whole ? "" : " (not all of it in it)",
                                          taken->blur * 1000.0F);
                    }
                    continue;
                }
                const bool ignored = std::any_of(camera.shots.begin(), camera.shots.end(),
                                                 [&](const Shot& s) { return !s.taken && s.item == id; });
                if (ignored) {
                    ImGui::TextColored(kProblem, "MISSED: busy");
                    ImGui::SetItemTooltip("Its trigger came while the camera was still busy with the last picture");
                    continue;
                }
                // Not taken, not ignored: has it gone past the picture yet?
                bool past = item == items.end();
                if (!past && camera.spec.conveyor < world_->conveyors().size()) {
                    const auto& conveyor = world_->conveyors()[camera.spec.conveyor];
                    const float along = glm::dot(item->position - conveyor.beltPoint(0.0F), conveyor.direction());
                    past = along - item->size.x / 2.0F > camera.spec.along + camera.spec.fieldOfView / 2.0F;
                }
                if (past) {
                    ImGui::TextColored(kProblem, "MISSED: no trigger");
                    ImGui::SetItemTooltip("The camera never got a trigger for it (or the picture had another item)");
                } else {
                    ImGui::TextColored(kDim, "...");
                }
            }
        }
        ImGui::EndTable();
    }
}

// Every gap from 0 up, with the line's timing alone (see tin_study.hpp):
// which camera misses tins at which gap, and the smallest gap that works.
void App::drawGapStudy() {
    const auto& s = scene();
    if (studyRunning_.valid() && studyRunning_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        study_ = studyRunning_.get();
    }
    const bool running = studyRunning_.valid();
    const auto conveyor = s.cameras.front().conveyor;
    const auto facts = lineFacts(s, conveyor);
    ImGui::TextColored(kDim, "%s at %.2f m/s, items %.0f mm long, a 10 ms scan, %s",
                       conveyor < s.conveyors.size() ? s.conveyors[conveyor].spec.name.c_str() : "?", facts.speed,
                       facts.itemLength * 1000.0F,
                       s.control == Control::SoftPlc ? "soft_plc's program" : "the built-in triggering");
    ImGui::SetNextItemWidth(140.0F);
    scaledSlider("gaps up to", studyUpTo_, 1000.0F, 20.0F, 1000.0F, "%.0f mm");
    ImGui::SameLine();
    ImGui::BeginDisabled(running || s.control == Control::S7Plc);
    if (ImGui::Button(running ? "Studying..." : "Study")) {
        // The program for each run: the scene's, made fresh. soft_plc without
        // its monitor port: the running world's soft_plc holds it.
        ProgramMaker make = [folder = sceneFolder()](const Scene& scene) {
            auto quiet = scene;
            quiet.monitorPort = 0;
            return makeAnyProgram(quiet, folder).program;
        };
        studied_ = s;
        studyRunning_ = std::async(std::launch::async, [scene = s, upTo = studyUpTo_, make] {
            return sweepGaps(scene, 0.0, upTo, 0.0025, make);
        });
    }
    ImGui::EndDisabled();
    if (s.control == Control::S7Plc) {
        ImGui::SameLine();
        ImGui::TextColored(kDim, "(a live PLC runs in real time: it can't be studied this way)");
    }
    if (study_.empty()) {
        ImGui::TextColored(kDim, "Tins evenly spaced at every gap from 0, each run several times across the scan.");
        return;
    }
    if (studied_ != s) {
        ImGui::SameLine();
        ImGui::TextColored(kWarn, "the scene has changed since: study again");
    }

    // The answer: the smallest gap that works, per camera and for the line.
    const auto describe = [&](std::optional<double> gap) -> std::string {
        if (!gap) {
            return "none in this range";
        }
        const double pitch = *gap + facts.itemLength;
        return facts.speed > 0.0F ? fmt::format("{:.1f} mm ({:.0f} ms apart, {:.0f} items/min)", *gap * 1000.0,
                                                pitch / facts.speed * 1000.0, facts.speed / pitch * 60.0)
                                  : fmt::format("{:.1f} mm", *gap * 1000.0);
    };
    ImGui::Text("Every camera takes every item from a gap of %s", describe(minimumGap(study_)).c_str());
    const auto& names = study_.front().cameras;
    for (std::size_t c = 0; c < names.size(); ++c) {
        ImGui::TextColored(kDim, "  %s from %s", names[c].name.c_str(), describe(minimumGap(study_, c)).c_str());
    }

    // Missed items against the gap, one line per camera, and the spawner's gaps.
    if (ImPlot::BeginPlot("##study", ImVec2(-1, -1))) {
        ImPlot::SetupAxes("gap between items (mm)", "items missed (%)");
        ImPlot::SetupAxisLimits(ImAxis_Y1, -5.0, 105.0, ImPlotCond_Always);
        std::vector<double> gaps;
        for (const auto& g : study_) {
            gaps.push_back(g.gap * 1000.0);
        }
        for (std::size_t c = 0; c < names.size(); ++c) {
            std::vector<double> missed;
            for (const auto& g : study_) {
                const auto& o = g.cameras[c];
                missed.push_back(o.items > 0 ? 100.0 * static_cast<double>(o.missed) / static_cast<double>(o.items)
                                             : 0.0);
            }
            ImPlot::PlotLine(names[c].name.c_str(), gaps.data(), missed.data(), static_cast<int>(gaps.size()));
        }
        // Where the spawner puts the items: its mean gap and its range.
        for (const auto& spawner : s.spawners) {
            if (spawner.conveyor == conveyor && facts.speed > 0.0F) {
                const double mean = (facts.speed * spawner.interval - facts.itemLength) * 1000.0;
                const double spread = facts.speed * spawner.intervalJitter * 1000.0;
                const double lines[] = {mean - spread, mean, mean + spread};
                ImPlot::PlotInfLines("the spawner's gaps", lines, spread > 0.0 ? 3 : 1);
            }
        }
        ImPlot::EndPlot();
    }
}

} // namespace fsim
