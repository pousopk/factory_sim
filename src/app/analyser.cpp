// The logic analyser window: every signal over time, the spread of trigger
// errors, and the log of every switch. The data comes from analysis.hpp
// (tested without a window); this only plots it, with ImPlot.
#include "analysis/analysis.hpp"
#include "app/app.hpp"
#include <algorithm>
#include <deque>
#include <fmt/format.h>
#include <fstream>
#include <imgui.h>
#include <imgui_stdlib.h>
#include <implot.h>

namespace fsim {

namespace {

const ImVec4 kDim{0.65F, 0.67F, 0.72F, 1.0F};

} // namespace

void App::drawAnalyser() {
    if (!showAnalyser_) {
        return;
    }
    ImGui::SetNextWindowPos(ImVec2(360, 352), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(670, 430), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Logic analyser", &showAnalyser_)) {
        ImGui::End();
        return;
    }
    if (ImGui::BeginTabBar("analyser")) {
        if (ImGui::BeginTabItem("Timelines")) {
            drawTimelines();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Trigger errors")) {
            drawHistograms();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Event log")) {
            drawEventLog();
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    ImGui::End();
}

// Digital traces stacked one above the other, like a logic analyser's, and
// the belts' speeds under them, over the same span of time. Two markers
// measure the time between any two moments.
void App::drawTimelines() {
    const auto now = world_->time();
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10.8F);
    ImGui::SliderFloat("span", &analyserSeconds_, 0.1F, 60.0F, "%.1f s", ImGuiSliderFlags_Logarithmic);
    ImGui::SameLine();
    ImGui::Checkbox("follow", &followTime_);
    ImGui::SetItemTooltip("Off: the view stays put, to zoom (wheel) and pan (drag) through what happened");
    if (followTime_) {
        viewTo_ = now;
        viewFrom_ = now - analyserSeconds_;
    }
    if (!markersPlaced_ || markerA_ < viewFrom_ - 600.0) {
        markerA_ = viewFrom_ + 0.25 * (viewTo_ - viewFrom_);
        markerB_ = viewFrom_ + 0.5 * (viewTo_ - viewFrom_);
        markersPlaced_ = true;
    }
    ImGui::SameLine();
    ImGui::Text("   A-B = %.1f ms", (markerB_ - markerA_) * 1000.0);
    ImGui::SameLine();
    if (ImGui::SmallButton("Markers here")) {
        markerA_ = viewFrom_ + 0.25 * (viewTo_ - viewFrom_);
        markerB_ = viewFrom_ + 0.5 * (viewTo_ - viewFrom_);
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Signals...")) {
        ImGui::OpenPopup("signals");
    }
    const auto all = traces(*world_);
    if (ImGui::BeginPopup("signals")) {
        for (const auto& trace : all) {
            bool shown = hiddenTraces_.count(trace.name) == 0;
            if (ImGui::Checkbox(trace.name.c_str(), &shown)) {
                if (shown) {
                    hiddenTraces_.erase(trace.name);
                } else {
                    hiddenTraces_.insert(trace.name);
                }
            }
        }
        ImGui::EndPopup();
    }
    std::vector<const Trace*> shown;
    for (const auto& trace : all) {
        if (hiddenTraces_.count(trace.name) == 0) {
            shown.push_back(&trace);
        }
    }

    const float available = ImGui::GetContentRegionAvail().y;
    const float speedHeight = std::max(90.0F, available * 0.28F);
    // Lasers get a plot of their own, in millimetres, under the speeds.
    const bool lasers = !world_->lasers().empty();
    const float laserHeight = lasers ? speedHeight : 0.0F;
    const auto timeAxis = [&](bool labels) {
        ImPlot::SetupAxis(ImAxis_X1, labels ? "time (s)" : nullptr, labels ? 0 : ImPlotAxisFlags_NoTickLabels);
        ImPlot::SetupAxisLinks(ImAxis_X1, &viewFrom_, &viewTo_); // both plots show the same span
        if (followTime_) {
            ImPlot::SetupAxisLimits(ImAxis_X1, viewFrom_, viewTo_, ImPlotCond_Always);
        }
    };

    // The traces: trace k drawn between k and k + 0.7, the first one on top.
    if (ImPlot::BeginPlot("##traces", ImVec2(-1, available - speedHeight - laserHeight),
                          ImPlotFlags_NoLegend | ImPlotFlags_NoMenus)) {
        timeAxis(false);
        ImPlot::SetupAxis(ImAxis_Y1, nullptr, ImPlotAxisFlags_Lock | ImPlotAxisFlags_NoGridLines);
        const auto n = static_cast<double>(shown.size());
        ImPlot::SetupAxisLimits(ImAxis_Y1, -0.3, std::max(n, 1.0), ImPlotCond_Always);
        std::vector<double> positions;
        std::vector<const char*> labels;
        for (std::size_t k = 0; k < shown.size(); ++k) {
            positions.push_back(n - 1.0 - static_cast<double>(k) + 0.35);
            labels.push_back(shown[k]->name.c_str());
        }
        if (!shown.empty()) {
            ImPlot::SetupAxisTicks(ImAxis_Y1, positions.data(), static_cast<int>(positions.size()), labels.data());
        }
        for (std::size_t k = 0; k < shown.size(); ++k) {
            const auto shape = steps(*shown[k]->signal, viewFrom_, viewTo_);
            std::vector<double> ys(shape.values.size());
            const double base = n - 1.0 - static_cast<double>(k);
            std::transform(shape.values.begin(), shape.values.end(), ys.begin(),
                           [&](double v) { return base + 0.7 * v; });
            ImPlot::PlotLine(shown[k]->name.c_str(), shape.times.data(), ys.data(), static_cast<int>(ys.size()));
        }
        ImPlot::DragLineX(0, &markerA_, ImVec4(1.0F, 0.8F, 0.2F, 1.0F), 1.0F);
        ImPlot::TagX(markerA_, ImVec4(1.0F, 0.8F, 0.2F, 1.0F), "A");
        ImPlot::DragLineX(1, &markerB_, ImVec4(0.3F, 0.8F, 1.0F, 1.0F), 1.0F);
        ImPlot::TagX(markerB_, ImVec4(0.3F, 0.8F, 1.0F, 1.0F), "B");
        ImPlot::EndPlot();
    }

    // The belts' speeds, sampled every scan: ramps up and down, stops; and
    // what their drives were asked for.
    if (ImPlot::BeginPlot("##speeds", ImVec2(-1, speedHeight - 4.0F), ImPlotFlags_NoMenus)) {
        timeAxis(true);
        ImPlot::SetupAxis(ImAxis_Y1, "m/s");
        // The visible samples: each belt's speed, and for a belt with a
        // drive, the speed its PLC asked for (NSOLL_A, in m/s).
        const auto visible = [&](const std::deque<std::pair<double, float>>& samples, std::vector<double>& xs,
                                 std::vector<double>& ys) {
            for (const auto& [time, value] : samples) {
                if (time >= viewFrom_ - 0.01 && time <= viewTo_ + 0.01) {
                    xs.push_back(time);
                    ys.push_back(value);
                }
            }
        };
        struct Line {
            std::string name;
            std::vector<double> xs;
            std::vector<double> ys;
            bool setpoint = false;
        };
        std::vector<Line> lines;
        double top = 0.1;
        double bottom = 0.0;
        for (const auto& conveyor : world_->conveyors()) {
            lines.push_back(Line{conveyor.spec.name, {}, {}, false});
            visible(conveyor.speedSamples, lines.back().xs, lines.back().ys);
            if (conveyor.spec.drive.fitted) {
                lines.push_back(Line{conveyor.spec.name + " setpoint", {}, {}, true});
                visible(conveyor.setpointSamples, lines.back().xs, lines.back().ys);
            }
            top = std::max(top, static_cast<double>(conveyor.spec.targetSpeed));
        }
        for (const auto& line : lines) {
            for (const double y : line.ys) {
                top = std::max(top, y);
                bottom = std::min(bottom, y);
            }
        }
        ImPlot::SetupAxisLimits(ImAxis_Y1, bottom * 1.15 - 0.05, top * 1.15, ImPlotCond_Always);
        for (const auto& line : lines) {
            // A setpoint thin and faint, under the speed that follows it.
            const ImPlotSpec style = line.setpoint ? ImPlotSpec(ImPlotProp_LineWeight, 1.0F, ImPlotProp_FillAlpha, 0.5F)
                                                   : ImPlotSpec(ImPlotProp_LineWeight, 2.0F);
            ImPlot::PlotLine(line.name.c_str(), line.xs.data(), line.ys.data(), static_cast<int>(line.xs.size()),
                             style);
        }
        ImPlot::EndPlot();
    }

    // What each laser measured, as it measured it (in steps of its
    // resolution, with its noise); out of range is left out.
    if (lasers && ImPlot::BeginPlot("##distances", ImVec2(-1, laserHeight - 4.0F), ImPlotFlags_NoMenus)) {
        timeAxis(true);
        ImPlot::SetupAxis(ImAxis_Y1, "mm", ImPlotAxisFlags_AutoFit);
        for (const auto& laser : world_->lasers()) {
            std::vector<double> xs;
            std::vector<double> ys;
            for (const auto& [time, distance] : laser.samples) {
                if (time >= viewFrom_ - 0.01 && time <= viewTo_ + 0.01 && distance >= 0.0F) {
                    xs.push_back(time);
                    ys.push_back(distance * 1000.0);
                }
            }
            ImPlot::PlotLine((laser.spec.name + " distance").c_str(), xs.data(), ys.data(),
                             static_cast<int>(xs.size()));
        }
        ImPlot::EndPlot();
    }
}

// How a photocell's trigger position errors spread: from the sensor alone,
// and as a PLC scanning every few ms sees them.
void App::drawHistograms() {
    const auto& photocells = world_->photocells();
    if (photocells.empty()) {
        ImGui::TextColored(kDim, "No photocells.");
        return;
    }
    histogramPhotocell_ = std::clamp(histogramPhotocell_, 0, static_cast<int>(photocells.size()) - 1);
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10.8F);
    if (ImGui::BeginCombo("photocell", photocells[static_cast<std::size_t>(histogramPhotocell_)].spec.name.c_str())) {
        for (std::size_t i = 0; i < photocells.size(); ++i) {
            if (ImGui::Selectable(photocells[i].spec.name.c_str(), static_cast<int>(i) == histogramPhotocell_)) {
                histogramPhotocell_ = static_cast<int>(i);
            }
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10.8F);
    ImGui::SliderFloat("PLC scan", &scanMilliseconds_, 1.0F, 50.0F, "%.0f ms");
    const auto& photocell = photocells[static_cast<std::size_t>(histogramPhotocell_)];
    const auto own = positionErrors(photocell);
    const auto atScan = positionErrorsAtScan(photocell, scanMilliseconds_ / 1000.0);
    if (own.empty()) {
        ImGui::TextColored(kDim, "No triggers yet.");
        return;
    }
    const auto line = [](const char* what, const Statistics& s) {
        ImGui::Text("%-12s %3zu triggers  mean %6.2f mm  deviation %5.2f mm  range %6.2f to %6.2f mm", what, s.count,
                    s.mean, s.deviation, s.min, s.max);
    };
    line("sensor", statistics(own));
    line("at the scan", statistics(atScan));
    ImGui::TextColored(kDim,
                       "How far past the beam's centre the box's front was when the signal arrived "
                       "(the last %zu triggers).",
                       own.size());
    // One range for both, so their bins line up. (All the sensor's errors
    // can be the same value: a range with no width would have no bins.)
    const auto [low, high] = std::minmax_element(atScan.begin(), atScan.end());
    const double from = std::min(*low, *std::min_element(own.begin(), own.end())) - 0.25;
    const double to = std::max(*high, *std::max_element(own.begin(), own.end())) + 0.25;
    const ImPlotRange range(from, to);
    const int bins = std::max(10, static_cast<int>((to - from) / 0.1));
    if (ImPlot::BeginPlot("##errors", ImVec2(-1, -1))) {
        // Refit every frame: the data grows, and changes with the scan time.
        ImPlot::SetupAxes("position error (mm)", "triggers", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
        // ImPlot 1.0 styles an item through its spec (SetNextFillStyle is gone).
        ImPlot::PlotHistogram(
            "sensor", own.data(), static_cast<int>(own.size()), bins, 1.0, range,
            ImPlotSpec(ImPlotProp_FillColor, ImVec4(0.45F, 0.85F, 0.50F, 1.0F), ImPlotProp_FillAlpha, 0.6F));
        ImPlot::PlotHistogram(
            "at the PLC scan", atScan.data(), static_cast<int>(atScan.size()), bins, 1.0, range,
            ImPlotSpec(ImPlotProp_FillColor, ImVec4(0.30F, 0.65F, 1.0F, 1.0F), ImPlotProp_FillAlpha, 0.5F));
        ImPlot::EndPlot();
    }
}

// Every switch of every signal, newest first, and saved as CSV.
void App::drawEventLog() {
    const auto log = eventLog(traces(*world_));
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 15.4F);
    ImGui::InputTextWithHint("##filter", "filter: part of a name", &eventFilter_);
    ImGui::SameLine();
    if (ImGui::SmallButton("Export CSV")) {
        const auto path = sceneFolder() / "events.csv";
        std::ofstream file(path, std::ios::binary);
        file << eventsCsv(log);
        exportStatus_ =
            file ? fmt::format("wrote {} events to {}", log.size(), path.string()) : "can't write " + path.string();
    }
    ImGui::SetItemTooltip("Every switch, with its exact time, into events.csv next to the scene file");
    ImGui::SameLine();
    ImGui::TextColored(kDim, "%s", exportStatus_.c_str());
    if (ImGui::BeginTable("events", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY)) {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("time (s)", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFontSize() * 6.9F);
        ImGui::TableSetupColumn("signal");
        ImGui::TableSetupColumn("address", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFontSize() * 5.4F);
        ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFontSize() * 3.5F);
        ImGui::TableHeadersRow();
        int rows = 0;
        for (auto it = log.rbegin(); it != log.rend() && rows < 2000; ++it) {
            if (!eventFilter_.empty() && it->signal.find(eventFilter_) == std::string::npos) {
                continue;
            }
            ++rows;
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%.6f", it->time);
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(it->signal.c_str());
            ImGui::TableNextColumn();
            ImGui::TextColored(kDim, "%s", it->address.c_str());
            ImGui::TableNextColumn();
            ImGui::TextColored(it->value ? ImVec4(1.0F, 0.85F, 0.1F, 1.0F) : kDim, "%s", it->value ? "1" : "0");
        }
        ImGui::EndTable();
    }
}

} // namespace fsim
