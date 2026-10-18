#include "sim/sensors.hpp"
#include <algorithm>
#include <numbers>

namespace fsim {

float beamCoverage(float centre, float diameter, std::vector<std::pair<float, float>> intervals) {
    if (diameter <= 0.0F) {
        return 0.0F;
    }
    const float beamFrom = centre - diameter / 2.0F;
    const float beamTo = centre + diameter / 2.0F;
    // Clip each interval to the beam, then merge overlapping ones so a spot
    // covered by two boxes (touching, or one tipped onto another) counts once.
    std::vector<std::pair<float, float>> clipped;
    for (const auto& [from, to] : intervals) {
        const float a = std::max(from, beamFrom);
        const float b = std::min(to, beamTo);
        if (b > a) {
            clipped.emplace_back(a, b);
        }
    }
    std::sort(clipped.begin(), clipped.end());
    float covered = 0.0F;
    float reached = beamFrom;
    for (const auto& [a, b] : clipped) {
        const float start = std::max(a, reached);
        if (b > start) {
            covered += b - start;
            reached = b;
        }
    }
    return std::clamp(covered / diameter, 0.0F, 1.0F);
}

std::optional<double> Photocell::sense(float coverage, double stepStart, double step) {
    const float before = covered;
    covered = coverage;
    // When, within the step, did the coverage pass `threshold`? It changes
    // evenly within one step, since items move evenly.
    const auto crossing = [&](float threshold) {
        const float fraction = coverage == before ? 1.0F : (threshold - before) / (coverage - before);
        return stepStart + step * std::clamp(static_cast<double>(fraction), 0.0, 1.0);
    };
    if (!beam && coverage > spec.onFraction) {
        beam = true;
        const double beamTime = crossing(spec.onFraction);
        pending.push_back(Edge{beamTime + spec.responseTime, true});
        return beamTime;
    }
    if (beam && coverage < spec.offFraction) {
        beam = false;
        pending.push_back(Edge{crossing(spec.offFraction) + spec.responseTime, false});
    }
    return std::nullopt;
}

std::optional<Edge> Photocell::nextOutput(double time) {
    if (pending.empty() || pending.front().time > time + 1e-12) {
        return std::nullopt;
    }
    const auto edge = pending.front();
    pending.pop_front();
    output.set(edge.time, edge.value);
    return edge;
}

double EncoderSpec::pulsesPerMetre() const {
    return static_cast<double>(pulsesPerRevolution) / (std::numbers::pi * static_cast<double>(rollerDiameter));
}

} // namespace fsim
