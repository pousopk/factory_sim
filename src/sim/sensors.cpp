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

double EncoderSpec::pulsesPerMetre() const {
    return static_cast<double>(pulsesPerRevolution) / (std::numbers::pi * static_cast<double>(rollerDiameter));
}

} // namespace fsim
