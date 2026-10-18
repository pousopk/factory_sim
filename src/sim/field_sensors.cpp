#include "sim/field_sensors.hpp"
#include "sim/io.hpp"
#include <algorithm>
#include <cmath>

namespace fsim {

float rangeFactor(const ProximitySpec& spec, Shape shape) {
    const bool metal = shape == Shape::Tin;
    if (metal) {
        return 1.0F;
    }
    return spec.inductive ? 0.0F : 0.4F; // cardboard: invisible to a coil, faint to a capacitor
}

std::uint16_t laserValue(const LaserSpec& spec, float distance) {
    if (distance < spec.rangeMin || distance > spec.rangeMax || spec.rangeMax <= spec.rangeMin) {
        return kAnalogOverflow;
    }
    const float fraction = (distance - spec.rangeMin) / (spec.rangeMax - spec.rangeMin);
    return static_cast<std::uint16_t>(std::lround(fraction * kAnalogFullScale));
}

} // namespace fsim
