#include "sim/air_jet.hpp"
#include <algorithm>
#include <cmath>

namespace fsim {

namespace {

constexpr float kPi = 3.14159265F;
constexpr float kOutside = 0.035F; // the nozzles' tips, outside the belt's edge (past a guide)

} // namespace

float thrustOf(const AirJetSpec& spec) {
    const float area = kPi * spec.bore * spec.bore / 4.0F;
    return 1.25F * std::max(spec.pressure, 0.0F) * 1e5F * area;
}

std::vector<Nozzle> nozzlesOf(const AirJetSpec& spec, const ConveyorSpec& conveyor) {
    const auto along = directionOf(conveyor.heading);
    const auto left = leftOf(along);
    const float side = spec.side == Side::Left ? 1.0F : -1.0F;
    const glm::vec3 up{0.0F, 1.0F, 0.0F};
    const auto across = -left * side; // towards the far side
    const auto direction = glm::normalize(across * std::cos(spec.aim) + up * std::sin(spec.aim));
    std::vector<Nozzle> nozzles;
    const int count = std::max(spec.nozzles, 1);
    for (int k = 0; k < count; ++k) {
        const float offset = (static_cast<float>(k) - static_cast<float>(count - 1) / 2.0F) * spec.pitch;
        const auto position = beltPoint(conveyor, spec.along + offset) + up * spec.height +
                              left * side * (conveyor.width / 2.0F + kOutside);
        nozzles.push_back(Nozzle{position, direction});
    }
    return nozzles;
}

std::vector<glm::vec3> streamsAround(glm::vec3 axis, float spread) {
    // Two directions square to the axis, and to each other.
    const auto helper = std::abs(axis.y) < 0.9F ? glm::vec3(0.0F, 1.0F, 0.0F) : glm::vec3(1.0F, 0.0F, 0.0F);
    const auto u = glm::normalize(glm::cross(axis, helper));
    const auto v = glm::cross(axis, u);
    std::vector<glm::vec3> streams{axis};
    constexpr int kRings = 3;
    for (int ring = 1; ring <= kRings; ++ring) {
        // Each ring at the middle of its band of the cone, with as many
        // streams as its circumference holds: an even spread over the area.
        const float angle = spread * (static_cast<float>(ring) - 0.5F) / (kRings - 0.5F);
        const int count = 6 * ring;
        for (int i = 0; i < count; ++i) {
            const float a =
                2.0F * kPi * (static_cast<float>(i) + 0.5F * static_cast<float>(ring % 2)) / static_cast<float>(count);
            const auto sideways = u * std::cos(a) + v * std::sin(a);
            streams.push_back(glm::normalize(axis * std::cos(angle) + sideways * std::sin(angle)));
        }
    }
    return streams;
}

AirJet::AirJet(AirJetSpec settings) : spec(std::move(settings)) {}

void AirJet::command(double time, bool open) {
    closeAt_ = -1.0; // the program decides now, not a blow by hand
    if (open == open_) {
        return;
    }
    open_ = open;
    valve_.set(time, open);
    // The air changes after the valve's delay. A short pulse can end before
    // the air has even started; kept in time order, they cancel out.
    const Edge change{time + (open ? spec.openTime : spec.closeTime), open};
    const auto at = std::upper_bound(pending_.begin(), pending_.end(), change.time,
                                     [](double t, const Edge& e) { return t < e.time; });
    pending_.insert(at, change);
}

void AirJet::blow(double time, double seconds) {
    command(time, true);
    closeAt_ = time + seconds;
}

double AirJet::update(double stepStart, double stepEnd) {
    if (closeAt_ >= 0.0 && closeAt_ <= stepEnd) {
        const double at = std::max(closeAt_, stepStart);
        command(at, false);
    }
    // The time the air flowed in this step: from each change to the next.
    double flowed = 0.0;
    double from = stepStart;
    bool on = air_.value();
    while (!pending_.empty() && pending_.front().time <= stepEnd) {
        const auto change = pending_.front();
        pending_.pop_front();
        const double at = std::max(change.time, stepStart);
        if (on) {
            flowed += at - from;
        }
        from = at;
        on = change.value;
        air_.set(at, on);
    }
    if (on) {
        flowed += stepEnd - from;
    }
    return std::clamp(flowed / (stepEnd - stepStart), 0.0, 1.0);
}

} // namespace fsim
