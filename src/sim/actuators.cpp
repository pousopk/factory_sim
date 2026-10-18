#include "sim/actuators.hpp"
#include <algorithm>
#include <cmath>

namespace fsim {

// ---------------------------------------------------------------- Stroke

Stroke::Stroke() : Stroke(StrokeSpec{}) {}

Stroke::Stroke(StrokeSpec s) : spec(s) {
    retracted_.set(0.0, true);
}

void Stroke::command(double time, bool extend) {
    if (extend == extend_) {
        return; // the valve is already there
    }
    extend_ = extend;
    commandTime_ = time;
    valve_.set(time, extend);
}

void Stroke::update(double stepStart, double stepEnd) {
    const double target = extend_ ? 1.0 : 0.0;
    if (position_ == target || commandTime_ < 0.0) {
        return;
    }
    // It starts moving after the dead time, possibly in the middle of this step.
    const double moving = std::max(stepStart, commandTime_ + spec.deadTime);
    if (moving >= stepEnd) {
        return;
    }
    const double speed = 1.0 / std::max(extend_ ? spec.extendTime : spec.retractTime, 1e-6); // strokes per second
    // Leaving an end position: that sensor switches off the moment it moves.
    if (extend_ && position_ == 0.0) {
        retracted_.set(moving, false);
    } else if (!extend_ && position_ == 1.0) {
        extended_.set(moving, false);
    }
    const double distance = std::abs(target - position_);
    const double travel = speed * (stepEnd - moving);
    // (A hair of tolerance: after many steps of float arithmetic the
    // remaining distance can be 1e-16 instead of 0, and then it would never
    // quite arrive and the end sensor would never switch.)
    if (travel >= distance - 1e-9) {
        // It arrives within this step: at exactly this time.
        const double arrival = moving + distance / speed;
        position_ = target;
        (extend_ ? extended_ : retracted_).set(arrival, true);
    } else {
        position_ += extend_ ? travel : -travel;
    }
}

// ---------------------------------------------------------------- Poses

namespace {

constexpr float kPaddleThickness = 0.02F;
constexpr float kPaddleHeight = 0.10F;
constexpr float kArmThickness = 0.02F;
constexpr float kArmHeight = 0.08F;
constexpr float kClearance = 0.005F; // between moving parts and the belt
constexpr float kFrameWidth = 0.04F; // the conveyor's side frames (as drawn)
constexpr float kWall = 0.02F;       // a bin's walls

glm::vec3 up() {
    return {0.0F, 1.0F, 0.0F};
}

// The unit vector from the belt's centre line towards `side`.
glm::vec3 towards(Side side, const ConveyorSpec& conveyor) {
    const auto left = leftOf(directionOf(conveyor.heading));
    return side == Side::Left ? left : -left;
}

glm::quat turnedBy(float heading) {
    return glm::angleAxis(heading, up());
}

} // namespace

Pose pusherPaddle(const PusherSpec& spec, const ConveyorSpec& conveyor, double position) {
    // Retracted, the paddle's face is just outside the belt's edge; extended,
    // it has crossed the whole belt and a bit more, so a box goes right off.
    const auto out = towards(spec.side, conveyor);
    const float retracted = conveyor.width / 2.0F + kPaddleThickness / 2.0F + 0.01F;
    const float stroke = conveyor.width + 0.04F;
    const float across = retracted - stroke * static_cast<float>(position);
    return Pose{beltPoint(conveyor, spec.along) + out * across + up() * (kClearance + kPaddleHeight / 2.0F),
                turnedBy(conveyor.heading),
                {spec.paddleLength, kPaddleHeight, kPaddleThickness}};
}

Pose diverterArm(const DiverterSpec& spec, const ConveyorSpec& conveyor, double position) {
    // The arm lies along the belt's edge, pivoted at its upstream end; it
    // swings out over the belt, its tip moving downstream and across, long
    // enough to reach the far edge at its full angle.
    const auto out = towards(spec.side, conveyor);
    const float reach = (conveyor.width + 0.02F) / std::max(std::sin(spec.angle), 0.1F);
    const float length = std::min(reach, 1.5F);
    const auto pivot = beltPoint(conveyor, spec.along) + out * (conveyor.width / 2.0F + kArmThickness / 2.0F + 0.005F);
    // Swinging towards the belt is clockwise (seen from above) for a pivot on
    // the left, anticlockwise for one on the right.
    const float swing = static_cast<float>(position) * spec.angle * (spec.side == Side::Left ? -1.0F : 1.0F);
    const float heading = conveyor.heading + swing;
    const auto center = pivot + directionOf(heading) * (length / 2.0F) + up() * (kClearance + kArmHeight / 2.0F);
    return Pose{center, turnedBy(heading), {length, kArmHeight, kArmThickness}};
}

Pose stopperBlade(const StopperSpec& spec, const ConveyorSpec& conveyor, double position) {
    // Retracted, the blade's top is just below the belt's surface; extended,
    // it stands `height` above it. It's taller than that, so part of it
    // always stays below, as a real one does in its slot.
    const float blade = spec.height + 0.03F;
    const float top = -0.002F + (spec.height + 0.002F) * static_cast<float>(position);
    return Pose{beltPoint(conveyor, spec.along) + up() * (top - blade / 2.0F),
                turnedBy(conveyor.heading),
                {kPaddleThickness, blade, conveyor.width}};
}

std::vector<Pose> binWalls(const BinSpec& spec, const ConveyorSpec& conveyor) {
    const auto inside = binInside(spec, conveyor);
    const auto rotation = turnedBy(conveyor.heading);
    const auto along = directionOf(conveyor.heading);
    const auto out = towards(spec.side, conveyor);
    const auto& s = spec.size;
    const glm::vec3 floorCenter{inside.center.x, kWall / 2.0F, inside.center.z};
    const glm::vec3 wallMiddle{inside.center.x, s.y / 2.0F, inside.center.z};
    // In the conveyor's axes: x along the belt, z across it.
    return {
        Pose{floorCenter, rotation, {s.x, kWall, s.z}},
        Pose{wallMiddle + along * (s.x / 2.0F - kWall / 2.0F), rotation, {kWall, s.y, s.z}},
        Pose{wallMiddle - along * (s.x / 2.0F - kWall / 2.0F), rotation, {kWall, s.y, s.z}},
        Pose{wallMiddle + out * (s.z / 2.0F - kWall / 2.0F), rotation, {s.x, s.y, kWall}},
        Pose{wallMiddle - out * (s.z / 2.0F - kWall / 2.0F), rotation, {s.x, s.y, kWall}},
    };
}

Pose binInside(const BinSpec& spec, const ConveyorSpec& conveyor) {
    // Beside the belt, a little beyond its frame, on the floor.
    const auto out = towards(spec.side, conveyor);
    const float near = conveyor.width / 2.0F + kFrameWidth + 0.03F;
    auto center = conveyor.start + directionOf(conveyor.heading) * spec.along + out * (near + spec.size.z / 2.0F);
    center.y = spec.size.y / 2.0F;
    return Pose{center, turnedBy(conveyor.heading), spec.size - glm::vec3(2.0F * kWall, kWall, 2.0F * kWall)};
}

} // namespace fsim
