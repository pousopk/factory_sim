#include "sim/mounting.hpp"
#include <cmath>

namespace fsim {

Frame Frame::inverse() const {
    const auto back = glm::conjugate(rotation);
    return {back * -origin, back};
}

Frame beltFrame(const ConveyorSpec& belt, float along) {
    return {beltPoint(belt, along), glm::angleAxis(belt.heading, glm::vec3(0.0F, 1.0F, 0.0F))};
}

ConveyorSpec standInBelt(float span, float height, float along) {
    ConveyorSpec belt;
    belt.name = "(on its own)";
    belt.start = {-along, 0.0F, 0.0F};
    belt.heading = 0.0F;
    belt.beltHeight = height;
    belt.width = span;
    belt.length = along + 1.0F;
    return belt;
}

Mounting mounting(const Placement& placement, const ConveyorSpec* conveyor, float along) {
    Mounting m;
    const Frame nudge{placement.offset, placement.turn};
    if (placement.parent == Parent::Conveyor && conveyor != nullptr) {
        m.belt = *conveyor;
        if (placement.asUsual()) {
            return m;
        }
        // From the belt's frame, nudged, and back: a device's own shape is
        // in world coordinates already, where it would usually be.
        const auto belt = beltFrame(*conveyor, along);
        m.transform = belt * nudge * belt.inverse();
    } else {
        // On a stand-in belt, from its frame at `along` to where it stands.
        m.belt = standInBelt(placement.span, placement.height, along);
        m.transform = nudge * beltFrame(m.belt, along).inverse();
    }
    m.identity = false;
    return m;
}

Frame placedFrame(const Placement& placement, const ConveyorSpec* conveyor, float along) {
    const Frame nudge{placement.offset, placement.turn};
    if (placement.parent == Parent::Conveyor && conveyor != nullptr) {
        return beltFrame(*conveyor, along) * nudge;
    }
    return nudge;
}

Placement placementAt(const Frame& frame, Parent parent, const ConveyorSpec* conveyor, float along, float span,
                      float height) {
    Placement p;
    p.parent = parent;
    p.span = span;
    p.height = height;
    if (parent == Parent::World || conveyor == nullptr) {
        p.parent = Parent::World;
        p.offset = frame.origin;
        p.turn = glm::normalize(frame.rotation);
        return p;
    }
    const auto nudge = beltFrame(*conveyor, along).inverse() * frame;
    p.offset = nudge.origin;
    p.turn = glm::normalize(nudge.rotation);
    // What's left of rounding is no nudge: the device is as usual again, and
    // works exactly as a device on its belt does.
    for (int i = 0; i < 3; ++i) {
        if (std::abs(p.offset[i]) < 5e-5F) {
            p.offset[i] = 0.0F;
        }
    }
    if (std::abs(p.turn.w) > std::cos(0.5F * 1.7e-4F)) { // within 0.01 degrees of no turn
        p.turn = glm::quat(1.0F, 0.0F, 0.0F, 0.0F);
    }
    return p;
}

} // namespace fsim
