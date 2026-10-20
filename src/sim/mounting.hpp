#pragma once
#include "sim/conveyor_spec.hpp"
#include "sim/placement.hpp"

// A placement (placement.hpp) against a real or stand-in belt: the frames
// and transforms that put a device where its placement says.
namespace fsim {

// The belt's frame at `along` (see above).
Frame beltFrame(const ConveyorSpec& belt, float along);

// How a device is mounted: the belt its own shape is worked out on (its
// conveyor, or a stand-in), and the transform from there to where it is.
struct Mounting {
    ConveyorSpec belt;
    Frame transform;
    bool identity = true; // attached as usual: nothing to move

    glm::vec3 point(glm::vec3 p) const { return identity ? p : transform.point(p); }
    glm::vec3 direction(glm::vec3 d) const { return identity ? d : transform.direction(d); }
    glm::quat rotation(glm::quat q) const { return identity ? q : transform.rotation * q; }
};

// `conveyor`: the device's conveyor, or null if it has none (it's then
// worked out on the stand-in belt, as one standing on its own).
Mounting mounting(const Placement& placement, const ConveyorSpec* conveyor, float along);

// The stand-in belt for a device standing on its own: `span` wide and
// `height` high, along +x, its frame at `along` straight above the origin.
ConveyorSpec standInBelt(float span, float height, float along);

// Where a device's usual place (its belt's frame at `along`, nudged) is in
// the world: the frame the gizmo shows and moves.
Frame placedFrame(const Placement& placement, const ConveyorSpec* conveyor, float along);

// The placement that puts a device's usual place at `frame` in the world:
// standing on its own there, or (if `conveyor` is given) nudged from its
// place along that belt. A nudge too small to matter (under 0.05 mm, or a
// hundredth of a degree) is no nudge at all: the device is as usual again.
// `span` and `height`: the stand-in belt's, for one standing on its own.
Placement placementAt(const Frame& frame, Parent parent, const ConveyorSpec* conveyor, float along, float span,
                      float height);

} // namespace fsim
