#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

// Where a device is, for every kind of device the same way.
//
// A device has a parent: a conveyor, or the world. On a conveyor it's at its
// place along the belt (its `along`), mounted the way that kind of device
// usually is (a pusher beside the belt, a camera above it), and it moves
// with the conveyor. A nudge (an offset and a turn, in the belt's frame
// there) can put it anywhere from there: raised, set back, turned. In the
// world it stands on its own, where its offset and turn put it.
//
// A device's own shape is worked out as it always was, for a device on a
// belt, and then moved by its placement's transform. So every kind of
// device places the same way, and one attached as usual (no nudge) is
// exactly where it always was. A device standing on its own is worked out
// as if on a stand-in belt as wide as `span` and `height` above the floor
// (a pusher's stroke reaches across that width, a photocell's beam spans
// it, a bin stands on the floor below it), whose frame at its place is
// where its offset and turn say.
//
// The belt's frame at a place along it: the origin on the belt's centre
// line, at its surface; x along the travel, y up, z to the right.
namespace fsim {

enum class Parent { Conveyor, World };

struct Placement {
    Parent parent = Parent::Conveyor;
    glm::vec3 offset{0.0F};                 // on a conveyor: the nudge, in the belt's frame; in the world: where it is
    glm::quat turn{1.0F, 0.0F, 0.0F, 0.0F}; // the same, its turn
    float span = 0.4F;                      // in the world: the width of the belt it's worked out for
    float height = 0.8F;                    // and that belt's height above the floor

    bool asUsual() const {
        return parent == Parent::Conveyor && offset == glm::vec3(0.0F) && turn == glm::quat(1.0F, 0.0F, 0.0F, 0.0F);
    }
    bool operator==(const Placement&) const = default;
};

// A rigid transform: a rotation, then a translation.
struct Frame {
    glm::vec3 origin{0.0F};
    glm::quat rotation{1.0F, 0.0F, 0.0F, 0.0F};

    glm::vec3 point(glm::vec3 local) const { return origin + rotation * local; }
    glm::vec3 direction(glm::vec3 local) const { return rotation * local; }
    Frame operator*(const Frame& inner) const { return {point(inner.origin), rotation * inner.rotation}; }
    Frame inverse() const;
};

} // namespace fsim
