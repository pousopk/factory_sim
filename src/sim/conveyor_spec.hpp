#pragma once
#include "sim/drive.hpp"
#include "sim/sensors.hpp"
#include <glm/glm.hpp>
#include <string>

// A conveyor's description, and the geometry worked out from it. On its own
// so that everything placed on a conveyor (sensors, actuators, bins) can use
// it without the whole World.
//
// Units everywhere: metres, seconds, kilograms. Y is up; the floor is y = 0.
namespace fsim {

struct ConveyorSpec {
    std::string name = "Conveyor";
    glm::vec3 start{0.0F}; // on the floor, under the start of the belt's centre line
    float heading = 0.0F;  // direction of travel, radians around Y: 0 = +X, pi/2 = -Z
    float length = 3.0F;
    float width = 0.4F;
    float beltHeight = 0.8F;   // height of the belt surface above the floor
    float targetSpeed = 0.5F;  // m/s; the belt ramps towards it
    float acceleration = 1.0F; // m/s², like a VFD's ramp time (0 to 1 m/s in 1 s)
    EncoderSpec encoder;       // on the drive roller
    // Side guides: rails along the belt's edges that keep boxes on it.
    bool guideLeft = false;
    bool guideRight = false;
    // I/O: an output that runs the motor (empty: it always runs), and the
    // input double word its encoder's count is written to (empty: none).
    std::string runOutput;
    std::string encoderInput;
    // A drive on the bus instead (PROFIdrive telegram 1): the PLC sets the
    // speed, not just on and off. With one, `targetSpeed` is only used
    // without a control program.
    DriveSpec drive;

    bool operator==(const ConveyorSpec&) const = default;
};

// The unit vector of travel for a heading.
glm::vec3 directionOf(float heading);
// The unit vector to the left of the travel (seen from above, looking along it).
glm::vec3 leftOf(glm::vec3 direction);
// The point on the belt's surface, `along` metres from its start, on its centre line.
glm::vec3 beltPoint(const ConveyorSpec& spec, float along);

} // namespace fsim
