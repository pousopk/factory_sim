#pragma once
#include "sim/drive.hpp"
#include "sim/sensors.hpp"
#include <glm/glm.hpp>
#include <string>
#include <vector>

// A conveyor's description, and the geometry worked out from it. On its own
// so that everything placed on a conveyor (sensors, actuators, bins) can use
// it without the whole World.
//
// Units everywhere: metres, seconds, kilograms. Y is up; the floor is y = 0.
namespace fsim {

// How an actuator moves (see Stroke in actuators.hpp): a dead time after
// the command, then a steady travel. (Here because a conveyor's carriage
// moves the same way.)
struct StrokeSpec {
    double deadTime = 0.015;   // s, from the valve switching to the movement starting
    double extendTime = 0.12;  // s, to travel the whole stroke out
    double retractTime = 0.12; // s, and back

    bool operator==(const StrokeSpec&) const = default;
};

// One zone of an accumulation conveyor: its photo-eye, near the zone's end,
// and its motor.
struct ZoneSpec {
    std::string eyeInput;    // %IX: the zone's photo-eye, to the PLC
    std::string motorOutput; // %QX: the zone's motor, when the PLC runs the zones

    bool operator==(const ZoneSpec&) const = default;
};

// Who runs an accumulation conveyor's zones: the zone cards themselves
// (zero-pressure accumulation, as motor-driven-roller conveyors do it), or
// the PLC, one output per zone.
enum class ZoneLogic { Local, Plc };

// What a zone card knows about its zone, in local logic: empty, a box on its
// way in from the zone before, or full (a box at its eye).
enum class ZoneState { Empty, Arriving, Full };

// A conveyor section on a moving carriage: a turntable turns it about its
// middle, a lift raises it to another level.
enum class Carriage { None, Turntable, Lift };

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

    // Accumulation: more than one zone, each with its own motor and eye. In
    // local logic a zone runs while it's empty or the next one can take its
    // item, and the last one discharges while `releaseOutput` is on (empty:
    // always), so items queue with gaps between them instead of pushing.
    int zones = 1;
    ZoneLogic zoneLogic = ZoneLogic::Local;
    std::vector<ZoneSpec> zoneIo; // one per zone
    std::string releaseOutput;    // %QX

    // A carriage, moved like an air cylinder (or a geared motor): on its
    // output, from home to the end, with a sensor at each.
    Carriage carriage = Carriage::None;
    float turnAngle = 1.5707964F; // turntable: radians, left positive (90 degrees)
    float liftHeight = 0.4F;      // lift: m up
    StrokeSpec carriageStroke{0.05, 1.5, 1.5};
    std::string carriageOutput;    // %QX: towards the end (turned, up)
    std::string carriageEndInput;  // %IX: at the end
    std::string carriageHomeInput; // %IX: at home

    bool operator==(const ConveyorSpec&) const = default;
};

// The unit vector of travel for a heading.
glm::vec3 directionOf(float heading);
// The unit vector to the left of the travel (seen from above, looking along it).
glm::vec3 leftOf(glm::vec3 direction);
// The point on the belt's surface, `along` metres from its start, on its centre line.
glm::vec3 beltPoint(const ConveyorSpec& spec, float along);

// Where a conveyor on a carriage is when its carriage is at `position` (0
// home, 1 the end): turned about its middle, or raised.
ConveyorSpec carried(const ConveyorSpec& home, double position);

// The zone of an accumulation conveyor under a point `along` metres from
// its start (0 for a conveyor without zones).
int zoneAt(const ConveyorSpec& spec, float along);

} // namespace fsim
