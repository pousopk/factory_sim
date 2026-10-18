#pragma once
#include "sim/signal.hpp"
#include <cstddef>
#include <cstdint>
#include <deque>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <string>
#include <utility>
#include <vector>

// Sensors that behave like real ones.
//
// A photocell's light beam crosses the belt at one point. It isn't a line:
// it has a diameter, and the receiver sees how much of the beam is blocked.
// The sensor switches on when more than `onFraction` of the beam is covered
// and off again when less than `offFraction` is. That gap is its
// hysteresis, which stops it chattering when an edge sits right in the
// beam; it also sets the smallest object it can see: one shorter than
// onFraction x the beam's diameter never covers enough.
//
//          beam (diameter d)
//            │ │                 box front edge moving right ──►
//   ─────────┼─┼───────
//   on  when the box covers more than 60 % of d
//   off when it covers less than 40 %
//
// The output follows the beam after the sensor's response time, as a real
// sensor's does (its electronics filter and decide before switching).
namespace fsim {

using ItemId = std::uint64_t;

struct PhotocellSpec {
    std::string name = "B1";
    std::size_t conveyor = 0;
    float along = 1.5F;          // where the beam crosses, metres from the belt's start
    float height = 0.05F;        // the beam's height above the belt surface
    float beamDiameter = 0.008F; // m
    float onFraction = 0.6F;     // covered more than this: on
    float offFraction = 0.4F;    // covered less than this: off
    double responseTime = 0.001; // s, from the beam to the output
    std::string input;           // the PLC input its output is wired to, like "%IX0.0" (empty: none)
    // Sees only NOK boxes: a stand-in for an inspection system (a camera, a
    // colour sensor) whose output says "this one is bad".
    bool onlyNok = false;

    bool operator==(const PhotocellSpec&) const = default;
};

// One time the output switched on: which box, when, and where it was.
struct Trigger {
    ItemId item = 0;
    double beamTime = 0.0;         // when enough of the beam was covered
    double outputTime = 0.0;       // when the output switched on (beamTime + response time)
    float speed = 0.0F;            // the box's speed along the belt then, m/s
    float positionError = 0.0F;    // how far past the beam's centre the box's front edge was at outputTime, m
    std::int64_t encoderCount = 0; // the conveyor's encoder count at outputTime
    // Where the box was at outputTime, to draw it there.
    glm::vec3 itemCenter{0.0F};
    glm::quat itemRotation{1.0F, 0.0F, 0.0F, 0.0F};
    glm::vec3 itemSize{0.0F};
};

struct Photocell {
    PhotocellSpec spec;
    float covered = 0.0F;         // the fraction of the beam covered now
    bool beam = false;            // the beam's state, with hysteresis, before the response time
    Signal output;                // what the sensor's output wire shows
    std::deque<Trigger> triggers; // the most recent first, up to kTriggerHistory
    static constexpr std::size_t kTriggerHistory = 200;

    // Output changes waiting for their response time to pass, and for each
    // rising one, the trigger it will become.
    std::deque<Edge> pending;
    std::deque<Trigger> pendingTriggers;
};

// How much of a beam, centred at `centre` with diameter `diameter`, the
// intervals [from, to] along the travel cover: 0 to 1. Intervals that
// overlap each other count once.
float beamCoverage(float centre, float diameter, std::vector<std::pair<float, float>> intervals);

// A rotary encoder on a conveyor's roller: `pulsesPerRevolution` pulses per
// turn of a roller of `rollerDiameter`, so this many pulses per metre of belt.
struct EncoderSpec {
    int pulsesPerRevolution = 1024;
    float rollerDiameter = 0.05F; // m
    double pulsesPerMetre() const;

    bool operator==(const EncoderSpec&) const = default;
};

} // namespace fsim
