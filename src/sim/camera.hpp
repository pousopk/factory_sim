#pragma once
#include "sim/sensors.hpp"
#include "sim/signal.hpp"
#include <cstddef>
#include <deque>
#include <string>

// An industrial camera looking down at a conveyor, triggered by the PLC.
//
// A triggered camera works through a fixed sequence, and while it does, it
// can't take another picture:
//
//   trigger ─┐
//            ├ trigger delay ┤ exposure ├──── readout ────┤
//            │◄─────────────────── busy ─────────────────►│
//
// - the **trigger delay** is the camera's own (GenICam's TriggerDelay):
//   it waits that long after the trigger before exposing;
// - the **exposure** is when the sensor collects light: whatever moves
//   during it is smeared (motion blur);
// - the **readout** is getting the picture off the sensor and out over the
//   network (a GigE camera sends megabytes per frame).
//
// A trigger that arrives while the camera is busy is **ignored**: no picture,
// and no error either, unless something checks. That's the tin line's story:
// two tins too close together, and a camera still busy with the first one.
namespace fsim {

struct CameraSpec {
    std::string name = "K1";
    std::size_t conveyor = 0;
    float along = 1.5F;        // the middle of its field of view, metres from the belt's start
    float fieldOfView = 0.20F; // how much of the belt it sees, along the travel, m
    int pixels = 1280;         // across the field of view, along the travel
    float tilt = 0.0F;         // where it looks from, turned around the travel: 0 straight down, + from the left (rad)
    double triggerDelay = 0.0; // s, from the trigger to the exposure starting
    double exposure = 0.0005;  // s
    double readout = 0.040;    // s, after the exposure, before it can take the next one
    std::string triggerOutput; // the PLC output wired to its trigger line, like "%QX0.0": a rising edge triggers
    std::string readyInput;    // the PLC input its "ready for a trigger" line is wired to (empty: none)

    // How long one picture keeps it busy.
    double busyTime() const { return triggerDelay + exposure + readout; }

    bool operator==(const CameraSpec&) const = default;
};

// The camera's timing alone: when it accepts a trigger and when it's free
// again. No picture, no physics: so a study of trigger timing can run it on
// its own, thousands of triggers in a moment (see analysis/tin_study.hpp).
class CameraTiming {
public:
    // A trigger at `time`: true if the camera takes it, false if it's busy.
    bool trigger(double time, double busyTime);
    bool busyAt(double time) const { return time < busyUntil_; }
    double busyUntil() const { return busyUntil_; }

private:
    double busyUntil_ = -1.0;
};

// One trigger and what came of it.
struct Shot {
    double triggerTime = 0.0;
    bool taken = false;    // false: the camera was busy and ignored the trigger
    double exposure = 0.0; // the middle of the exposure (or when it would have been)
    // The item in the field of view then (0: none), and where: its centre's
    // distance from the middle of the picture along the travel, m (positive:
    // past the middle). For an ignored trigger, the item it would have seen.
    ItemId item = 0;
    float offset = 0.0F;
    bool whole = false; // the whole item was inside the field of view
    float blur = 0.0F;  // how far it moved during the exposure, m
    // Where the item was at the exposure, to draw it there.
    glm::vec3 itemCenter{0.0F};
    glm::quat itemRotation{1.0F, 0.0F, 0.0F, 0.0F};
    glm::vec3 itemSize{0.0F};
    Shape itemShape = Shape::Box;
};

struct Camera {
    CameraSpec spec;
    CameraTiming timing;
    Signal trigger;         // the trigger line, as the PLC drives it
    Signal busy;            // busy with a picture: the inverse of its "ready" line
    std::deque<Shot> shots; // the most recent first, up to kShotHistory
    static constexpr std::size_t kShotHistory = 400;

    // Shots whose exposure hasn't happened yet: what's in the picture is
    // decided when it does. Oldest first.
    std::deque<Shot> pending;
};

} // namespace fsim
