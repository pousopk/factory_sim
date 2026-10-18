#include "sim/control.hpp"

namespace fsim {

BuiltInSorting::BuiltInSorting(std::vector<SorterSpec> sorters) {
    for (auto& spec : sorters) {
        sorters_.push_back(Sorter{std::move(spec), false, false, {}, 0});
    }
}

// The same logic a PLC program would have, scan by scan:
//
//   NOK sensor's rising edge   ->  remember "fire at count + distance"
//   count has reached the first remembered value  ->  valve on
//   valve on and pusher extended                  ->  valve off
//
// The encoder count is a 32-bit counter that wraps around, so "has it
// reached" is decided on the difference, read as signed: it works across
// the wrap, as long as a box is never more than 2^31 pulses away.
void BuiltInSorting::scan(IoImage& io, double /*time*/) {
    for (auto& sorter : sorters_) {
        const auto nokAddress = parseAddress(sorter.spec.nokInput);
        const auto countAddress = parseAddress(sorter.spec.encoderInput);
        const auto valveAddress = parseAddress(sorter.spec.valveOutput);
        const auto extendedAddress = parseAddress(sorter.spec.extendedInput);
        if (!nokAddress || !countAddress || !valveAddress || !extendedAddress) {
            continue; // not set up: does nothing, as a PLC program with no I/O would
        }
        const bool nok = io.bit(*nokAddress);
        const auto count = io.value(*countAddress);
        if (nok && !sorter.lastNok) {
            sorter.firingCounts.push_back(count + static_cast<std::uint32_t>(sorter.spec.distance));
        }
        sorter.lastNok = nok;
        if (!sorter.firingCounts.empty() && static_cast<std::int32_t>(count - sorter.firingCounts.front()) >= 0) {
            sorter.firingCounts.pop_front();
            sorter.valve = true;
            ++sorter.fired;
        }
        if (sorter.valve && io.bit(*extendedAddress)) {
            sorter.valve = false;
        }
        io.setBit(*valveAddress, sorter.valve);
    }
}

std::vector<BuiltInSorting::Status> BuiltInSorting::status() const {
    std::vector<Status> result;
    for (const auto& sorter : sorters_) {
        result.push_back(Status{sorter.fired, sorter.firingCounts.size()});
    }
    return result;
}

CameraTriggering::CameraTriggering(std::vector<CameraTriggerSpec> triggers) {
    for (auto& spec : triggers) {
        Trigger trigger;
        trigger.spec = std::move(spec);
        triggers_.push_back(std::move(trigger));
    }
}

// Scan by scan, for each camera:
//
//   Timer:    sensor's rising edge, timer idle  ->  start the timer
//             (an edge while it runs is lost)
//             timer elapsed                     ->  trigger
//   PerItem:  sensor's rising edge              ->  queue "trigger at now + delay"
//             the first queued time has come    ->  trigger
//
// A trigger is the output on for one scan. If it was on in the last scan,
// the trigger waits a scan: the camera needs an edge, not a level.
void CameraTriggering::scan(IoImage& io, double time) {
    // A timer "has elapsed" at the first scan at or after start + delay; the
    // tolerance keeps 0.1 + 0.05 from missing 0.15 by a rounding error.
    constexpr double kTolerance = 1e-9;
    for (auto& t : triggers_) {
        const auto sensorAddress = parseAddress(t.spec.sensorInput);
        const auto outputAddress = parseAddress(t.spec.triggerOutput);
        if (!sensorAddress || !outputAddress) {
            continue;
        }
        const bool sensor = io.bit(*sensorAddress);
        const bool edge = sensor && !t.lastSensor;
        t.lastSensor = sensor;
        bool due = false;
        if (t.spec.method == DelayMethod::Timer) {
            if (edge) {
                if (t.timing) {
                    ++t.status.lost; // the timer is busy with the last item
                } else {
                    t.timing = true;
                    t.started = time;
                }
            }
            due = t.timing && time + kTolerance >= t.started + t.spec.delay;
        } else {
            if (edge) {
                t.due.push_back(time + t.spec.delay);
            }
            due = !t.due.empty() && time + kTolerance >= t.due.front();
        }
        const bool fire = due && !t.output;
        if (fire) {
            ++t.status.fired;
            if (t.spec.method == DelayMethod::Timer) {
                t.timing = false;
            } else {
                t.due.pop_front();
            }
        }
        t.output = fire;
        t.status.waiting = t.spec.method == DelayMethod::Timer ? (t.timing ? 1 : 0) : t.due.size();
        io.setBit(*outputAddress, fire);
    }
}

std::vector<CameraTriggering::Status> CameraTriggering::status() const {
    std::vector<Status> result;
    for (const auto& t : triggers_) {
        result.push_back(t.status);
    }
    return result;
}

} // namespace fsim
