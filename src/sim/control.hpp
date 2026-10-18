#pragma once
#include "sim/io.hpp"
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

// Control: what decides when the actuators move.
//
// A control program sees the line only through the I/O image, the way a PLC
// program does: inputs (sensors, buttons, the encoder's count) come in as
// bits and words, and it writes outputs (valves, motors, lamps). Every scan,
// the World reads the devices into the image, runs the program, and applies
// the outputs to the devices. So anything that reads and writes the image
// can control the line: the built-in sorter here, soft_plc in milestone 6.
namespace fsim {

// The operator's control panel: buttons the program reads, lamps it lights.
struct ButtonSpec {
    std::string name = "Start";
    std::string address = "%IX0.0";
    // A normally closed button (stop, emergency stop) gives TRUE while it's
    // NOT pressed, so a broken wire stops the machine, as on a real one.
    bool normallyClosed = false;
    bool latching = false; // stays pressed until pressed again (a switch), instead of only while held

    bool operator==(const ButtonSpec&) const = default;
};

// A potentiometer on the panel: an analog input word, 0 to 27648 as a
// Siemens analog input module gives 0 to 10 V (the nominal range).
struct PotentiometerSpec {
    std::string name = "Speed";
    std::string address = "%IW8";

    bool operator==(const PotentiometerSpec&) const = default;
};

struct LampSpec {
    std::string name = "Running";
    std::string address = "%QX0.0";

    bool operator==(const LampSpec&) const = default;
};

// The built-in sorter: when the NOK sensor sees a box, it notes the encoder
// count; when the belt has moved `distance` pulses further, it fires the
// pusher; when the pusher is fully out, it pulls it back. Like a PLC
// program, it knows only addresses.
struct SorterSpec {
    std::string name = "Sorter";
    std::string nokInput = "%IX0.1";      // the sensor that sees NOK boxes
    std::string encoderInput = "%ID8";    // the conveyor's encoder count
    std::string valveOutput = "%QX0.0";   // the pusher's valve
    std::string extendedInput = "%IX0.2"; // the pusher's "extended" sensor
    std::int64_t distance = 5000;         // encoder pulses from the sensor to the point where it fires

    bool operator==(const SorterSpec&) const = default;
};

// How the built-in camera triggering delays a camera's trigger after the
// sensor sees an item.
enum class DelayMethod {
    // One on-delay timer per camera, as a quick PLC program would have it:
    // the sensor's edge starts it, its end fires the trigger. While it runs,
    // further edges are lost: a second item that arrives before the timer
    // ends gets no trigger at all.
    Timer,
    // A queue of trigger times, one per item (like the sorter's queue of
    // encoder counts): every item gets its own trigger, however close.
    PerItem,
};

struct CameraTriggerSpec {
    std::string name = "K1";
    std::string sensorInput = "%IX0.0";   // the photocell that sees the items
    std::string triggerOutput = "%QX0.0"; // the camera's trigger line
    double delay = 0.0;                   // s, from the sensor's edge to the trigger
    DelayMethod method = DelayMethod::Timer;

    bool operator==(const CameraTriggerSpec&) const = default;
};

// Anything that controls the line by reading and writing the I/O image.
class ControlProgram {
public:
    virtual ~ControlProgram() = default;
    // One scan: inputs are in `io`, at simulated time `time`; write the outputs.
    virtual void scan(IoImage& io, double time) = 0;
    virtual std::string name() const = 0;
};

// The built-in sorters, one per SorterSpec, run as one program.
class BuiltInSorting final : public ControlProgram {
public:
    explicit BuiltInSorting(std::vector<SorterSpec> sorters);
    void scan(IoImage& io, double time) override;
    std::string name() const override { return "built-in sorting"; }

    // For the panel: how many boxes each sorter has fired at, and how many it's tracking.
    struct Status {
        std::size_t fired = 0;
        std::size_t waiting = 0; // NOK boxes seen, not yet at the pusher
    };
    std::vector<Status> status() const;

private:
    struct Sorter {
        SorterSpec spec;
        bool lastNok = false;
        bool valve = false;
        std::deque<std::uint32_t> firingCounts; // where each tracked box should be pushed
        std::size_t fired = 0;
    };
    std::vector<Sorter> sorters_;
};

// The built-in camera triggering: one photocell, any number of cameras,
// each triggered a set time after the photocell sees an item. A trigger is
// a pulse one scan long; a camera takes a picture on its rising edge, so
// two triggers need at least one scan with the output off between them.
class CameraTriggering final : public ControlProgram {
public:
    explicit CameraTriggering(std::vector<CameraTriggerSpec> triggers);
    void scan(IoImage& io, double time) override;
    std::string name() const override { return "built-in camera triggering"; }

    struct Status {
        std::size_t fired = 0;   // triggers sent
        std::size_t lost = 0;    // sensor edges that got no trigger (the timer was still running)
        std::size_t waiting = 0; // items seen, trigger not sent yet
    };
    std::vector<Status> status() const;

private:
    struct Trigger {
        CameraTriggerSpec spec;
        bool lastSensor = false;
        bool output = false;
        bool timing = false; // the timer is running (Timer)
        double started = 0.0;
        std::deque<double> due; // when each item's trigger is due (PerItem)
        Status status;
    };
    std::vector<Trigger> triggers_;
};

} // namespace fsim
