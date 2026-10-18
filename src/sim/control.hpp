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

} // namespace fsim
