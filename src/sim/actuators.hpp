#pragma once
#include "sim/conveyor_spec.hpp"
#include "sim/signal.hpp"
#include <cstddef>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <string>
#include <vector>

// Pneumatic actuators: pushers, diverters and stoppers, and a reject bin.
//
// All three actuators move the same way, the way an air cylinder does. When
// its valve switches, nothing happens for a moment (the air has to flow and
// the pressure build: the dead time); then it travels at a steady speed to
// the end of its stroke. Reed switches on the cylinder tell when it's fully
// extended or fully retracted. That's a Stroke; a pusher turns it into a
// paddle sliding across the belt, a diverter into an arm swinging out, a
// stopper into a blade rising out of the belt.
//
//   valve:      ____/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\______________________
//   position:   ________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\_______________
//                   dead  travel                dead   travel
//   extended:   _________________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\_______________
//   retracted:  ‾‾‾‾‾‾‾‾‾\___________________________________/‾‾‾‾
namespace fsim {

struct StrokeSpec {
    double deadTime = 0.015;   // s, from the valve switching to the movement starting
    double extendTime = 0.12;  // s, to travel the whole stroke out
    double retractTime = 0.12; // s, and back

    bool operator==(const StrokeSpec&) const = default;
};

class Stroke {
public:
    Stroke();
    explicit Stroke(StrokeSpec spec);

    StrokeSpec spec;

    // Switches the valve at `time`: towards extended (true) or retracted.
    void command(double time, bool extend);
    bool commanded() const { return extend_; }

    // Moves through one step, from `stepStart` to `stepEnd`.
    void update(double stepStart, double stepEnd);

    double position() const { return position_; } // 0 = retracted, 1 = extended

    // The cylinder's end position sensors, with exact switching times.
    const Signal& extended() const { return extended_; }
    const Signal& retracted() const { return retracted_; }

private:
    bool extend_ = false;
    double commandTime_ = -1.0;
    double position_ = 0.0;
    Signal extended_;
    Signal retracted_;
};

enum class Side { Left, Right }; // seen looking along the belt's direction of travel

// A pusher on one side of a conveyor; it pushes boxes across, off the other side.
struct PusherSpec {
    std::string name = "P1";
    std::size_t conveyor = 0;
    float along = 2.0F;        // metres from the belt's start, to the paddle's centre
    Side side = Side::Left;    // where it's mounted
    float paddleLength = 0.3F; // along the belt
    // Slow enough out that a pushed box drops into a bin beside the belt
    // instead of flying over it: the stroke time decides where it lands.
    StrokeSpec stroke{0.015, 0.5, 0.3};

    // I/O: the valve's output, and the end position sensors' inputs (empty: not wired).
    std::string valveOutput;
    std::string extendedInput;
    std::string retractedInput;

    bool operator==(const PusherSpec&) const = default;
};

// An arm pivoted at one side of a conveyor; swung out, it guides boxes off the other side.
struct DiverterSpec {
    std::string name = "D1";
    std::size_t conveyor = 0;
    float along = 1.0F;     // metres from the belt's start, to the pivot
    Side side = Side::Left; // where the pivot is
    float angle = 0.7F;     // radians the arm swings out (40 degrees)
    StrokeSpec stroke{0.015, 0.25, 0.25};

    // I/O: the valve's output, and the end position sensors' inputs (empty: not wired).
    std::string valveOutput;
    std::string extendedInput;
    std::string retractedInput;

    bool operator==(const DiverterSpec&) const = default;
};

// A blade that rises out of the belt and holds boxes back.
struct StopperSpec {
    std::string name = "S1";
    std::size_t conveyor = 0;
    float along = 2.5F;   // metres from the belt's start
    float height = 0.05F; // how far above the belt it rises
    StrokeSpec stroke{0.015, 0.06, 0.06};

    // I/O: the valve's output, and the end position sensors' inputs (empty: not wired).
    std::string valveOutput;
    std::string extendedInput;
    std::string retractedInput;

    bool operator==(const StopperSpec&) const = default;
};

// An open box beside a conveyor, on the floor, that counts the boxes landing in it.
struct BinSpec {
    std::string name = "Bin1";
    std::size_t conveyor = 0;
    float along = 2.0F; // metres from the belt's start, to the bin's centre
    Side side = Side::Right;
    glm::vec3 size{0.5F, 0.45F, 0.45F}; // along the belt, height, out from the belt

    bool operator==(const BinSpec&) const = default;
};

// A part's place in the world, as the physics and the drawing both use it.
struct Pose {
    glm::vec3 center;
    glm::quat rotation;
    glm::vec3 size;
};

// Where the moving part of each actuator is, at a stroke position from 0 to 1.
// The physics moves its body there and the view draws it there: one formula,
// so what's drawn is exactly what pushes.
Pose pusherPaddle(const PusherSpec& spec, const ConveyorSpec& conveyor, double position);
Pose diverterArm(const DiverterSpec& spec, const ConveyorSpec& conveyor, double position);
Pose stopperBlade(const StopperSpec& spec, const ConveyorSpec& conveyor, double position);
// The bin's floor and four walls.
std::vector<Pose> binWalls(const BinSpec& spec, const ConveyorSpec& conveyor);
// Where the bin's inside is: its centre and half size, for counting.
Pose binInside(const BinSpec& spec, const ConveyorSpec& conveyor);

// How far the guides (side rails) rise above the belt.
constexpr float kGuideHeight = 0.05F;
constexpr float kGuideThickness = 0.02F;

} // namespace fsim
