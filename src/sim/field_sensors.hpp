#pragma once
#include "sim/sensors.hpp"
#include "sim/signal.hpp"
#include <cstddef>
#include <cstdint>
#include <deque>
#include <random>
#include <string>
#include <vector>

// More sensors from a real line: proximity switches, a laser distance
// sensor and a light curtain. (Photocells and encoders are in sensors.hpp.)
namespace fsim {

// Where a sensor sits around the belt: above it looking down, or beside it
// looking across (left and right looking along the travel).
enum class Mount { Above, Left, Right };

// ---------------------------------------------------------------- Proximity

// An inductive or capacitive proximity switch. It switches when something
// is within its switching distance `range` (sn) in front of its face.
//
// - Inductive: a coil's field, damped by metal. It sees tins, not cardboard.
// - Capacitive: an electric field, changed by anything denser than air, but
//   a datasheet's sn is for metal: cardboard is seen only much closer
//   (here at 40 % of sn).
//
// Sideways, it works like a photocell whose "beam" is its face: an item
// passing in front of it covers more and more of the face, and the output
// switches at half of it covered, with hysteresis and a response time. So
// it shares the photocell's switching code (`detector`).
struct ProximitySpec {
    std::string name = "BP1";
    std::size_t conveyor = 0;
    float along = 1.0F;
    Mount mount = Mount::Left;
    bool inductive = true;       // false: capacitive
    float faceDiameter = 0.018F; // M18
    float range = 0.008F;        // sn, for metal
    float height = 0.05F;        // beside: the face's centre above the belt; above: the face's height above the belt
    float inset = 0.0F;          // beside: how far in from the belt's edge the face sits (through the guide rail)
    double responseTime = 0.001;
    std::string input;

    bool operator==(const ProximitySpec&) const = default;
};

// How far a proximity switch sees an item, metal or not, as a share of sn.
float rangeFactor(const ProximitySpec& spec, bool metal);

struct Proximity {
    ProximitySpec spec;
    Photocell detector;     // the switching: its "beam" is the face
    float distance = -1.0F; // to the nearest item in front of it, m (-1: none)
};

// ---------------------------------------------------------------- Laser

// A laser distance sensor above the belt, looking down: it measures how far
// the surface under its spot is (the belt, or the top of an item), and gives
// it to the PLC as an analog word: rangeMin..rangeMax as 0..27648, the
// Siemens analog range, and 32767 (overflow) when nothing is in range.
// Real ones measure in steps: every `measureTime`, with a resolution and
// some noise.
struct LaserSpec {
    std::string name = "BL1";
    std::size_t conveyor = 0;
    float along = 1.0F;
    float across = 0.0F;      // the spot's distance from the belt's centre line (left positive), m
    float mountHeight = 0.5F; // the sensor above the belt surface, m
    float rangeMin = 0.1F;    // its measuring range, from the sensor
    float rangeMax = 0.6F;
    float resolution = 0.0005F; // the smallest step it reports, m
    float noise = 0.0002F;      // standard deviation of its measurement, m
    double measureTime = 0.002; // s per measurement
    std::string output;         // the input word it's wired to (%IW)

    bool operator==(const LaserSpec&) const = default;
};

constexpr std::uint16_t kAnalogOverflow = 32767;

struct Laser {
    LaserSpec spec;
    float trueDistance = -1.0F;            // what's really there now, m
    float distance = -1.0F;                // what it measured last, m (-1: out of range)
    std::uint16_t value = kAnalogOverflow; // the word the PLC reads
    double nextMeasurement = 0.0;
    std::deque<std::pair<double, float>> samples; // its measurements over time, for the analyser
    static constexpr std::size_t kSamples = 12000;
    std::mt19937 random{7}; // seeded: the same noise every run
};

// The analog word for a measured distance (-1: out of range).
std::uint16_t laserValue(const LaserSpec& spec, float distance);

// ---------------------------------------------------------------- Light curtain

// A row of beams across the belt, one above the other: an emitter on one
// side, a receiver on the other. It scans its beams one after another, and
// gives a result once per scan cycle: a "clear" output (on while no beam is
// broken, like a safety light curtain's OSSD) and, as a measuring curtain
// does, how many beams are broken, as an input word.
struct LightCurtainSpec {
    std::string name = "BC1";
    std::size_t conveyor = 0;
    float along = 1.0F;
    int beams = 16;
    float pitch = 0.01F;     // m between beams
    float lowest = 0.01F;    // the lowest beam above the belt, m
    double scanTime = 0.005; // s for one scan of all the beams
    std::string clearInput;  // %IX: on while every beam is clear
    std::string countInput;  // %IW: how many beams are broken

    bool operator==(const LightCurtainSpec&) const = default;
};

struct LightCurtain {
    LightCurtainSpec spec;
    std::vector<bool> broken; // per beam, lowest first, from the last scan
    int blocked = 0;          // how many
    Signal clear;
    double nextScan = 0.0;
};

} // namespace fsim
