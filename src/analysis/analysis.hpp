#pragma once
#include "sim/world.hpp"
#include <cstddef>
#include <span>
#include <string>
#include <vector>

// The logic analyser's data: which signals there are, their shape over a
// stretch of time, the switches as an event log, and statistics of the
// trigger errors. No drawing here, so it's tested like the simulation; the
// app's analyser window only plots what these functions give it.
namespace fsim {

// One digital signal to show: what it is, where it's wired, and its history.
struct Trace {
    std::string name;    // "B1 photocell", "P1 valve", "Start button", ...
    std::string address; // "%IX0.0", or empty if it isn't wired
    const Signal* signal;
};

// Every digital signal of the world, in a fixed order: conveyors' motors,
// photocells, actuators (valve, extended, retracted), proximity switches,
// light curtains (clear), cameras (trigger, busy), buttons, lamps.
// The pointers are valid as long as the world isn't rebuilt or changed.
std::vector<Trace> traces(const World& world);

// The signal between `from` and `to` as the corners of its step shape, for
// plotting as a line: a point at `from` with its value then, two points
// (old value, new value) at each change, and a point at `to`.
struct Steps {
    std::vector<double> times;
    std::vector<double> values; // 0 or 1
};
Steps steps(const Signal& signal, double from, double to);

// Every switch of every trace, oldest first: the event log.
struct Event {
    double time;
    std::string signal;
    std::string address;
    bool value;
};
std::vector<Event> eventLog(const std::vector<Trace>& traces);

// The log as CSV, one line per switch, times in seconds to the microsecond.
std::string eventsCsv(const std::vector<Event>& events);

// A spread of values: how many, their mean, standard deviation and range.
struct Statistics {
    std::size_t count = 0;
    double mean = 0.0;
    double deviation = 0.0; // standard deviation
    double min = 0.0;
    double max = 0.0;
};
Statistics statistics(std::span<const double> values);

// A photocell's trigger position errors, in millimetres: as the sensor
// gives them, or as a PLC reading it every `scanSeconds` would see them
// (the next scan after the output switched, the box moving on meanwhile).
std::vector<double> positionErrors(const Photocell& photocell);
std::vector<double> positionErrorsAtScan(const Photocell& photocell, double scanSeconds);

} // namespace fsim
