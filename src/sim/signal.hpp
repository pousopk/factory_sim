#pragma once
#include <cstddef>
#include <deque>

// A digital signal with exact switching times.
//
// The simulation runs in 1 ms steps, but a sensor doesn't switch on a step:
// a box edge reaches the beam somewhere in between. A Signal records each
// change with its exact time (a double, in seconds), so measurements aren't
// rounded to the step. Its value at any moment is the last change before it.
namespace fsim {

struct Edge {
    double time;
    bool value; // the value from `time` on
};

class Signal {
public:
    bool value() const { return value_; }

    // The value at time `t`, from the remembered changes (false before the first one).
    bool valueAt(double t) const;

    // Changes the value at `time`. Changes must come in time order; setting
    // the value it already has does nothing.
    void set(double time, bool value);

    // The most recent changes, oldest first (up to kHistory of them).
    const std::deque<Edge>& edges() const { return edges_; }

    static constexpr std::size_t kHistory = 4096;

private:
    bool value_ = false;
    std::deque<Edge> edges_;
};

} // namespace fsim
