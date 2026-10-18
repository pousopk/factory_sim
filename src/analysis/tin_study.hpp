#pragma once
#include "sim/scene.hpp"
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

// The tin scenario's question: at this belt speed, how close can the tins
// come before a camera misses one?
//
// The full simulation answers it for the tins that happen to come by. This
// answers it for every gap, by running the line's *timing* alone: tins
// placed exactly `gap` apart moving at the belt's set speed, the scene's own
// photocells (the same switching code as in the World: hysteresis, response
// time), its control program scan by scan through the I/O image, and the
// cameras' timing (busy from the trigger to the end of the readout). No
// physics: tins on a running belt move at its speed, so nothing physics
// would add changes the answer, and a whole sweep of gaps takes a moment.
//
// The scan is the one thing the gap doesn't decide: where a tin's edge falls
// within the 10 ms cycle. So each gap is run several times, with the tins
// shifted by a fraction of a scan each time, and counts as working only if
// every camera takes every tin in all of them.
namespace fsim {

struct CameraOutcome {
    std::string name;
    std::size_t items = 0;   // tins that passed it (over all the runs)
    std::size_t missed = 0;  // of which it took no picture
    std::size_t ignored = 0; // triggers it ignored because it was busy
};

struct GapOutcome {
    double gap = 0.0;      // m between tins, edge to edge
    double interval = 0.0; // s from one tin to the next
    std::vector<CameraOutcome> cameras;
    bool allTaken() const;
};

// The cameras on the first camera's conveyor, with tins `gap` apart. The
// program is the scene's, made by `make` (the built-in ones by default; the
// app passes one that makes soft_plc too).
GapOutcome studyGap(const Scene& scene, double gap, const ProgramMaker& make = makeProgram, std::size_t tins = 12,
                    int phases = 5);

// studyGap for every gap from `from` to `to`, `step` apart.
std::vector<GapOutcome> sweepGaps(const Scene& scene, double from, double to, double step,
                                  const ProgramMaker& make = makeProgram);

// The smallest gap of the sweep from which every larger one works: for one
// camera, or (without `camera`) for all of them. Nothing if the largest gap
// doesn't work either.
std::optional<double> minimumGap(const std::vector<GapOutcome>& sweep, std::optional<std::size_t> camera = {});

} // namespace fsim
