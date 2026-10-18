#include "analysis/tin_study.hpp"
#include <algorithm>
#include <cmath>

namespace fsim {

bool GapOutcome::allTaken() const {
    return !cameras.empty() &&
           std::all_of(cameras.begin(), cameras.end(), [](const CameraOutcome& c) { return c.missed == 0; });
}

GapOutcome studyGap(const Scene& scene, double gap, const ProgramMaker& make, std::size_t tins, int phases) {
    GapOutcome outcome;
    outcome.gap = gap;
    if (scene.cameras.empty() || scene.cameras.front().conveyor >= scene.conveyors.size()) {
        return outcome;
    }
    const auto conveyor = scene.cameras.front().conveyor;
    const double speed = scene.conveyors[conveyor].spec.targetSpeed;
    double length = 0.073;
    for (const auto& spawner : scene.spawners) {
        if (spawner.conveyor == conveyor) {
            length = spawner.size.x;
        }
    }
    std::vector<const CameraSpec*> cameras;
    for (const auto& camera : scene.cameras) {
        if (camera.conveyor == conveyor) {
            cameras.push_back(&camera);
            outcome.cameras.push_back(CameraOutcome{camera.name});
        }
    }
    std::vector<PhotocellSpec> cells;
    for (const auto& photocell : scene.photocells) {
        if (photocell.conveyor == conveyor) {
            cells.push_back(photocell);
        }
    }
    if (speed <= 0.0 || cells.empty()) {
        return outcome;
    }
    const double pitch = length + gap;
    outcome.interval = pitch / speed;

    // From where the first tin starts (just before the first photocell) to
    // past the last camera's picture, for the last tin.
    const double start =
        std::min_element(cells.begin(), cells.end(), [](const auto& a, const auto& b) { return a.along < b.along; })
            ->along -
        0.02;
    double end = 0.0;
    double longestBusy = 0.0;
    for (const auto* camera : cameras) {
        end = std::max(end, static_cast<double>(camera->along + camera->fieldOfView / 2.0F));
        longestBusy = std::max(longestBusy, camera->busyTime());
    }
    const double scan = World::kStep * World::kScanSteps;
    // The addresses, read once (the program reads its own).
    std::vector<std::optional<IoAddress>> cellInputs;
    for (const auto& cell : cells) {
        cellInputs.push_back(parseAddress(cell.input));
    }
    std::vector<std::optional<IoAddress>> triggerOutputs;
    std::vector<std::optional<IoAddress>> readyInputs;
    for (const auto* camera : cameras) {
        const auto out = parseAddress(camera->triggerOutput);
        triggerOutputs.push_back(out && out->area == IoAddress::Area::Output ? out : std::nullopt);
        readyInputs.push_back(parseAddress(camera->readyInput));
    }
    std::vector<std::pair<float, float>> intervals;

    for (int phase = 0; phase < phases; ++phase) {
        // Shifting the tins by a fraction of a scan moves their edges within the cycle.
        const double shift = speed * scan * phase / phases;
        const auto front = [&](std::size_t k, double t) {
            return start - shift + speed * t - static_cast<double>(k) * pitch;
        };
        const double duration = (end - start + shift + length + static_cast<double>(tins) * pitch) / speed +
                                longestBusy + 0.2; // and the delays, the busy time and a margin
        const auto steps = static_cast<long>(std::ceil(duration / World::kStep));

        auto program = make(scene);
        IoImage io;
        std::vector<Photocell> photocells(cells.size());
        for (std::size_t i = 0; i < cells.size(); ++i) {
            photocells[i].spec = cells[i];
        }
        std::vector<CameraTiming> timing(cameras.size());
        std::vector<bool> lastTrigger(cameras.size(), false);
        std::vector<std::vector<double>> exposures(cameras.size()); // pending, in time order
        std::vector<std::vector<bool>> taken(cameras.size(), std::vector<bool>(tins, false));

        for (long n = 0; n < steps; ++n) {
            const double t0 = static_cast<double>(n) * World::kStep;
            const double t1 = t0 + World::kStep;
            if (n % World::kScanSteps == 0) {
                // A scan, as World::scan does it: inputs, the program, outputs.
                io.inputs().fill(0);
                for (std::size_t i = 0; i < photocells.size(); ++i) {
                    if (cellInputs[i]) {
                        io.setBit(*cellInputs[i], photocells[i].output.value());
                    }
                }
                for (std::size_t c = 0; c < cameras.size(); ++c) {
                    if (readyInputs[c]) {
                        io.setBit(*readyInputs[c], !timing[c].busyAt(t0));
                    }
                }
                if (program) {
                    program->scan(io, t0);
                }
                for (std::size_t c = 0; c < cameras.size(); ++c) {
                    const bool on = triggerOutputs[c] && io.bit(*triggerOutputs[c]);
                    if (on && !lastTrigger[c]) {
                        if (timing[c].trigger(t0, cameras[c]->busyTime())) {
                            exposures[c].push_back(t0 + cameras[c]->triggerDelay + cameras[c]->exposure / 2.0);
                        } else {
                            ++outcome.cameras[c].ignored;
                        }
                    }
                    lastTrigger[c] = on;
                }
            }
            // The photocells, at the end of the step. Only the tins at the beam
            // can cover it: those whose front has reached it and whose back
            // hasn't passed it, worked out from the positions rather than
            // tried one by one.
            for (auto& photocell : photocells) {
                const double half = photocell.spec.beamDiameter / 2.0;
                const double lead = front(0, t1); // tin k's front is lead - k * pitch
                const auto first = std::max(0.0, std::ceil((lead - (photocell.spec.along + half) - length) / pitch));
                const auto last = std::floor((lead - (photocell.spec.along - half)) / pitch);
                intervals.clear();
                for (auto k = static_cast<std::size_t>(first); static_cast<double>(k) <= last && k < tins; ++k) {
                    const double f = front(k, t1);
                    intervals.emplace_back(static_cast<float>(f - length), static_cast<float>(f));
                }
                // Most of the time the beam is clear, or inside one tin: only
                // with an edge in the beam does the coverage need working out.
                const auto from = static_cast<float>(photocell.spec.along - half);
                const auto to = static_cast<float>(photocell.spec.along + half);
                const bool inside = std::any_of(intervals.begin(), intervals.end(),
                                                [&](const auto& i) { return i.first <= from && i.second >= to; });
                const float coverage = intervals.empty() ? 0.0F
                                       : inside
                                           ? 1.0F
                                           : beamCoverage(photocell.spec.along, photocell.spec.beamDiameter, intervals);
                photocell.sense(coverage, t0, World::kStep);
                while (photocell.nextOutput(t1)) {
                }
            }
            // Exposures by now: the tin nearest the middle of the picture, if any is in it.
            for (std::size_t c = 0; c < cameras.size(); ++c) {
                auto& due = exposures[c];
                while (!due.empty() && due.front() <= t1) {
                    const double t = due.front();
                    due.erase(due.begin());
                    std::optional<std::size_t> nearest;
                    double best = 0.0;
                    for (std::size_t k = 0; k < tins; ++k) {
                        const double offset = std::abs(front(k, t) - length / 2.0 - cameras[c]->along);
                        if (offset - length / 2.0 < cameras[c]->fieldOfView / 2.0 && (!nearest || offset < best)) {
                            nearest = k;
                            best = offset;
                        }
                    }
                    if (nearest) {
                        taken[c][*nearest] = true;
                    }
                }
            }
        }
        for (std::size_t c = 0; c < cameras.size(); ++c) {
            outcome.cameras[c].items += tins;
            outcome.cameras[c].missed += static_cast<std::size_t>(std::count(taken[c].begin(), taken[c].end(), false));
        }
    }
    return outcome;
}

std::vector<GapOutcome> sweepGaps(const Scene& scene, double from, double to, double step, const ProgramMaker& make) {
    std::vector<GapOutcome> sweep;
    // Counting steps rather than adding `step` up, so the gaps don't drift.
    const auto count = static_cast<long>(std::floor((to - from) / step + 1e-9));
    for (long i = 0; i <= count; ++i) {
        sweep.push_back(studyGap(scene, from + static_cast<double>(i) * step, make));
    }
    return sweep;
}

std::optional<double> minimumGap(const std::vector<GapOutcome>& sweep, std::optional<std::size_t> camera) {
    const auto works = [&](const GapOutcome& g) {
        if (camera) {
            return *camera < g.cameras.size() && g.cameras[*camera].missed == 0;
        }
        return g.allTaken();
    };
    std::optional<double> minimum;
    for (auto it = sweep.rbegin(); it != sweep.rend() && works(*it); ++it) {
        minimum = it->gap; // walking down from the largest gap while it still works
    }
    return minimum;
}

} // namespace fsim
