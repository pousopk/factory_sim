// factory_sim_bench: how long one 1 ms simulation step takes on a busy line.
//
//   factory_sim_bench              six 8 m belts, a box every 0.22 s on each (about 540 boxes)
//   factory_sim_bench scene.json   that scene instead
//   factory_sim_bench ... 5        fill for 5 simulated seconds instead of 20 (for a profiler)
//   factory_sim_bench ... threads=1  step the physics on one thread (default: as many as useful)
//
// It fills the line, then times 2 more simulated seconds. Built with
// the project's build type: measure in Release.
#include "sim/scene.hpp"
#include <chrono>
#include <cstdio>
#include <string>

namespace {

fsim::Scene busyLine() {
    fsim::Scene scene;
    for (int i = 0; i < 6; ++i) {
        fsim::PlacedConveyor conveyor;
        conveyor.spec.name = "C" + std::to_string(i + 1);
        conveyor.spec.length = 8.0F;
        conveyor.spec.start = {0.0F, 0.0F, -1.5F + 0.6F * static_cast<float>(i)};
        conveyor.spec.targetSpeed = 0.5F;
        scene.conveyors.push_back(conveyor);
        fsim::SpawnerSpec spawner;
        spawner.conveyor = static_cast<std::size_t>(i);
        spawner.interval = 0.22;
        spawner.size = {0.1F, 0.1F, 0.1F};
        spawner.atBeltSpeed = true;
        scene.spawners.push_back(spawner);
    }
    return scene;
}

} // namespace

int main(int argc, char** argv) {
    fsim::Scene scene = busyLine();
    double fill = 20.0;
    int threads = 0;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg.ends_with(".json")) {
            auto loaded = fsim::loadScene(arg);
            if (!loaded) {
                std::fprintf(stderr, "%s\n", loaded.error().c_str());
                return 1;
            }
            scene = *loaded;
        } else if (arg.starts_with("threads=")) {
            threads = std::stoi(arg.substr(8));
        } else {
            fill = std::stod(arg);
        }
    }
    auto world = fsim::buildWorld(scene, fsim::makeProgram, threads);
    world->run(fill); // fill the line
    const auto start = std::chrono::steady_clock::now();
    world->run(2.0);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    std::printf("%zu items, %d physics threads: %.3f ms per 1 ms step (%.2fx real time)\n", world->itemCount(),
                world->physicsThreads(), ms / 2000.0, 2000.0 / ms);
    return 0;
}
