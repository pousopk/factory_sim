#pragma once
#include "render/scene.hpp"
#include "sim/scene.hpp"
#include <cstddef>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

// Editing in the 3D view, as geometry: which belt the mouse is over and
// where along it, where it meets the floor, moving a device along its
// conveyor, and copy and paste. No ImGui and no camera here (a ray is an
// origin and a direction), so it's all tested; the app turns mouse moves
// into rays and calls these.
namespace fsim {

// Where the mouse "is" along a conveyor, even off the belt: where the ray
// meets the belt's level, projected onto its centre line (or, looking almost
// level with the belt, the point of the centre line nearest the ray). Not
// clamped to the conveyor's length.
float alongNearest(const ConveyorSpec& conveyor, glm::vec3 origin, glm::vec3 direction);

// Where the ray meets the horizontal plane at `height`: nothing if it runs
// parallel to it or away from it.
std::optional<glm::vec3> onPlane(glm::vec3 origin, glm::vec3 direction, float height);

// The belt the ray hits first (its top surface), and where along it.
struct BeltHit {
    std::size_t conveyor;
    float along;
    float distance; // along the ray
};
std::optional<BeltHit> beltUnder(const std::vector<ConveyorSpec>& conveyors, glm::vec3 origin, glm::vec3 direction);

// `value` to the nearest multiple of `step` (0: as it is).
float snap(float value, float step);

// A device's conveyor and its place along it (a spawner's offset), if the
// pick is a device that sits on a conveyor.
std::optional<std::pair<std::size_t, float>> placeOf(const Scene& scene, Pick pick);
// Moves it there, within its conveyor's length.
void moveAlong(Scene& scene, Pick pick, float along);

// A new device of `kind` on `conveyor` at `along` (click to place), wired to
// free addresses. Nothing for a kind that doesn't sit on a conveyor.
std::optional<Pick> addAt(Scene& scene, Pick::Kind kind, std::size_t conveyor, float along);

// What Ctrl+C keeps: a copy of one thing in the scene.
using Clip = std::variant<PlacedConveyor, PhotocellSpec, SpawnerSpec, PusherSpec, DiverterSpec, StopperSpec, BinSpec,
                          CameraSpec, ProximitySpec, LaserSpec, LightCurtainSpec>;
std::optional<Clip> copyOf(const Scene& scene, Pick pick);

// Adds what the clip holds, as a new thing: the next free name of its kind,
// its I/O on free addresses (a copy wired to the same inputs and outputs
// would fight the original). A device goes on `conveyor` at `along` if given,
// or on its old conveyor 0.2 m further; a conveyor goes beside the original,
// on its own. Returns the new thing.
Pick paste(Scene& scene, const Clip& clip, std::optional<std::size_t> conveyor = {}, std::optional<float> along = {});

} // namespace fsim
