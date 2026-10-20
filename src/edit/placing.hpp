#pragma once
#include "render/scene.hpp"
#include "sim/scene.hpp"
#include <array>
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

// What the gizmo shows for `pick`: the frame it moves and turns (a device's
// usual place, nudged, or where it stands; a conveyor's middle, at its belt,
// turned with it), and which of its arrows and rings it has. A conveyor
// that follows or joins another is placed by it: none.
struct GizmoTarget {
    Frame frame;
    std::array<bool, 3> canMove{true, true, true};
    std::array<bool, 3> canTurn{true, true, true};
};
std::optional<GizmoTarget> gizmoTarget(const Scene& scene, Pick pick);

// Puts `pick` where the gizmo has moved its frame to, `to`, starting again
// from `original` (the scene when the drag began), so a long drag doesn't
// pile up rounding. A device on a belt slides along it as far as the move
// goes along the belt, the rest is its nudge; one standing on its own goes
// there. A conveyor moves, rises and turns about its middle.
void placeAt(Scene& scene, const Scene& original, Pick pick, const Frame& to);

// Takes a device off its conveyor, leaving it exactly where it is
// (standing on its own), or puts it on `conveyor` at its usual place
// nearest to where it is now.
void detach(Scene& scene, Pick pick);
void attach(Scene& scene, Pick pick, std::size_t conveyor);
// A device's placement, and whether it's on a belt as usual (sensing as it
// always has), nudged, or on its own.
const Placement* placementOf(const Scene& scene, Pick pick);
Placement* placementOf(Scene& scene, Pick pick);
// The conveyor nearest to where a device is (its usual place, or where it
// stands): the one to put it on.
std::optional<std::size_t> nearestConveyor(const Scene& scene, Pick pick);

// What Ctrl+C keeps: a copy of one thing in the scene.
using Clip = std::variant<PlacedConveyor, PhotocellSpec, SpawnerSpec, PusherSpec, DiverterSpec, StopperSpec, AirJetSpec,
                          BinSpec, CameraSpec, ProximitySpec, LaserSpec, LightCurtainSpec>;
std::optional<Clip> copyOf(const Scene& scene, Pick pick);

// Adds what the clip holds, as a new thing: the next free name of its kind,
// its I/O on free addresses (a copy wired to the same inputs and outputs
// would fight the original). A device goes on `conveyor` at `along` if given,
// or on its old conveyor 0.2 m further; a conveyor goes beside the original,
// on its own. Returns the new thing.
Pick paste(Scene& scene, const Clip& clip, std::optional<std::size_t> conveyor = {}, std::optional<float> along = {});

} // namespace fsim
