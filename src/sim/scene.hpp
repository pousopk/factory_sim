#pragma once
#include "sim/control.hpp"
#include "sim/sensors.hpp"
#include "sim/world.hpp"
#include <cstddef>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <tl/expected.hpp>
#include <vector>

// The line as it's designed: which conveyors, sensors and spawners, where.
//
// A Scene is a plain value, like a project in ladder_editor: it can be
// copied, compared, saved and loaded, and every edit is "make a new scene".
// The running World (with its boxes, physics and time) is built from it.
// Undo keeps scenes; autosave writes the scene; the world is only ever a
// consequence of it.
//
// Conveyors snap together: one can *follow* another, and then its start and
// direction aren't stored but worked out from the end of the one it
// follows, straight on or turned 90 degrees. Moving or lengthening the
// first conveyor carries the whole chain with it.
namespace fsim {

enum class Turn { Straight, Left, Right };

struct PlacedConveyor {
    ConveyorSpec spec;                  // spec.start and spec.heading only count when it follows nothing
    std::optional<std::size_t> follows; // the conveyor it continues: always an earlier one
    Turn turn = Turn::Straight;         // relative to the one it follows

    bool operator==(const PlacedConveyor&) const = default;
};

// What drives the actuators and motors: nothing (they're moved by hand and
// conveyors always run), or the built-in sorting program.
enum class Control { Manual, BuiltInSorting };

struct Scene {
    std::vector<PlacedConveyor> conveyors;
    std::vector<PhotocellSpec> photocells;
    std::vector<SpawnerSpec> spawners;
    std::vector<PusherSpec> pushers;
    std::vector<DiverterSpec> diverters;
    std::vector<StopperSpec> stoppers;
    std::vector<BinSpec> bins;
    // The control panel, and the control program.
    std::vector<ButtonSpec> buttons;
    std::vector<LampSpec> lamps;
    std::vector<SorterSpec> sorters;
    Control control = Control::Manual;

    bool operator==(const Scene&) const = default;
};

// The gap between a conveyor's end and the next one's start.
constexpr float kTransferGap = 0.02F;

// What a new scene starts with: one conveyor, a spawner, a photocell.
Scene defaultScene();

// Every conveyor's spec with its start and heading worked out, following
// the chains.
std::vector<ConveyorSpec> placeConveyors(const Scene& scene);

// Whether two scenes have the same physical layout: the same conveyors,
// actuators and bins, in the same places and sizes. If they do, a running
// world can take the new scene's settings (speeds, sensors, spawners,
// actuator timing) live; if not, the world has to be built again.
bool sameLayout(const Scene& a, const Scene& b);

// A ready-made line for the sorting demo: a conveyor with an encoder, boxes
// of which about a third are bad, an inspection sensor, a pusher and its
// bin, and the built-in sorter wired to them.
Scene sortingDemo();

// The control program the scene asks for (null for manual control).
std::unique_ptr<ControlProgram> makeProgram(const Scene& scene);

// The first free address of a kind, as soft_plc writes it: the first input
// or output bit not used by any device, button or lamp ("%IX0.3"), or the
// first free double word for an encoder ("%ID8"). Bits are in bytes 0-7,
// double words in bytes 8-15. Empty if none is left.
std::string freeAddress(const Scene& scene, IoAddress::Area area, IoAddress::Size size);

// A sorter wired to an inspection photocell and a pusher on the same
// conveyor: their addresses, and the distance in encoder pulses worked out
// from the layout (see the comment in scene.cpp).
SorterSpec sorterFor(const Scene& scene, std::size_t photocell, std::size_t pusher);

// A new world for the scene (no boxes yet, time 0).
std::unique_ptr<World> buildWorld(const Scene& scene);

// Gives a running world the scene's settings (speeds, sensors, spawners),
// keeping its boxes and time. Only for a scene with the same layout as the
// one the world was built from.
void applyLive(World& world, const Scene& scene);

// ---------------------------------------------------------------- Editing

// A new conveyor continuing `conveyor`, straight on or turned, with the same
// size and speed. Returns its index.
std::size_t addConveyorAfter(Scene& scene, std::size_t conveyor, Turn turn);
// A new conveyor on its own, beside the others. Returns its index.
std::size_t addConveyor(Scene& scene);
// Removes a conveyor and everything on it. Conveyors that followed it stay
// where they are, now on their own; indices after it shift down.
void removeConveyor(Scene& scene, std::size_t conveyor);
// A photocell halfway along `conveyor`, named B1, B2, ... Returns its index.
std::size_t addPhotocell(Scene& scene, std::size_t conveyor);
void removePhotocell(Scene& scene, std::size_t photocell);
// A spawner at the start of `conveyor`. Returns its index.
std::size_t addSpawner(Scene& scene, std::size_t conveyor);
void removeSpawner(Scene& scene, std::size_t spawner);
// Actuators and bins on `conveyor`, at a sensible place: a pusher two thirds
// along on the left, a diverter a third along, a stopper near the end, a bin
// opposite the last pusher (or halfway, on the right). Each returns its index.
std::size_t addPusher(Scene& scene, std::size_t conveyor);
std::size_t addDiverter(Scene& scene, std::size_t conveyor);
std::size_t addStopper(Scene& scene, std::size_t conveyor);
std::size_t addBin(Scene& scene, std::size_t conveyor);
void removePusher(Scene& scene, std::size_t index);
void removeDiverter(Scene& scene, std::size_t index);
void removeStopper(Scene& scene, std::size_t index);
void removeBin(Scene& scene, std::size_t index);

// ---------------------------------------------------------------- Files

std::string sceneToJson(const Scene& scene);
tl::expected<Scene, std::string> sceneFromJson(std::string_view text);

// Writes the scene to `path` safely: to a temporary file first, then renamed
// over the old one, so a crash never leaves half a file.
tl::expected<void, std::string> saveScene(const Scene& scene, const std::filesystem::path& path);
tl::expected<Scene, std::string> loadScene(const std::filesystem::path& path);

} // namespace fsim
