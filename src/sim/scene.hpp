#pragma once
#include "sim/control.hpp"
#include "sim/sensors.hpp"
#include "sim/world.hpp"
#include <cstddef>
#include <filesystem>
#include <functional>
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
    // Or a merge: it joins an earlier conveyor from the side, at right angles,
    // ending at its edge `joinAt` metres along it, 2 cm higher so items drop
    // onto it.
    std::optional<std::size_t> joins;
    float joinAt = 1.0F;
    Side joinSide = Side::Left;

    bool operator==(const PlacedConveyor&) const = default;
};

// What drives the actuators and motors: nothing (they're moved by hand and
// conveyors always run), the built-in sorting program, soft_plc running an
// ST program in lockstep with the simulation, a live PLC over S7, or the
// built-in camera triggering.
enum class Control { Manual, BuiltInSorting, SoftPlc, S7Plc, BuiltInCameras };

struct Scene {
    std::vector<PlacedConveyor> conveyors;
    std::vector<PhotocellSpec> photocells;
    std::vector<SpawnerSpec> spawners;
    std::vector<PusherSpec> pushers;
    std::vector<DiverterSpec> diverters;
    std::vector<StopperSpec> stoppers;
    std::vector<BinSpec> bins;
    std::vector<CameraSpec> cameras;
    std::vector<ProximitySpec> proximities;
    std::vector<LaserSpec> lasers;
    std::vector<LightCurtainSpec> curtains;
    // The control panel, and the control program.
    std::vector<ButtonSpec> buttons;
    std::vector<LampSpec> lamps;
    std::vector<PotentiometerSpec> potentiometers;
    std::vector<SorterSpec> sorters;
    std::vector<CameraTriggerSpec> cameraTriggers;
    Control control = Control::Manual;
    // soft_plc: the ST file it runs (relative to the scene file's folder, or
    // absolute), and a port for monitoring connections (0: none), so
    // ladder_editor or soft_plc_monitor can go online with it.
    std::string plcProgram;
    int monitorPort = 0;
    // A live PLC over S7: where it is (soft_plc --s7, or a real S7, rack 0 slot 1).
    std::string s7Host = "127.0.0.1";
    int s7Port = 102;
    int s7Bytes = 16; // how many input and output bytes to exchange: soft_plc --s7 has 16

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

// The tin line from the reference scenario: tins at 1 m/s with uneven
// gaps, one photocell, three cameras around the tin right after it and a
// fourth further down, which the PLC triggers 150 ms later with a timer.
// The fourth misses tins that come too close; the other three don't.
Scene tinDemo();

// The control program the scene asks for: null for manual control, the
// built-in sorting, or null for the PLC kinds (the PLC link, in factory_sim_plc,
// makes those: see plc/programs.hpp).
std::unique_ptr<ControlProgram> makeProgram(const Scene& scene);
using ProgramMaker = std::function<std::unique_ptr<ControlProgram>(const Scene&)>;

// Whether two scenes ask for the same control program, set up the same way:
// if not, a running world needs a new one.
bool sameControl(const Scene& a, const Scene& b);

// The line from ladder_editor's conveyor example, wired to its addresses: a
// belt whose motor is %QX0.0, light barriers at the start (%IX0.3) and the
// end (%IX0.4), Start, Stop, E-stop and Reset buttons, the program's lamps,
// and soft_plc running `program` (its ST export).
Scene conveyorDemo(std::string program);

// The first free address of a kind, as soft_plc writes it: the first input
// or output bit not used by any device, button or lamp ("%IX0.3"), or the
// first free word or double word ("%IW12", "%ID8"). Bits are in bytes 0-7,
// words and double words from byte 8. Empty if none is left.
std::string freeAddress(const Scene& scene, IoAddress::Area area, IoAddress::Size size);

// A sorter wired to an inspection photocell and a pusher on the same
// conveyor: their addresses, and the distance in encoder pulses worked out
// from the layout (see the comment in scene.cpp).
SorterSpec sorterFor(const Scene& scene, std::size_t photocell, std::size_t pusher);

// A new world for the scene (no boxes yet, time 0), controlled by the program
// `make` gives (by default, makeProgram).
// `physicsThreads` as for World (0: as many as useful).
std::unique_ptr<World> buildWorld(const Scene& scene, const ProgramMaker& make = makeProgram, int physicsThreads = 0);

// Gives a running world the scene's settings (speeds, sensors, spawners),
// keeping its boxes and time. Only for a scene with the same layout as the
// one the world was built from.
void applyLive(World& world, const Scene& scene);

// ---------------------------------------------------------------- Editing

// A new conveyor continuing `conveyor`, straight on or turned, with the same
// size and speed. Returns its index.
std::size_t addConveyorAfter(Scene& scene, std::size_t conveyor, Turn turn);
// A new conveyor joining `conveyor` from `side`, halfway along it (a merge).
// Returns its index.
std::size_t addConveyorJoining(Scene& scene, std::size_t conveyor, Side side);
// Splits a conveyor into `zones` (1: back to one belt), each with its eye
// wired to the next free input (and its motor output, if the PLC runs them).
void setZones(Scene& scene, std::size_t conveyor, int zones, ZoneLogic logic);
// Puts a conveyor on a carriage (or takes it off: Carriage::None), with its
// output and end sensors wired to the next free addresses.
void setCarriage(Scene& scene, std::size_t conveyor, Carriage carriage);
// The line equipment demo: an accumulation conveyor feeding a turntable,
// which turns its boxes onto a second line, up a lift to a third, all
// sequenced by soft_plc running `program` (examples/equipment.st).
Scene equipmentDemo(std::string program);
// A merge: a side conveyor joining a main line, which has priority; the
// side waits for a gap (examples/merge.st).
Scene mergeDemo(std::string program);

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

// Gives `conveyor` a drive on the bus, with the telegram's four words at the
// next free addresses (two output words, two input words).
void fitDrive(Scene& scene, std::size_t conveyor);

// A conveyor with a drive, run by soft_plc running `program`: Start, Stop
// and Acknowledge buttons, a speed potentiometer, and lamps for running and
// fault. examples/drive.st is the program.
Scene driveDemo(std::string program);

// A line to try the sensors on: boxes of different heights and tins, an
// inductive and a capacitive proximity switch beside the belt, a laser
// measuring heights from above, and a light curtain.
Scene sensorsDemo();

// New sensors on `conveyor`, named and wired to the next free addresses.
// Each returns its index.
std::size_t addProximity(Scene& scene, std::size_t conveyor);
std::size_t addLaser(Scene& scene, std::size_t conveyor);
std::size_t addLightCurtain(Scene& scene, std::size_t conveyor);
void removeProximity(Scene& scene, std::size_t index);
void removeLaser(Scene& scene, std::size_t index);
void removeLightCurtain(Scene& scene, std::size_t index);

// A camera over `conveyor`, a little past its middle, named K1, K2, ...,
// with the next free trigger output and ready input. Returns its index.
std::size_t addCamera(Scene& scene, std::size_t conveyor);
void removeCamera(Scene& scene, std::size_t index);

// A trigger for a camera from a photocell on the same conveyor, with the
// delay that puts an item in the middle of the picture at the belt's set
// speed (see the comment in scene.cpp).
CameraTriggerSpec triggerFor(const Scene& scene, std::size_t photocell, std::size_t camera);

// ---------------------------------------------------------------- Files

std::string sceneToJson(const Scene& scene);
tl::expected<Scene, std::string> sceneFromJson(std::string_view text);

// Writes the scene to `path` safely: to a temporary file first, then renamed
// over the old one, so a crash never leaves half a file.
tl::expected<void, std::string> saveScene(const Scene& scene, const std::filesystem::path& path);
tl::expected<Scene, std::string> loadScene(const std::filesystem::path& path);

// The scene is being saved in folder `to` instead of `from`: paths in it that
// are relative to its folder (the soft_plc program) are rewritten to point
// at the same files from there. Relative still if they can be, absolute if
// not (another drive, say); absolute ones are left alone.
void moveScene(Scene& scene, const std::filesystem::path& from, const std::filesystem::path& to);

} // namespace fsim
