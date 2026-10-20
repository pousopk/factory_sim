#pragma once
#include "sim/actuators.hpp"
#include "sim/camera.hpp"
#include "sim/control.hpp"
#include "sim/conveyor_spec.hpp"
#include "sim/field_sensors.hpp"
#include "sim/io.hpp"
#include "sim/physics.hpp"
#include "sim/sensors.hpp"
#include <cstdint>
#include <deque>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace fsim {

struct Conveyor {
    ConveyorSpec spec;
    float speed = 0.0F;           // the actual belt speed now, m/s
    double travelled = 0.0;       // how far the belt surface has moved in total, m (for the moving stripes)
    double travelledBefore = 0.0; // the same, at the start of the last step
    JPH::BodyID body;
    std::vector<JPH::BodyID> guides; // its side rails, if it has any
    bool running = true;             // its motor's run command (from the control program, if it has one)
    Signal motor;                    // the same, as a signal with the times it changed
    // The belt's speed, sampled every scan (for the logic analyser): time and m/s.
    std::deque<std::pair<double, float>> speedSamples;
    // Its drive, if it has one (spec.drive.fitted), and the speed the PLC asked
    // it for, sampled every scan like the speed (m/s).
    Drive drive;
    std::deque<std::pair<double, float>> setpointSamples;
    // An accumulation conveyor's zones: each one's speed, whether its motor
    // runs, and its photo-eye (a photocell near the zone's end).
    std::vector<float> zoneSpeed;
    std::vector<bool> zoneRun;
    std::vector<ZoneState> zoneState; // local logic: what each zone holds
    std::vector<bool> eyeBefore;      // each eye at the last step, for its edges
    std::vector<Photocell> eyes;
    // A conveyor on a carriage: where it is placed at home (spec is where it
    // is now), and the carriage's movement.
    ConveyorSpec home;
    Stroke carriage;
    std::vector<glm::vec3> guideOffsets;                // its guides, from the belt's middle, in the belt's own axes
    static constexpr std::size_t kSpeedSamples = 12000; // two minutes of scans

    // The encoder's count: whole pulses since the start (negative if the belt ran backwards).
    std::int64_t encoderCount() const;

    glm::vec3 direction() const;            // unit vector of travel
    float speedAt(float along) const;       // the belt's speed there: its zone's, on an accumulation conveyor
    glm::vec3 beltPoint(float along) const; // the point on the belt surface, `along` metres from its start
};

struct SpawnerSpec {
    std::size_t conveyor = 0;
    double interval = 1.0;             // seconds between boxes
    double intervalJitter = 0.0;       // ± random variation, seconds
    glm::vec3 size{0.2F, 0.15F, 0.2F}; // length (along the belt), height, width
    float offset = 0.2F;               // where boxes appear, metres from the belt's start
    bool enabled = true;
    float nokFraction = 0.0F; // the share of boxes that are bad (NOK), at random: 0.25 = one in four
    // What it makes. Size is the items' bounding box (a tin's: diameter,
    // height, diameter); an STL part's size comes from its file.
    Shape shape = Shape::Box;
    StlSpec stl;               // the file, for Shape::Stl
    float heightJitter = 0.0F; // ± m at random on each item's height (not an STL part's)
    // Items arrive already moving with the belt, as from an infeed conveyor,
    // instead of being put down on it standing still.
    bool atBeltSpeed = false;

    bool operator==(const SpawnerSpec&) const = default;
};

// Everything the renderer and the UI need to know about one box.
struct ItemView {
    ItemId id;
    glm::vec3 position;
    glm::quat rotation;
    glm::vec3 size;
    glm::vec3 color;
    glm::vec3 velocity;
    bool nok = false; // a bad box, to be sorted out
    Shape shape = Shape::Box;
    std::shared_ptr<const ItemGeometry> geometry{}; // its shape, if not a box or a tin
};

// The simulated factory: conveyors, boxes on them, and time.
//
// Time advances in fixed steps of exactly 1 ms, whatever the screen's frame
// rate, so timing effects are real and repeatable: at 2 m/s a box moves
// exactly 2 mm per step.
// The actuators as they run: their settings, their stroke, their body.
struct Pusher {
    PusherSpec spec;
    Stroke stroke;
    JPH::BodyID body;
};
struct Diverter {
    DiverterSpec spec;
    Stroke stroke;
    JPH::BodyID body;
};
struct Stopper {
    StopperSpec spec;
    Stroke stroke;
    JPH::BodyID body;
};
struct Bin {
    BinSpec spec;
    std::vector<JPH::BodyID> walls;
    std::size_t count = 0;      // boxes that landed in it
    std::size_t nokCount = 0;   // of which bad ones
    std::vector<ItemId> inside; // the boxes in it now (counted once each)
};

class World {
public:
    static constexpr double kStep = 0.001; // seconds per simulation step

    // `physicsThreads`: how many threads step the physics (0: as many as useful).
    explicit World(int physicsThreads = 0);
    ~World();
    World(const World&) = delete;
    World& operator=(const World&) = delete;

    std::size_t addConveyor(const ConveyorSpec& spec);
    std::size_t addSpawner(const SpawnerSpec& spec);
    std::size_t addPhotocell(const PhotocellSpec& spec);
    // Actuators and bins sit on an existing conveyor (add it first).
    std::size_t addPusher(const PusherSpec& spec);
    std::size_t addDiverter(const DiverterSpec& spec);
    std::size_t addStopper(const StopperSpec& spec);
    std::size_t addBin(const BinSpec& spec);

    // Replaces the photocells' and spawners' settings while running (an
    // edit in the inspector). A photocell that stays on the same conveyor
    // keeps its state (its output, its triggers); a spawner keeps its timing.
    void setPhotocells(const std::vector<PhotocellSpec>& specs);
    void setSpawners(const std::vector<SpawnerSpec>& specs);
    // Actuator timing changes while running (their place is the layout: a new world).
    void setStrokes(const std::vector<PusherSpec>& pushers, const std::vector<DiverterSpec>& diverters,
                    const std::vector<StopperSpec>& stoppers);

    // An item at `position` (the middle of its bounding box). A shape other
    // than a box or a tin brings its `geometry` (made for `size` if it's
    // missing), which also decides its size.
    ItemId spawnItem(glm::vec3 position, glm::vec3 size, glm::vec3 color, float heading = 0.0F, bool nok = false,
                     Shape shape = Shape::Box, glm::vec3 velocity = glm::vec3(0.0F),
                     std::shared_ptr<const ItemGeometry> geometry = nullptr);
    // The colour items of a shape get (a box's depends on the box).
    glm::vec3 itemColor(Shape shape) const;
    void removeItem(ItemId id);
    void clearItems(); // removes every item

    // Runs as many 1 ms steps as fit in `seconds` (plus what was left over
    // last time). At most `maxSteps`: if the computer can't keep up, the rest
    // is dropped rather than freezing the program. Returns the steps run.
    int advance(double seconds, int maxSteps = 250);
    void step(); // exactly one step

    // Simulates exactly `seconds` (rounded to whole steps), however long that
    // takes. For tests and measurements; the app uses advance().
    void run(double seconds);

    int physicsThreads() const { return physics_.threads(); }
    double time() const { return static_cast<double>(stepCount_) * kStep; }
    std::uint64_t stepCount() const { return stepCount_; }

    // New settings for the conveyors, in order, while running: speeds,
    // zones' and carriages' wiring and timing. (Their places and sizes are the
    // layout: a new world.) A conveyor on a carriage stays where it has moved.
    void setConveyorSpecs(const std::vector<ConveyorSpec>& specs);
    std::vector<Conveyor>& conveyors() { return conveyors_; }
    const std::vector<Conveyor>& conveyors() const { return conveyors_; }
    std::vector<SpawnerSpec>& spawners() { return spawners_; }
    const std::vector<SpawnerSpec>& spawners() const { return spawners_; }
    // Why a spawner can't make its items (its STL file is missing...), or empty.
    std::string spawnerProblem(std::size_t spawner) const;
    std::vector<Photocell>& photocells() { return photocells_; }
    const std::vector<Photocell>& photocells() const { return photocells_; }
    std::vector<Pusher>& pushers() { return pushers_; }
    const std::vector<Pusher>& pushers() const { return pushers_; }
    std::vector<Diverter>& diverters() { return diverters_; }
    const std::vector<Diverter>& diverters() const { return diverters_; }
    std::vector<Stopper>& stoppers() { return stoppers_; }
    const std::vector<Stopper>& stoppers() const { return stoppers_; }
    // ---- Control
    //
    // Every scan (10 ms, like soft_plc's cycle), the devices' inputs are
    // written into io(), the control program runs, and its outputs drive the
    // valves, motors and lamps. Without a program, nothing drives them:
    // actuators are moved by hand, and conveyors always run.
    static constexpr int kScanSteps = 10;
    IoImage& io() { return io_; }
    const IoImage& io() const { return io_; }
    void setProgram(std::unique_ptr<ControlProgram> program) { program_ = std::move(program); }
    ControlProgram* program() { return program_.get(); }
    // The control panel: its buttons (pressed or not) and lamps.
    void setPanel(const std::vector<ButtonSpec>& buttons, const std::vector<LampSpec>& lamps,
                  const std::vector<PotentiometerSpec>& potentiometers = {});
    const std::vector<PotentiometerSpec>& potentiometers() const { return potentiometers_; }
    int potentiometer(std::size_t index) const { return index < turned_.size() ? turned_[index] : 0; }
    void turn(std::size_t potentiometer, int value); // 0 to kAnalogFullScale
    // A drive fault, as if the drive had raised it.
    void tripDrive(std::size_t conveyor, std::string cause);
    const std::vector<ButtonSpec>& buttons() const { return buttons_; }
    const std::vector<LampSpec>& lamps() const { return lamps_; }
    bool pressed(std::size_t button) const { return button < pressed_.size() && pressed_[button]; }
    // The buttons and lamps as signals, for the logic analyser.
    const Signal& buttonSignal(std::size_t button) const { return buttonSignals_.at(button); }
    const Signal& lampSignal(std::size_t lamp) const { return lampSignals_.at(lamp); }
    void press(std::size_t button, bool down);
    bool lit(std::size_t lamp) const;

    // ---- Cameras
    //
    // Triggered by the PLC through their trigger output (a rising edge, at a
    // scan), or by hand with triggerCamera. Their "ready" line is an input.
    std::size_t addCamera(const CameraSpec& spec);
    // New settings while running; a camera that stays on its conveyor keeps
    // its history.
    void setCameras(const std::vector<CameraSpec>& specs);
    std::vector<Camera>& cameras() { return cameras_; }
    const std::vector<Camera>& cameras() const { return cameras_; }
    void triggerCamera(std::size_t camera); // now, by hand

    // ---- Proximity switches, laser distance sensors, light curtains
    // Settings while running: each keeps its state if it stays on its conveyor.
    void setFieldSensors(const std::vector<ProximitySpec>& proximities, const std::vector<LaserSpec>& lasers,
                         const std::vector<LightCurtainSpec>& curtains);
    const std::vector<Proximity>& proximities() const { return proximities_; }
    const std::vector<Laser>& lasers() const { return lasers_; }
    const std::vector<LightCurtain>& curtains() const { return curtains_; }

    std::vector<Bin>& bins() { return bins_; }
    const std::vector<Bin>& bins() const { return bins_; }

    // A conveyor's encoder count at time `t` within the last step (the belt's
    // travel is interpolated between the step's start and end).
    std::int64_t encoderCountAt(std::size_t conveyor, double t) const;

    std::vector<ItemView> items() const;
    std::size_t itemCount() const { return items_.size(); }
    // How many boxes the spawners have made since the start, and how many of them were bad.
    std::size_t spawned() const { return spawned_; }
    std::size_t spawnedNok() const { return spawnedNok_; }

private:
    struct Item {
        ItemId id;
        JPH::BodyID body;
        glm::vec3 size;
        glm::vec3 color;
        double onFloorSince = -1.0; // when it came to rest on the floor, or -1
        bool nok = false;
        Shape shape = Shape::Box;
        std::shared_ptr<const ItemGeometry> geometry{}; // null for a box or a tin
    };
    struct SpawnerState {
        double nextSpawn = 0.0;
        bool waiting = false; // its spot was taken when a box was due
        std::string problem;  // why it can't make its items, or empty
    };

    class ConveyorContacts;

    void updateConveyors();
    void updateZones(double stepStart, double stepEnd);
    void updateCarriages(double stepStart, double stepEnd);
    // The items' extents along a conveyor that cross a beam across it at
    // `height` above the belt (for photocells and zone eyes), and which item
    // each is.
    std::vector<std::pair<float, float>> beamIntervals(const Conveyor& conveyor, float height, bool onlyNok,
                                                       std::vector<std::size_t>* owners = nullptr) const;
    void updateSpawners();
    void removeFallenItems();
    void updatePhotocells(double stepStart, double stepEnd);
    void updateActuators(double stepStart, double stepEnd);
    void updateBins();
    void updateCameras(double stepEnd);
    void takeLines(Camera& camera, double stepEnd);
    void takeLine(Camera& camera, double time, double stepEnd);
    void updateFieldSensors(double stepStart, double stepEnd);
    static void recordSetpoint(Conveyor& conveyor, double time);
    void cameraTrigger(Camera& camera, double time);
    void scan(double time);
    // How far an item turned by `rotation` reaches from its centre along
    // `direction`: the lowest and the highest, and the larger of the two.
    static std::pair<float, float> span(const Item& item, const glm::mat3& rotation, glm::vec3 direction);
    static float reachOf(const Item& item, const glm::mat3& rotation, glm::vec3 direction);
    JPH::BodyID addKinematic(const Pose& pose, float friction);
    JPH::BodyID addStatic(const Pose& pose, float friction);

    Physics physics_;
    std::vector<Conveyor> conveyors_; // before contacts_, which refers to it
    std::unique_ptr<ConveyorContacts> contacts_;
    JPH::BodyID floor_;
    std::vector<SpawnerSpec> spawners_;
    std::vector<SpawnerState> spawnerStates_;
    std::vector<Item> items_;
    std::vector<Photocell> photocells_;
    std::vector<Pusher> pushers_;
    std::vector<Diverter> diverters_;
    std::vector<Stopper> stoppers_;
    std::vector<Bin> bins_;
    std::vector<Camera> cameras_;
    std::vector<Proximity> proximities_;
    std::vector<Laser> lasers_;
    std::vector<LightCurtain> curtains_;
    IoImage io_;
    std::unique_ptr<ControlProgram> program_;
    std::vector<ButtonSpec> buttons_;
    std::vector<LampSpec> lamps_;
    std::vector<bool> pressed_;
    std::vector<PotentiometerSpec> potentiometers_;
    std::vector<int> turned_; // each potentiometer's value
    std::vector<Signal> buttonSignals_;
    std::vector<Signal> lampSignals_;
    ItemId nextItemId_ = 1;
    std::size_t spawned_ = 0;
    std::size_t spawnedNok_ = 0;
    std::uint64_t stepCount_ = 0;
    double pending_ = 0.0;    // time not yet simulated (less than one step)
    std::mt19937 random_{42}; // fixed seed: every run is the same
};

} // namespace fsim
