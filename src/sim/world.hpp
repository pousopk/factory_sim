#pragma once
#include "sim/physics.hpp"
#include "sim/sensors.hpp"
#include <cstdint>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace fsim {

// Units everywhere: metres, seconds, kilograms. Y is up; the floor is y = 0.

struct ConveyorSpec {
    std::string name = "Conveyor";
    glm::vec3 start{0.0F}; // on the floor, under the start of the belt's centre line
    float heading = 0.0F;  // direction of travel, radians around Y: 0 = +X, pi/2 = -Z
    float length = 3.0F;
    float width = 0.4F;
    float beltHeight = 0.8F;   // height of the belt surface above the floor
    float targetSpeed = 0.5F;  // m/s; the belt ramps towards it
    float acceleration = 1.0F; // m/s², like a VFD's ramp time (0 to 1 m/s in 1 s)
    EncoderSpec encoder;       // on the drive roller
};

struct Conveyor {
    ConveyorSpec spec;
    float speed = 0.0F;           // the actual belt speed now, m/s
    double travelled = 0.0;       // how far the belt surface has moved in total, m (for the moving stripes)
    double travelledBefore = 0.0; // the same, at the start of the last step
    JPH::BodyID body;

    // The encoder's count: whole pulses since the start (negative if the belt ran backwards).
    std::int64_t encoderCount() const;

    glm::vec3 direction() const;            // unit vector of travel
    glm::vec3 beltPoint(float along) const; // the point on the belt surface, `along` metres from its start
};

struct SpawnerSpec {
    std::size_t conveyor = 0;
    double interval = 1.0;             // seconds between boxes
    double intervalJitter = 0.0;       // ± random variation, seconds
    glm::vec3 size{0.2F, 0.15F, 0.2F}; // length (along the belt), height, width
    float offset = 0.2F;               // where boxes appear, metres from the belt's start
    bool enabled = true;
};

// Everything the renderer and the UI need to know about one box.
struct ItemView {
    ItemId id;
    glm::vec3 position;
    glm::quat rotation;
    glm::vec3 size;
    glm::vec3 color;
    glm::vec3 velocity;
};

// The simulated factory: conveyors, boxes on them, and time.
//
// Time advances in fixed steps of exactly 1 ms, whatever the screen's frame
// rate, so timing effects are real and repeatable: at 2 m/s a box moves
// exactly 2 mm per step.
class World {
public:
    static constexpr double kStep = 0.001; // seconds per simulation step

    World();
    ~World();
    World(const World&) = delete;
    World& operator=(const World&) = delete;

    std::size_t addConveyor(const ConveyorSpec& spec);
    std::size_t addSpawner(const SpawnerSpec& spec);
    std::size_t addPhotocell(const PhotocellSpec& spec);

    ItemId spawnItem(glm::vec3 position, glm::vec3 size, glm::vec3 color, float heading = 0.0F);
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

    double time() const { return static_cast<double>(stepCount_) * kStep; }
    std::uint64_t stepCount() const { return stepCount_; }

    std::vector<Conveyor>& conveyors() { return conveyors_; }
    const std::vector<Conveyor>& conveyors() const { return conveyors_; }
    std::vector<SpawnerSpec>& spawners() { return spawners_; }
    std::vector<Photocell>& photocells() { return photocells_; }
    const std::vector<Photocell>& photocells() const { return photocells_; }

    // A conveyor's encoder count at time `t` within the last step (the belt's
    // travel is interpolated between the step's start and end).
    std::int64_t encoderCountAt(std::size_t conveyor, double t) const;

    std::vector<ItemView> items() const;
    std::size_t itemCount() const { return items_.size(); }

private:
    struct Item {
        ItemId id;
        JPH::BodyID body;
        glm::vec3 size;
        glm::vec3 color;
        double onFloorSince = -1.0; // when it came to rest on the floor, or -1
    };
    struct SpawnerState {
        double nextSpawn = 0.0;
    };

    class ConveyorContacts;

    void updateConveyors();
    void updateSpawners();
    void removeFallenItems();
    void updatePhotocells(double stepStart, double stepEnd);

    Physics physics_;
    std::vector<Conveyor> conveyors_; // before contacts_, which refers to it
    std::unique_ptr<ConveyorContacts> contacts_;
    JPH::BodyID floor_;
    std::vector<SpawnerSpec> spawners_;
    std::vector<SpawnerState> spawnerStates_;
    std::vector<Item> items_;
    std::vector<Photocell> photocells_;
    ItemId nextItemId_ = 1;
    std::uint64_t stepCount_ = 0;
    double pending_ = 0.0;    // time not yet simulated (less than one step)
    std::mt19937 random_{42}; // fixed seed: every run is the same
};

} // namespace fsim
