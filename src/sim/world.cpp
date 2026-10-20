#include "sim/world.hpp"
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace fsim {

namespace {

JPH::Vec3 toJolt(glm::vec3 v) {
    return JPH::Vec3(v.x, v.y, v.z);
}
glm::vec3 toGlm(JPH::Vec3 v) {
    return {v.GetX(), v.GetY(), v.GetZ()};
}
glm::quat toGlm(JPH::Quat q) {
    return {q.GetW(), q.GetX(), q.GetY(), q.GetZ()};
}

constexpr float kBeltThickness = 0.06F;

// A box shape. Jolt rounds box edges slightly (the "convex radius", 5 cm by
// default), which must not be more than half the smallest side, so thin
// boxes like a belt get a smaller one.
JPH::Ref<JPH::BoxShape> makeBox(JPH::Vec3 halfSize) {
    const float radius = std::min(JPH::cDefaultConvexRadius, 0.5F * halfSize.ReduceMin());
    return new JPH::BoxShape(halfSize, radius);
}

// An item's shape for Jolt: a box, or a tin (a cylinder around its own y
// axis, standing upright).
JPH::Ref<JPH::Shape> makeItemShape(Shape shape, glm::vec3 size) {
    if (shape == Shape::Tin) {
        const float radius = size.x / 2.0F;
        const float halfHeight = size.y / 2.0F;
        return new JPH::CylinderShape(halfHeight, radius,
                                      std::min(JPH::cDefaultConvexRadius, 0.5F * std::min(radius, halfHeight)));
    }
    return JPH::Ref<JPH::Shape>(makeBox(toJolt(size / 2.0F)));
}

// How far an item (turned by `rotation`, half its size `half`) reaches from
// its centre along `direction`. A box: each half-size, projected onto it. A
// tin: its radius across its axis, half its height along it (for a tin
// standing upright, its radius in any horizontal direction).
float reach(Shape shape, const glm::mat3& rotation, glm::vec3 half, glm::vec3 direction) {
    if (shape == Shape::Tin) {
        const float along = std::min(1.0F, std::abs(glm::dot(rotation[1], direction)));
        return half.x * std::sqrt(1.0F - along * along) + half.y * along;
    }
    return std::abs(glm::dot(rotation[0], direction)) * half.x + std::abs(glm::dot(rotation[1], direction)) * half.y +
           std::abs(glm::dot(rotation[2], direction)) * half.z;
}

// A body's user data says what it is: 0 for boxes and the floor, and
// conveyor index + 1 for conveyor belts.
constexpr JPH::uint64 kConveyorTag = 1ULL << 32;

// Tins are tinplate grey.
const glm::vec3 kTinColor{0.78F, 0.80F, 0.83F};

// Bad (NOK) boxes are red, and the good ones never are.
const glm::vec3 kNokColor{0.80F, 0.12F, 0.12F};

// Pastel colours for boxes, so they're easy to tell apart.
glm::vec3 boxColor(ItemId id) {
    static const glm::vec3 palette[] = {{0.85F, 0.55F, 0.30F},
                                        {0.40F, 0.65F, 0.85F},
                                        {0.55F, 0.80F, 0.45F},
                                        {0.90F, 0.75F, 0.35F},
                                        {0.75F, 0.50F, 0.80F}};
    return palette[id % std::size(palette)];
}

} // namespace

std::int64_t Conveyor::encoderCount() const {
    return static_cast<std::int64_t>(std::floor(travelled * spec.encoder.pulsesPerMetre()));
}

glm::vec3 directionOf(float heading) {
    return {std::cos(heading), 0.0F, -std::sin(heading)};
}

glm::vec3 leftOf(glm::vec3 direction) {
    return {direction.z, 0.0F, -direction.x};
}

glm::vec3 beltPoint(const ConveyorSpec& spec, float along) {
    return spec.start + directionOf(spec.heading) * along + glm::vec3(0.0F, spec.beltHeight, 0.0F);
}

ConveyorSpec carried(const ConveyorSpec& home, double position) {
    auto spec = home;
    const auto t = static_cast<float>(std::clamp(position, 0.0, 1.0));
    if (home.carriage == Carriage::Turntable) {
        // Turned about the middle of its belt, which stays where it is.
        const auto middle = home.start + directionOf(home.heading) * (home.length / 2.0F);
        spec.heading = home.heading + home.turnAngle * t;
        spec.start = middle - directionOf(spec.heading) * (home.length / 2.0F);
    } else if (home.carriage == Carriage::Lift) {
        spec.beltHeight = home.beltHeight + home.liftHeight * t;
    }
    return spec;
}

int zoneAt(const ConveyorSpec& spec, float along) {
    if (spec.zones <= 1 || spec.length <= 0.0F) {
        return 0;
    }
    const auto zone = static_cast<int>(std::floor(along / spec.length * static_cast<float>(spec.zones)));
    return std::clamp(zone, 0, spec.zones - 1);
}

float Conveyor::speedAt(float along) const {
    return zoneSpeed.empty() ? speed : zoneSpeed[static_cast<std::size_t>(zoneAt(spec, along))];
}

glm::vec3 Conveyor::direction() const {
    return directionOf(spec.heading);
}

glm::vec3 Conveyor::beltPoint(float along) const {
    return fsim::beltPoint(spec, along);
}

// Makes the belts move things. Jolt calls these for every new or continuing
// contact between two bodies; for a contact with a belt, we tell it the
// belt's surface moves at the belt speed, and friction does the rest, just
// like a real belt. (This is how Jolt's own conveyor sample does it.)
class World::ConveyorContacts final : public JPH::ContactListener {
public:
    explicit ConveyorContacts(const std::vector<Conveyor>& conveyors) : conveyors_(conveyors) {}

    void OnContactAdded(const JPH::Body& body1, const JPH::Body& body2, const JPH::ContactManifold&,
                        JPH::ContactSettings& settings) override {
        apply(body1, body2, settings);
    }
    void OnContactPersisted(const JPH::Body& body1, const JPH::Body& body2, const JPH::ContactManifold&,
                            JPH::ContactSettings& settings) override {
        apply(body1, body2, settings);
    }

private:
    // The belt's surface velocity under `other`, if `body` is a belt. On an
    // accumulation conveyor, it's the speed of the zone under the other
    // body's centre: each zone's rollers turn on their own.
    JPH::Vec3 surfaceVelocity(const JPH::Body& body, const JPH::Body& other) const {
        const auto tag = body.GetUserData();
        if ((tag & kConveyorTag) == 0) {
            return JPH::Vec3::sZero();
        }
        const auto& conveyor = conveyors_[static_cast<std::size_t>(tag & 0xFFFFFFFFULL)];
        float speed = conveyor.speed;
        if (!conveyor.zoneSpeed.empty()) {
            const auto at = toGlm(JPH::Vec3(other.GetCenterOfMassPosition()));
            const float along = glm::dot(at - conveyor.beltPoint(0.0F), conveyor.direction());
            speed = conveyor.zoneSpeed[static_cast<std::size_t>(zoneAt(conveyor.spec, along))];
        }
        return toJolt(conveyor.direction() * speed);
    }

    void apply(const JPH::Body& body1, const JPH::Body& body2, JPH::ContactSettings& settings) const {
        // "Body 2's surface velocity minus body 1's", in world space.
        settings.mRelativeLinearSurfaceVelocity = surfaceVelocity(body2, body1) - surfaceVelocity(body1, body2);
    }

    const std::vector<Conveyor>& conveyors_;
};

World::World(int physicsThreads) : physics_(physicsThreads), contacts_(std::make_unique<ConveyorContacts>(conveyors_)) {
    physics_.setContactListener(contacts_.get());

    // A big floor, its top surface at y = 0.
    JPH::BodyCreationSettings floor(makeBox(JPH::Vec3(50.0F, 0.5F, 50.0F)), JPH::RVec3(0.0, -0.5, 0.0),
                                    JPH::Quat::sIdentity(), JPH::EMotionType::Static, layers::kStatic);
    floor.mFriction = 0.8F;
    floor_ = physics_.bodies().CreateAndAddBody(floor, JPH::EActivation::DontActivate);
}

World::~World() {
    auto& bodies = physics_.bodies();
    for (const auto& item : items_) {
        bodies.RemoveBody(item.body);
        bodies.DestroyBody(item.body);
    }
    const auto destroy = [&](JPH::BodyID id) {
        if (!id.IsInvalid()) {
            bodies.RemoveBody(id);
            bodies.DestroyBody(id);
        }
    };
    for (const auto& conveyor : conveyors_) {
        destroy(conveyor.body);
        for (const auto id : conveyor.guides) {
            destroy(id);
        }
    }
    for (const auto& pusher : pushers_) {
        destroy(pusher.body);
    }
    for (const auto& diverter : diverters_) {
        destroy(diverter.body);
    }
    for (const auto& stopper : stoppers_) {
        destroy(stopper.body);
    }
    for (const auto& bin : bins_) {
        for (const auto id : bin.walls) {
            destroy(id);
        }
    }
    bodies.RemoveBody(floor_);
    bodies.DestroyBody(floor_);
}

std::size_t World::addConveyor(const ConveyorSpec& spec) {
    Conveyor conveyor;
    conveyor.spec = spec;
    conveyor.home = spec;
    conveyor.carriage = Stroke(spec.carriageStroke);
    const auto index = conveyors_.size();
    // On a carriage, the belt and its guides move: bodies the World moves
    // itself every step (kinematic), like the actuators' paddles. Otherwise
    // they're static.
    const bool moves = spec.carriage != Carriage::None;

    // The belt is a thin box whose top surface is at beltHeight.
    const JPH::Vec3 halfSize(spec.length / 2.0F, kBeltThickness / 2.0F, spec.width / 2.0F);
    const auto center = conveyor.beltPoint(spec.length / 2.0F) - glm::vec3(0.0F, kBeltThickness / 2.0F, 0.0F);
    JPH::BodyCreationSettings belt(makeBox(halfSize), JPH::RVec3(center.x, center.y, center.z),
                                   JPH::Quat::sRotation(JPH::Vec3::sAxisY(), spec.heading),
                                   moves ? JPH::EMotionType::Kinematic : JPH::EMotionType::Static,
                                   moves ? layers::kMoving : layers::kStatic);
    belt.mFriction = 1.0F; // rubber
    belt.mUserData = kConveyorTag | index;
    belt.mAllowSleeping = !moves;
    conveyor.body =
        physics_.bodies().CreateAndAddBody(belt, moves ? JPH::EActivation::Activate : JPH::EActivation::DontActivate);

    // Side guides: rails just outside the belt's edges, rising above it.
    // Their place is kept relative to the belt (along, up, left), so a
    // carriage can carry them.
    for (const auto& [wanted, sign] : {std::pair{spec.guideLeft, 1.0F}, std::pair{spec.guideRight, -1.0F}}) {
        if (wanted) {
            conveyor.guideOffsets.emplace_back(0.0F, kGuideHeight / 2.0F,
                                               sign * (spec.width / 2.0F + kGuideThickness / 2.0F));
        }
    }
    for (const auto& offset : conveyor.guideOffsets) {
        const Pose rail{conveyor.beltPoint(spec.length / 2.0F) + glm::vec3(0.0F, offset.y, 0.0F) +
                            leftOf(conveyor.direction()) * offset.z,
                        glm::angleAxis(spec.heading, glm::vec3(0, 1, 0)),
                        {spec.length, kGuideHeight, kGuideThickness}};
        conveyor.guides.push_back(moves ? addKinematic(rail, 0.2F) : addStatic(rail, 0.2F));
    }

    // An accumulation conveyor's zones, all running to begin with.
    if (spec.zones > 1) {
        const auto zones = static_cast<std::size_t>(spec.zones);
        conveyor.zoneSpeed.assign(zones, 0.0F);
        conveyor.zoneRun.assign(zones, true);
        conveyor.zoneState.assign(zones, ZoneState::Empty);
        conveyor.eyeBefore.assign(zones, false);
        conveyor.eyes.resize(zones);
    }

    conveyor.motor.set(time(), true); // runs until a program says otherwise
    conveyors_.push_back(conveyor);
    return index;
}

JPH::BodyID World::addStatic(const Pose& pose, float friction) {
    JPH::BodyCreationSettings settings(makeBox(toJolt(pose.size / 2.0F)),
                                       JPH::RVec3(pose.center.x, pose.center.y, pose.center.z),
                                       JPH::Quat(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w),
                                       JPH::EMotionType::Static, layers::kStatic);
    settings.mFriction = friction;
    return physics_.bodies().CreateAndAddBody(settings, JPH::EActivation::DontActivate);
}

// A body we move ourselves, every step: it pushes boxes but nothing pushes it.
JPH::BodyID World::addKinematic(const Pose& pose, float friction) {
    JPH::BodyCreationSettings settings(makeBox(toJolt(pose.size / 2.0F)),
                                       JPH::RVec3(pose.center.x, pose.center.y, pose.center.z),
                                       JPH::Quat(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w),
                                       JPH::EMotionType::Kinematic, layers::kMoving);
    settings.mFriction = friction;
    settings.mAllowSleeping = false;
    return physics_.bodies().CreateAndAddBody(settings, JPH::EActivation::Activate);
}

std::size_t World::addPusher(const PusherSpec& spec) {
    Pusher pusher{spec, Stroke(spec.stroke), {}};
    pusher.body = addKinematic(pusherPaddle(spec, conveyors_.at(spec.conveyor).spec, 0.0), 0.4F);
    pushers_.push_back(pusher);
    return pushers_.size() - 1;
}

std::size_t World::addDiverter(const DiverterSpec& spec) {
    Diverter diverter{spec, Stroke(spec.stroke), {}};
    diverter.body =
        addKinematic(diverterArm(spec, conveyors_.at(spec.conveyor).spec, 0.0), 0.05F); // boxes slide along it
    diverters_.push_back(diverter);
    return diverters_.size() - 1;
}

std::size_t World::addStopper(const StopperSpec& spec) {
    Stopper stopper{spec, Stroke(spec.stroke), {}};
    stopper.body = addKinematic(stopperBlade(spec, conveyors_.at(spec.conveyor).spec, 0.0), 0.1F);
    stoppers_.push_back(stopper);
    return stoppers_.size() - 1;
}

std::size_t World::addBin(const BinSpec& spec) {
    Bin bin{spec, {}, 0, 0, {}};
    for (const auto& wall : binWalls(spec, conveyors_.at(spec.conveyor).spec)) {
        bin.walls.push_back(addStatic(wall, 0.6F));
    }
    bins_.push_back(bin);
    return bins_.size() - 1;
}

void World::setStrokes(const std::vector<PusherSpec>& pushers, const std::vector<DiverterSpec>& diverters,
                       const std::vector<StopperSpec>& stoppers) {
    for (std::size_t i = 0; i < pushers.size() && i < pushers_.size(); ++i) {
        pushers_[i].spec = pushers[i];
        pushers_[i].stroke.spec = pushers[i].stroke;
    }
    for (std::size_t i = 0; i < diverters.size() && i < diverters_.size(); ++i) {
        diverters_[i].spec = diverters[i];
        diverters_[i].stroke.spec = diverters[i].stroke;
    }
    for (std::size_t i = 0; i < stoppers.size() && i < stoppers_.size(); ++i) {
        stoppers_[i].spec = stoppers[i];
        stoppers_[i].stroke.spec = stoppers[i].stroke;
    }
}

std::size_t World::addPhotocell(const PhotocellSpec& spec) {
    Photocell photocell;
    photocell.spec = spec;
    photocells_.push_back(std::move(photocell));
    return photocells_.size() - 1;
}

void World::setPhotocells(const std::vector<PhotocellSpec>& specs) {
    photocells_.resize(specs.size());
    for (std::size_t i = 0; i < specs.size(); ++i) {
        if (photocells_[i].spec.conveyor != specs[i].conveyor) {
            photocells_[i] = Photocell{}; // on another conveyor: a different sensor, starting afresh
        }
        photocells_[i].spec = specs[i];
    }
}

void World::setSpawners(const std::vector<SpawnerSpec>& specs) {
    const auto before = spawners_.size();
    spawners_ = specs;
    spawnerStates_.resize(specs.size());
    for (auto i = before; i < specs.size(); ++i) {
        spawnerStates_[i].nextSpawn = time() + specs[i].interval; // a new one starts its own rhythm
    }
}

std::int64_t World::encoderCountAt(std::size_t conveyor, double t) const {
    const auto& c = conveyors_.at(conveyor);
    const double stepEnd = time();
    const double fraction = std::clamp(1.0 - (stepEnd - t) / kStep, 0.0, 1.0);
    const double travelled = c.travelledBefore + (c.travelled - c.travelledBefore) * fraction;
    return static_cast<std::int64_t>(std::floor(travelled * c.spec.encoder.pulsesPerMetre()));
}

std::size_t World::addSpawner(const SpawnerSpec& spec) {
    // An STL part is read and split into pieces now, while the line is
    // being built, not in the middle of the run when the first one is due.
    if (spec.shape != Shape::Box && spec.shape != Shape::Tin) {
        (void)itemGeometry(spec.shape, spec.size, spec.stl);
    }
    spawners_.push_back(spec);
    spawnerStates_.push_back(SpawnerState{time() + spec.interval, false, {}});
    return spawners_.size() - 1;
}

ItemId World::spawnItem(glm::vec3 position, glm::vec3 size, glm::vec3 color, float heading, bool nok, Shape shape,
                        glm::vec3 velocity, std::shared_ptr<const ItemGeometry> geometry) {
    if (shape == Shape::Tin) {
        size.z = size.x; // round
    }
    if (geometry == nullptr && shape != Shape::Box && shape != Shape::Tin) {
        geometry = itemGeometry(shape, size).value_or(nullptr);
        if (geometry == nullptr) {
            shape = Shape::Box; // an STL part without its file: a box in its place
        }
    }
    JPH::BodyCreationSettings settings(
        geometry != nullptr ? JPH::RefConst<JPH::Shape>(geometry->physicsShape()) : makeItemShape(shape, size),
        JPH::RVec3(position.x, position.y, position.z), JPH::Quat::sRotation(JPH::Vec3::sAxisY(), heading),
        JPH::EMotionType::Dynamic, layers::kMoving);
    if (geometry != nullptr) {
        // Its own material: how it slides on the belt and bounces off things.
        size = geometry->size();
        const auto& material = propertiesOf(geometry->material());
        settings.mFriction = material.friction;
        settings.mRestitution = material.restitution;
        // Jolt merges the contacts of a body's convex pieces when their
        // normals agree, to solve fewer. On a curved underside made of
        // pieces (a fish lying on the belt), the points it drops are the
        // ones holding it up, and it sinks into the belt: so, not for these.
        settings.mUseManifoldReduction = false;
    } else {
        settings.mFriction = shape == Shape::Tin ? 0.4F : 0.6F; // tinplate, cardboard
    }
    settings.mLinearVelocity = toJolt(velocity);
    // A box resting on a belt that has stopped would fall asleep, and Jolt
    // doesn't wake sleeping bodies when the belt starts again.
    settings.mAllowSleeping = false;
    const auto body = physics_.bodies().CreateAndAddBody(settings, JPH::EActivation::Activate);
    const auto id = nextItemId_++;
    items_.push_back(Item{id, body, size, color, -1.0, nok, shape, std::move(geometry)});
    return id;
}

glm::vec3 World::itemColor(Shape shape) const {
    switch (shape) {
    case Shape::Box:
    case Shape::Stl:
        return boxColor(nextItemId_);
    case Shape::Tin:
        return kTinColor;
    case Shape::Bottle:
        return {0.30F, 0.58F, 0.38F}; // green glass
    case Shape::Bag:
        return {0.95F, 0.78F, 0.22F};
    case Shape::Ball:
        return {0.80F, 0.90F, 0.25F};
    case Shape::Bracket:
        return {0.62F, 0.64F, 0.68F}; // steel
    case Shape::Fish:
        return {0.60F, 0.68F, 0.76F}; // silver under the ice
    }
    return boxColor(nextItemId_);
}

std::string World::spawnerProblem(std::size_t spawner) const {
    return spawner < spawnerStates_.size() ? spawnerStates_[spawner].problem : std::string();
}

std::pair<float, float> World::span(const Item& item, const glm::mat3& rotation, glm::vec3 direction) {
    if (item.geometry != nullptr) {
        return item.geometry->span(rotation, direction);
    }
    const float r = reach(item.shape, rotation, item.size / 2.0F, direction);
    return {-r, r};
}

float World::reachOf(const Item& item, const glm::mat3& rotation, glm::vec3 direction) {
    const auto [low, high] = span(item, rotation, direction);
    return std::max(-low, high);
}

void World::removeItem(ItemId id) {
    const auto it = std::find_if(items_.begin(), items_.end(), [&](const Item& item) { return item.id == id; });
    if (it == items_.end()) {
        return;
    }
    physics_.bodies().RemoveBody(it->body);
    physics_.bodies().DestroyBody(it->body);
    items_.erase(it);
    for (auto& bin : bins_) {
        std::erase(bin.inside, id); // it may come back as a new box with a new id, never this one
    }
}

void World::clearItems() {
    for (const auto& item : items_) {
        physics_.bodies().RemoveBody(item.body);
        physics_.bodies().DestroyBody(item.body);
    }
    items_.clear();
    for (auto& bin : bins_) {
        bin.inside.clear();
    }
}

int World::advance(double seconds, int maxSteps) {
    pending_ += seconds;
    int steps = 0;
    // A small tolerance, so 0.0105 s gives 10 steps despite floating point rounding.
    while (pending_ + 1e-9 >= kStep && steps < maxSteps) {
        step();
        pending_ -= kStep;
        ++steps;
    }
    if (steps == maxSteps) {
        pending_ = 0.0; // falling behind: drop the backlog instead of trying to catch up
    }
    return steps;
}

void World::run(double seconds) {
    const auto steps = static_cast<long>(std::llround(seconds / kStep));
    for (long i = 0; i < steps; ++i) {
        step();
    }
}

void World::step() {
    const double stepStart = time();
    if (stepCount_ % kScanSteps == 0) {
        scan(stepStart);
    }
    updateConveyors();
    updateCarriages(stepStart, stepStart + kStep);
    updateActuators(stepStart, stepStart + kStep);
    updateAirJets(stepStart, stepStart + kStep); // its push is applied in the physics step
    updateSpawners();
    physics_.step(static_cast<float>(kStep));
    removeFallenItems();
    ++stepCount_;
    updatePhotocells(stepStart, time()); // the world now is as it is at the end of the step
    updateZones(stepStart, time());
    updateCameras(time());
    updateFieldSensors(stepStart, time());
    updateBins();
}

void World::setPanel(const std::vector<ButtonSpec>& buttons, const std::vector<LampSpec>& lamps,
                     const std::vector<PotentiometerSpec>& potentiometers) {
    buttons_ = buttons;
    lamps_ = lamps;
    potentiometers_ = potentiometers;
    turned_.resize(potentiometers.size(), 0);
    pressed_.resize(buttons.size(), false);
    buttonSignals_.resize(buttons.size());
    lampSignals_.resize(lamps.size());
}

void World::press(std::size_t button, bool down) {
    if (button < pressed_.size()) {
        pressed_[button] = down;
        buttonSignals_[button].set(time(), down);
    }
}

void World::turn(std::size_t potentiometer, int value) {
    if (potentiometer < turned_.size()) {
        turned_[potentiometer] = std::clamp(value, 0, kAnalogFullScale);
    }
}

void World::tripDrive(std::size_t conveyor, std::string cause) {
    if (conveyor < conveyors_.size() && conveyors_[conveyor].spec.drive.fitted) {
        conveyors_[conveyor].drive.trip(std::move(cause));
    }
}

bool World::lit(std::size_t lamp) const {
    if (lamp >= lamps_.size()) {
        return false;
    }
    const auto address = parseAddress(lamps_[lamp].address);
    return address && io_.bit(*address);
}

// One scan, as a PLC does it: read every input into the image, run the
// program, apply its outputs. The outputs then hold until the next scan,
// so a program that sets a valve at 1.230 s moves it from 1.230 s on.
void World::scan(double time) {
    // ---- Inputs: what the devices show now.
    io_.inputs().fill(0);
    const auto write = [&](const std::string& text, std::uint32_t value) {
        if (const auto address = parseAddress(text); address && address->area == IoAddress::Area::Input) {
            io_.setValue(*address, value);
        }
    };
    for (const auto& photocell : photocells_) {
        write(photocell.spec.input, photocell.output.value() ? 1U : 0U);
    }
    for (const auto& conveyor : conveyors_) {
        write(conveyor.spec.encoderInput, static_cast<std::uint32_t>(conveyor.encoderCount())); // 32 bits, wrapping
    }
    const auto strokeInputs = [&](const auto& actuator) {
        write(actuator.spec.extendedInput, actuator.stroke.extended().value() ? 1U : 0U);
        write(actuator.spec.retractedInput, actuator.stroke.retracted().value() ? 1U : 0U);
    };
    std::for_each(pushers_.begin(), pushers_.end(), strokeInputs);
    std::for_each(diverters_.begin(), diverters_.end(), strokeInputs);
    std::for_each(stoppers_.begin(), stoppers_.end(), strokeInputs);
    for (const auto& camera : cameras_) {
        write(camera.spec.readyInput, camera.timing.busyAt(time) ? 0U : 1U);
    }
    for (const auto& conveyor : conveyors_) {
        if (conveyor.spec.drive.fitted) {
            write(conveyor.spec.drive.statusInput, conveyor.drive.statusWord());
            // A negative speed as the 16-bit two's complement the PLC reads as an INT.
            write(conveyor.spec.drive.actualInput, static_cast<std::uint16_t>(conveyor.drive.actualSpeed()));
        }
    }
    for (std::size_t i = 0; i < potentiometers_.size(); ++i) {
        write(potentiometers_[i].address, static_cast<std::uint32_t>(turned_[i]));
    }
    for (const auto& conveyor : conveyors_) {
        for (std::size_t z = 0; z < conveyor.eyes.size() && z < conveyor.spec.zoneIo.size(); ++z) {
            write(conveyor.spec.zoneIo[z].eyeInput, conveyor.eyes[z].output.value() ? 1U : 0U);
        }
        if (conveyor.home.carriage != Carriage::None) {
            write(conveyor.spec.carriageEndInput, conveyor.carriage.extended().value() ? 1U : 0U);
            write(conveyor.spec.carriageHomeInput, conveyor.carriage.retracted().value() ? 1U : 0U);
        }
    }
    for (const auto& proximity : proximities_) {
        write(proximity.spec.input, proximity.detector.output.value() ? 1U : 0U);
    }
    for (const auto& laser : lasers_) {
        write(laser.spec.output, laser.value);
    }
    for (const auto& curtain : curtains_) {
        write(curtain.spec.clearInput, curtain.clear.value() ? 1U : 0U);
        write(curtain.spec.countInput, static_cast<std::uint32_t>(curtain.blocked));
    }
    for (std::size_t i = 0; i < buttons_.size(); ++i) {
        // A normally closed contact is closed (TRUE) while the button is NOT pressed.
        write(buttons_[i].address, pressed(i) != buttons_[i].normallyClosed ? 1U : 0U);
    }

    // The belts' speeds, for the analyser's speed plot.
    for (auto& conveyor : conveyors_) {
        conveyor.speedSamples.emplace_back(time, conveyor.speed);
        if (conveyor.speedSamples.size() > Conveyor::kSpeedSamples) {
            conveyor.speedSamples.pop_front();
        }
    }

    if (!program_) {
        for (auto& conveyor : conveyors_) {
            if (conveyor.spec.drive.fitted) {
                // Nobody to send the telegram: the drive is run as if a PLC
                // said "run at the target speed", the way a belt without a
                // program always runs.
                const auto setpoint = static_cast<std::int16_t>(std::lround(
                    std::clamp(conveyor.spec.targetSpeed / conveyor.spec.drive.referenceSpeed, -1.99F, 1.99F) *
                    kFullScale));
                if (conveyor.drive.state() == Drive::State::SwitchingOnInhibited) {
                    conveyor.drive.command(stw1::kReady, setpoint);
                }
                conveyor.drive.command(stw1::kRun, setpoint);
                conveyor.running = conveyor.drive.state() == Drive::State::OperationEnabled;
            } else {
                conveyor.running = true;
            }
            conveyor.motor.set(time, conveyor.running);
            recordSetpoint(conveyor, time);
        }
        for (auto& lamp : lampSignals_) {
            lamp.set(time, false);
        }
        return; // no program: nothing drives the outputs
    }

    // ---- The program.
    program_->scan(io_, time);

    // ---- Outputs: valves and motors follow the image.
    const auto read = [&](const std::string& text) -> std::optional<bool> {
        if (const auto address = parseAddress(text); address && address->area == IoAddress::Area::Output) {
            return io_.bit(*address);
        }
        return std::nullopt;
    };
    const auto valve = [&](auto& actuator) {
        if (const auto open = read(actuator.spec.valveOutput)) {
            actuator.stroke.command(time, *open);
        }
    };
    std::for_each(pushers_.begin(), pushers_.end(), valve);
    std::for_each(diverters_.begin(), diverters_.end(), valve);
    std::for_each(stoppers_.begin(), stoppers_.end(), valve);
    for (auto& jet : airJets_) {
        if (const auto open = read(jet.spec.valveOutput)) {
            jet.command(time, *open);
        }
    }
    for (auto& conveyor : conveyors_) {
        if (conveyor.spec.drive.fitted) {
            // The telegram: two output words, read as the PLC wrote them.
            const auto word = [&](const std::string& text) -> std::uint16_t {
                const auto address = parseAddress(text);
                if (!address || address->area != IoAddress::Area::Output || address->size != IoAddress::Size::Word) {
                    return 0;
                }
                return static_cast<std::uint16_t>(io_.value(*address));
            };
            conveyor.drive.command(word(conveyor.spec.drive.controlOutput),
                                   static_cast<std::int16_t>(word(conveyor.spec.drive.setpointOutput)));
            conveyor.running = conveyor.drive.state() == Drive::State::OperationEnabled;
        } else {
            conveyor.running = read(conveyor.spec.runOutput).value_or(true); // no run output: it always runs
        }
        conveyor.motor.set(time, conveyor.running);
        recordSetpoint(conveyor, time);
        // Zones run by the PLC: one motor output each.
        if (conveyor.spec.zoneLogic == ZoneLogic::Plc) {
            for (std::size_t z = 0; z < conveyor.zoneRun.size(); ++z) {
                const auto motor =
                    z < conveyor.spec.zoneIo.size() ? read(conveyor.spec.zoneIo[z].motorOutput) : std::nullopt;
                conveyor.zoneRun[z] = motor.value_or(false);
            }
        }
        if (conveyor.home.carriage != Carriage::None) {
            if (const auto out = read(conveyor.spec.carriageOutput)) {
                conveyor.carriage.command(time, *out);
            }
        }
    }
    for (std::size_t i = 0; i < lamps_.size(); ++i) {
        lampSignals_[i].set(time, lit(i));
    }
    // A camera takes a picture on its trigger line's rising edge.
    for (auto& camera : cameras_) {
        if (const auto on = read(camera.spec.triggerOutput)) {
            const bool was = camera.trigger.value();
            camera.trigger.set(time, *on);
            if (*on && !was && camera.spec.kind == CameraKind::Area) {
                cameraTrigger(camera, time); // (a line-scan camera takes lines on its own clock)
            }
        }
    }
}

// The speed a drive was asked for, in m/s, for the analyser's speed plot.
void World::recordSetpoint(Conveyor& conveyor, double time) {
    if (!conveyor.spec.drive.fitted) {
        return;
    }
    const bool enabled = (conveyor.drive.controlWord() & stw1::kSetpointEnable) != 0;
    const float setpoint =
        enabled ? static_cast<float>(conveyor.drive.setpoint()) / kFullScale * conveyor.spec.drive.referenceSpeed
                : 0.0F;
    conveyor.setpointSamples.emplace_back(time, setpoint);
    if (conveyor.setpointSamples.size() > Conveyor::kSpeedSamples) {
        conveyor.setpointSamples.pop_front();
    }
}

// ---------------------------------------------------------------- Field sensors

void World::setFieldSensors(const std::vector<ProximitySpec>& proximities, const std::vector<LaserSpec>& lasers,
                            const std::vector<LightCurtainSpec>& curtains) {
    // Each list follows the specs; one that moves to another conveyor starts afresh.
    const auto follow = [](auto& running, const auto& specs) {
        running.resize(specs.size());
        for (std::size_t i = 0; i < specs.size(); ++i) {
            if (running[i].spec.conveyor != specs[i].conveyor) {
                running[i] = {};
            }
            running[i].spec = specs[i];
        }
    };
    follow(proximities_, proximities);
    follow(lasers_, lasers);
    follow(curtains_, curtains);
}

// Proximity switches, laser sensors and light curtains look at the items as
// they are at the end of the step, as photocells do.
void World::updateFieldSensors(double stepStart, double stepEnd) {
    if (proximities_.empty() && lasers_.empty() && curtains_.empty()) {
        return;
    }
    const auto& bodies = physics_.bodies();
    // Each item once: where it is, how it's turned, its half size, its
    // shape, whether it's metal.
    struct Seen {
        glm::vec3 center;
        glm::mat3 rotation;
        glm::vec3 half;
        Shape shape;
        const ItemGeometry* geometry; // null: a box or a tin
        bool metal;
    };
    std::vector<Seen> seen;
    seen.reserve(items_.size());
    for (const auto& item : items_) {
        const bool metal =
            item.geometry != nullptr ? propertiesOf(item.geometry->material()).metal : item.shape == Shape::Tin;
        seen.push_back(Seen{toGlm(JPH::Vec3(bodies.GetPosition(item.body))),
                            glm::mat3_cast(toGlm(bodies.GetRotation(item.body))), item.size / 2.0F, item.shape,
                            item.geometry.get(), metal});
    }
    // Whether an item could reach a point: within its bounding sphere.
    const auto near = [](const Seen& item, glm::vec3 point, float margin) {
        return glm::length(point - item.center) <= glm::length(item.half) + margin;
    };
    const glm::vec3 up{0.0F, 1.0F, 0.0F};

    for (auto& proximity : proximities_) {
        const auto& spec = proximity.spec;
        if (spec.conveyor >= conveyors_.size()) {
            continue;
        }
        const auto& conveyor = conveyors_[spec.conveyor];
        const auto origin = conveyor.beltPoint(0.0F);
        const auto along = conveyor.direction();
        const auto left = leftOf(along);
        const float faceSide = conveyor.spec.width / 2.0F - spec.inset; // beside: the face, from the centre line
        // The switching is a photocell's, whose "beam" is the face.
        auto& detector = proximity.detector;
        detector.spec.along = spec.along;
        detector.spec.beamDiameter = spec.faceDiameter;
        detector.spec.onFraction = 0.5F;
        detector.spec.offFraction = 0.3F;
        detector.spec.responseTime = spec.responseTime;
        std::vector<std::pair<float, float>> intervals;
        float nearest = -1.0F;
        // The face's axis: straight down from above, or in across the belt.
        auto face = conveyor.beltPoint(spec.along) + glm::vec3(0.0F, spec.height, 0.0F);
        glm::vec3 axis{0.0F, -1.0F, 0.0F};
        if (spec.mount != Mount::Above) {
            const float side = spec.mount == Mount::Left ? 1.0F : -1.0F;
            face += left * (side * faceSide);
            axis = -left * side;
        }
        for (const auto& item : seen) {
            const auto offset = item.center - origin;
            const float range = spec.range * rangeFactor(spec, item.metal) * (detector.beam ? 1.1F : 1.0F);
            if (item.geometry != nullptr) {
                // Its surface on the face's axis, within reach; along the
                // belt, it covers the face as far as it reaches.
                if (range <= 0.0F || !near(item, face, range + 0.01F)) {
                    continue;
                }
                const auto distance = item.geometry->castRay(item.center, item.rotation, face, axis, range + 0.01F);
                if (!distance || *distance > range) {
                    continue;
                }
                const float a = glm::dot(offset, along);
                const auto [low, high] = item.geometry->span(item.rotation, along);
                intervals.emplace_back(a + low, a + high);
                nearest = nearest < 0.0F ? *distance : std::min(nearest, *distance);
                continue;
            }
            const float lateral = glm::dot(offset, left);
            const float height = item.center.y - conveyor.spec.beltHeight;
            const float reachUp = reach(item.shape, item.rotation, item.half, up);
            const float reachSide = reach(item.shape, item.rotation, item.half, left);
            float distance = 0.0F;
            bool facing = false; // the item is in line with the face
            switch (spec.mount) {
            case Mount::Above:
                distance = spec.height - (height + reachUp);
                facing = std::abs(lateral) <= reachSide;
                break;
            case Mount::Left:
                distance = faceSide - (lateral + reachSide);
                facing = std::abs(height - spec.height) <= reachUp;
                break;
            case Mount::Right:
                distance = (lateral - reachSide) + faceSide;
                facing = std::abs(height - spec.height) <= reachUp;
                break;
            }
            // Seen within sn for its material; once on, 10 % further (the
            // hysteresis datasheets give).
            if (!facing || distance < -0.01F || distance > range) {
                continue;
            }
            const float a = glm::dot(offset, along);
            const float extent = reach(item.shape, item.rotation, item.half, along);
            intervals.emplace_back(a - extent, a + extent);
            nearest = nearest < 0.0F ? distance : std::min(nearest, distance);
        }
        proximity.distance = nearest;
        detector.sense(beamCoverage(spec.along, spec.faceDiameter, intervals), stepStart, kStep);
        while (detector.nextOutput(stepEnd)) {
        }
    }

    for (auto& laser : lasers_) {
        const auto& spec = laser.spec;
        if (spec.conveyor >= conveyors_.size()) {
            continue;
        }
        const auto& conveyor = conveyors_[spec.conveyor];
        // The spot on the belt; the highest surface over it is what the laser sees.
        const auto spot = conveyor.beltPoint(spec.along) + leftOf(conveyor.direction()) * spec.across;
        float top = 0.0F; // above the belt
        for (const auto& item : seen) {
            if (item.geometry != nullptr) {
                // Its surface straight down from the laser.
                const auto from = spot + glm::vec3(0.0F, spec.mountHeight, 0.0F);
                if (near(item, spot, spec.mountHeight)) {
                    if (const auto d = item.geometry->castRay(item.center, item.rotation, from, {0.0F, -1.0F, 0.0F},
                                                              spec.mountHeight)) {
                        top = std::max(top, spec.mountHeight - *d);
                    }
                }
                continue;
            }
            const auto local = glm::transpose(item.rotation) * (spot - item.center); // in the item's own axes
            const bool under = item.shape == Shape::Tin
                                   ? local.x * local.x + local.z * local.z <= item.half.x * item.half.x
                                   : std::abs(local.x) <= item.half.x && std::abs(local.z) <= item.half.z;
            if (under) {
                top = std::max(top, item.center.y - conveyor.spec.beltHeight +
                                        reach(item.shape, item.rotation, item.half, up));
            }
        }
        laser.trueDistance = spec.mountHeight - top;
        if (stepEnd + 1e-9 < laser.nextMeasurement) {
            continue; // between measurements: the output holds the last one
        }
        laser.nextMeasurement = stepEnd + spec.measureTime;
        float measured = laser.trueDistance;
        if (spec.noise > 0.0F) {
            std::normal_distribution<float> noise(0.0F, spec.noise);
            measured += noise(laser.random);
        }
        if (spec.resolution > 0.0F) {
            measured = std::round(measured / spec.resolution) * spec.resolution;
        }
        const bool inRange = measured >= spec.rangeMin && measured <= spec.rangeMax;
        laser.distance = inRange ? measured : -1.0F;
        laser.value = laserValue(spec, laser.distance);
        laser.samples.emplace_back(stepEnd, laser.distance);
        if (laser.samples.size() > Laser::kSamples) {
            laser.samples.pop_front();
        }
    }

    for (auto& curtain : curtains_) {
        const auto& spec = curtain.spec;
        if (spec.conveyor >= conveyors_.size() || stepEnd + 1e-9 < curtain.nextScan) {
            continue; // between scans: the outputs hold
        }
        curtain.nextScan = stepEnd + spec.scanTime;
        const auto& conveyor = conveyors_[spec.conveyor];
        const auto origin = conveyor.beltPoint(0.0F);
        const auto along = conveyor.direction();
        curtain.broken.assign(static_cast<std::size_t>(std::max(spec.beams, 0)), false);
        const auto left = leftOf(along);
        for (const auto& item : seen) {
            const float a = glm::dot(item.center - origin, along);
            if (item.geometry != nullptr) {
                // Each beam is a ray across the belt; it's broken if it hits.
                const auto [low, high] = item.geometry->span(item.rotation, along);
                if (spec.along < a + low || spec.along > a + high) {
                    continue; // not in the curtain's plane
                }
                const float half = conveyor.spec.width / 2.0F + 0.1F;
                for (std::size_t b = 0; b < curtain.broken.size(); ++b) {
                    const float beam = spec.lowest + static_cast<float>(b) * spec.pitch;
                    const auto from = conveyor.beltPoint(spec.along) + glm::vec3(0.0F, beam, 0.0F) + left * half;
                    if (!curtain.broken[b] &&
                        item.geometry->castRay(item.center, item.rotation, from, -left, 2.0F * half)) {
                        curtain.broken[b] = true;
                    }
                }
                continue;
            }
            if (std::abs(a - spec.along) > reach(item.shape, item.rotation, item.half, along)) {
                continue; // not in the curtain's plane
            }
            const float height = item.center.y - conveyor.spec.beltHeight;
            const float reachUp = reach(item.shape, item.rotation, item.half, up);
            for (std::size_t b = 0; b < curtain.broken.size(); ++b) {
                const float beam = spec.lowest + static_cast<float>(b) * spec.pitch;
                if (beam >= height - reachUp && beam <= height + reachUp) {
                    curtain.broken[b] = true;
                }
            }
        }
        curtain.blocked = static_cast<int>(std::count(curtain.broken.begin(), curtain.broken.end(), true));
        curtain.clear.set(stepEnd, curtain.blocked == 0);
    }
}

// ---------------------------------------------------------------- Air jets

void World::setAirJets(const std::vector<AirJetSpec>& specs) {
    airJets_.resize(specs.size());
    for (std::size_t i = 0; i < specs.size(); ++i) {
        if (airJets_[i].spec.conveyor != specs[i].conveyor) {
            airJets_[i] = AirJet(specs[i]); // on another conveyor: another jet
        }
        airJets_[i].spec = specs[i];
    }
}

void World::blowAirJet(std::size_t jet, double seconds) {
    if (jet < airJets_.size()) {
        airJets_[jet].blow(time(), seconds);
    }
}

namespace {

// A ray against a box of half size `half` around the origin, in the box's
// own axes (the slab method: where it enters each pair of faces; it's in
// the box from the last entry to the first exit). The distance and the
// face's normal.
std::optional<std::pair<float, glm::vec3>> rayBox(glm::vec3 from, glm::vec3 way, glm::vec3 half, float length) {
    float enter = 0.0F;
    float exit = length;
    glm::vec3 normal = -way;
    for (int axis = 0; axis < 3; ++axis) {
        if (std::abs(way[axis]) < 1e-9F) {
            if (std::abs(from[axis]) > half[axis]) {
                return std::nullopt; // parallel to this pair of faces, and outside them
            }
            continue;
        }
        float t1 = (-half[axis] - from[axis]) / way[axis];
        float t2 = (half[axis] - from[axis]) / way[axis];
        float sign = -1.0F; // entering through the low face
        if (t1 > t2) {
            std::swap(t1, t2);
            sign = 1.0F;
        }
        if (t1 > enter) {
            enter = t1;
            normal = glm::vec3(0.0F);
            normal[axis] = sign;
        }
        exit = std::min(exit, t2);
        if (enter > exit) {
            return std::nullopt;
        }
    }
    return std::pair{enter, normal};
}

// A ray against an upright cylinder (radius `radius` around its own y axis,
// half height `halfHeight`), in its own axes: its side or a lid.
std::optional<std::pair<float, glm::vec3>> rayCylinder(glm::vec3 from, glm::vec3 way, float radius, float halfHeight,
                                                       float length) {
    std::optional<std::pair<float, glm::vec3>> best;
    const auto keep = [&](float t, glm::vec3 n) {
        if (t >= 0.0F && t <= length && (!best || t < best->first)) {
            best = std::pair{t, n};
        }
    };
    // The side: x² + z² = r², between the lids.
    const float a = way.x * way.x + way.z * way.z;
    if (a > 1e-12F) {
        const float b = 2.0F * (from.x * way.x + from.z * way.z);
        const float c = from.x * from.x + from.z * from.z - radius * radius;
        const float d = b * b - 4.0F * a * c;
        if (d >= 0.0F) {
            const float t = (-b - std::sqrt(d)) / (2.0F * a);
            const auto p = from + way * t;
            if (std::abs(p.y) <= halfHeight) {
                keep(t, glm::normalize(glm::vec3(p.x, 0.0F, p.z)));
            }
        }
    }
    // The lids.
    if (std::abs(way.y) > 1e-9F) {
        for (const float lid : {-halfHeight, halfHeight}) {
            const float t = (lid - from.y) / way.y;
            const auto p = from + way * t;
            if (p.x * p.x + p.z * p.z <= radius * radius) {
                keep(t, glm::vec3(0.0F, lid > 0.0F ? 1.0F : -1.0F, 0.0F));
            }
        }
    }
    if (best && glm::dot(best->second, way) > 0.0F) {
        best->second = -best->second;
    }
    return best;
}

} // namespace

std::optional<World::ItemHit> World::hitItem(glm::vec3 origin, glm::vec3 direction, float length) const {
    const auto& bodies = physics_.bodies();
    std::optional<ItemHit> best;
    for (std::size_t i = 0; i < items_.size(); ++i) {
        const auto& item = items_[i];
        const auto center = toGlm(JPH::Vec3(bodies.GetPosition(item.body)));
        // Too far from the ray to be touched: past its bounding sphere.
        const float radius = glm::length(item.size) / 2.0F;
        const float along = glm::dot(center - origin, direction);
        if (along < -radius || along > length + radius || glm::length(center - (origin + direction * along)) > radius) {
            continue;
        }
        const auto rotation = glm::mat3_cast(toGlm(bodies.GetRotation(item.body)));
        std::optional<std::pair<float, glm::vec3>> hit; // distance, normal (in the world's axes)
        if (item.geometry != nullptr) {
            glm::vec3 normal;
            if (const auto d = item.geometry->castRay(center, rotation, origin, direction, length, &normal)) {
                hit = std::pair{*d, normal};
            }
        } else {
            const auto from = glm::transpose(rotation) * (origin - center);
            const auto way = glm::transpose(rotation) * direction;
            const auto local = item.shape == Shape::Tin
                                   ? rayCylinder(from, way, item.size.x / 2.0F, item.size.y / 2.0F, length)
                                   : rayBox(from, way, item.size / 2.0F, length);
            if (local) {
                hit = std::pair{local->first, rotation * local->second};
            }
        }
        if (hit && (!best || hit->first < best->distance)) {
            best = ItemHit{i, hit->first, origin + direction * hit->first, hit->second};
        }
    }
    return best;
}

// Each jet's streams push what they hit: at the point they hit it, at right
// angles to its surface, by the share of their thrust that's square to it.
// Jolt applies the forces in the step that follows.
void World::updateAirJets(double stepStart, double stepEnd) {
    auto& bodies = physics_.bodies();
    for (auto& jet : airJets_) {
        const double flowing = jet.update(stepStart, stepEnd);
        jet.lastPush = 0.0F;
        if (flowing <= 0.0 || jet.spec.conveyor >= conveyors_.size() || items_.empty()) {
            continue;
        }
        const auto& conveyor = conveyors_[jet.spec.conveyor].spec;
        constexpr float kReach = 0.8F; // m: past that, the jet has spread and slowed to nothing useful
        for (const auto& nozzle : nozzlesOf(jet.spec, conveyor)) {
            const auto streams = streamsAround(nozzle.direction, jet.spec.spread);
            const float share = thrustOf(jet.spec) * static_cast<float>(flowing) / static_cast<float>(streams.size());
            for (const auto& stream : streams) {
                const auto hit = hitItem(nozzle.position, stream, kReach);
                if (!hit) {
                    continue;
                }
                const float square = -glm::dot(stream, hit->normal); // 1: head on; 0: grazing
                if (square <= 0.0F) {
                    continue;
                }
                const auto force = -hit->normal * (share * square);
                bodies.AddForce(items_[hit->item].body, toJolt(force),
                                JPH::RVec3(hit->point.x, hit->point.y, hit->point.z));
                jet.lastPush += share * square;
            }
        }
    }
}

// ---------------------------------------------------------------- Cameras

std::size_t World::addCamera(const CameraSpec& spec) {
    Camera camera;
    camera.spec = spec;
    cameras_.push_back(std::move(camera));
    return cameras_.size() - 1;
}

// A line-scan camera's lines due in the step that ends at `stepEnd`: on its
// own clock, or when the belt's encoder has counted another `pulsesPerLine`
// (the moment found within the step, as the belt moves evenly in it).
void World::takeLines(Camera& camera, double stepEnd) {
    const auto& spec = camera.spec;
    if (spec.conveyor >= conveyors_.size()) {
        return;
    }
    const double stepStart = stepEnd - kStep;
    if (spec.lineTrigger == LineTrigger::FreeRun) {
        const double period = 1.0 / std::max(spec.lineRate, 1.0);
        camera.nextLine = std::max(camera.nextLine, stepStart);
        while (camera.nextLine <= stepEnd + 1e-12) {
            takeLine(camera, camera.nextLine, stepEnd);
            camera.nextLine += period;
        }
        return;
    }
    const auto& conveyor = conveyors_[spec.conveyor];
    const double perMetre = conveyor.spec.encoder.pulsesPerMetre();
    const auto per = std::max(spec.pulsesPerLine, 1);
    const double before = conveyor.travelledBefore;
    const double after = conveyor.travelled;
    // A camera that starts late starts at the next line from where the belt is.
    const auto first = static_cast<std::int64_t>(std::ceil(before * perMetre / per)) * per;
    camera.nextCount = std::max(camera.nextCount, first);
    // Forwards only: a belt running back takes no lines (it would draw over the image).
    while (after > before && static_cast<double>(camera.nextCount) / perMetre <= after) {
        const double at = static_cast<double>(camera.nextCount) / perMetre;
        if (at >= before) {
            takeLine(camera, stepStart + kStep * (at - before) / (after - before), stepEnd);
        }
        camera.nextCount += per;
    }
}

// One line: for each pixel across the belt, what's straight below it (the top
// of the highest item, or the belt), averaged over the line's exposure while
// the items move (motion blur), plus noise. Items are moved back (or on) from
// where they are at the end of the step to where they are then, at their
// present velocity, as for photocells.
void World::takeLine(Camera& camera, double time, double stepEnd) {
    const auto& spec = camera.spec;
    const auto& conveyor = conveyors_[spec.conveyor];
    const auto& bodies = physics_.bodies();
    const auto along = conveyor.direction();
    const auto left = leftOf(along);
    const auto lineMiddle = conveyor.beltPoint(spec.along);
    const int width = std::clamp(spec.pixelsAcross, 16, 4096);
    const float pixel = spec.lineWidth / static_cast<float>(width);
    // The items near the line, with what's needed to see their tops.
    struct Near {
        glm::vec3 center;
        glm::vec3 velocity;
        glm::mat3 rotation;
        glm::vec3 half;
        Shape shape;
        glm::vec3 color;
        const ItemGeometry* geometry; // null: a box or a tin
    };
    std::vector<Near> near;
    float fastest = 0.0F;
    for (const auto& item : items_) {
        const auto center = toGlm(JPH::Vec3(bodies.GetPosition(item.body)));
        const auto velocity = toGlm(bodies.GetLinearVelocity(item.body));
        if (std::abs(glm::dot(center - lineMiddle, along)) > glm::length(item.size) + 0.1F) {
            continue;
        }
        near.push_back(Near{center, velocity, glm::mat3_cast(toGlm(bodies.GetRotation(item.body))), item.size / 2.0F,
                            item.shape, item.color, item.geometry.get()});
        fastest = std::max(fastest, std::abs(glm::dot(velocity, along)));
    }
    // Enough samples over the exposure for the smear to look smooth: about
    // two per pixel it moves, up to 16.
    const float smear = fastest * static_cast<float>(spec.exposure) / pixel;
    const int samples = std::clamp(static_cast<int>(std::ceil(smear * 2.0F)), 1, 16);
    const glm::vec3 belt{0.18F, 0.19F, 0.21F};
    // A normal distribution needs a standard deviation above 0 (0 is
    // undefined behaviour): made only when there is noise.
    std::optional<std::normal_distribution<float>> noise;
    if (spec.noise > 0.0F) {
        noise.emplace(0.0F, spec.noise);
    }
    std::vector<std::uint8_t> line(static_cast<std::size_t>(width) * 3);
    for (int x = 0; x < width; ++x) {
        // Pixel 0 on the left of the belt, looking along the travel.
        const auto point = lineMiddle + left * (spec.lineWidth / 2.0F - (static_cast<float>(x) + 0.5F) * pixel);
        glm::vec3 sum{0.0F};
        for (int k = 0; k < samples; ++k) {
            const double t = time + spec.exposure * (static_cast<double>(k) + 0.5) / samples;
            float top = -1.0F;
            glm::vec3 color = belt;
            for (const auto& item : near) {
                const auto center = item.center - item.velocity * static_cast<float>(stepEnd - t);
                if (item.geometry != nullptr) {
                    // Its surface straight down from 1 m above the belt.
                    const auto flat = point - center;
                    if (glm::length(glm::vec2(flat.x, flat.z)) > glm::length(item.half)) {
                        continue; // too far to be under the pixel
                    }
                    const auto from = point + glm::vec3(0.0F, 1.0F, 0.0F);
                    if (const auto d =
                            item.geometry->castRay(center, item.rotation, from, {0.0F, -1.0F, 0.0F}, 1.0F + 1e-3F)) {
                        const float height = from.y - *d;
                        if (height > top) {
                            top = height;
                            color = item.color;
                        }
                    }
                    continue;
                }
                const auto local = glm::transpose(item.rotation) * (point - center);
                const bool under = item.shape == Shape::Tin
                                       ? local.x * local.x + local.z * local.z <= item.half.x * item.half.x
                                       : std::abs(local.x) <= item.half.x && std::abs(local.z) <= item.half.z;
                const float height = center.y + reach(item.shape, item.rotation, item.half, {0.0F, 1.0F, 0.0F});
                if (under && height > top) {
                    top = height;
                    color = item.color;
                }
            }
            sum += color;
        }
        const auto average = sum / static_cast<float>(samples) * 255.0F;
        for (int c = 0; c < 3; ++c) {
            const float value = average[c] + (noise ? (*noise)(camera.random) : 0.0F);
            line[static_cast<std::size_t>(x) * 3 + static_cast<std::size_t>(c)] =
                static_cast<std::uint8_t>(std::clamp(std::lround(value), 0L, 255L));
        }
    }
    auto& image = camera.image;
    if (image.width != width) {
        image.lines.clear(); // a new width: a new image
        image.width = width;
    }
    image.lines.push_back(std::move(line));
    if (image.lines.size() > LineImage::kLines) {
        image.lines.pop_front();
    }
    ++image.taken;
}

void World::setCameras(const std::vector<CameraSpec>& specs) {
    cameras_.resize(specs.size());
    for (std::size_t i = 0; i < specs.size(); ++i) {
        if (cameras_[i].spec.conveyor != specs[i].conveyor) {
            cameras_[i] = Camera{}; // on another conveyor: a different camera, starting afresh
        }
        cameras_[i].spec = specs[i];
    }
}

void World::triggerCamera(std::size_t camera) {
    if (camera < cameras_.size()) {
        cameraTrigger(cameras_[camera], time());
    }
}

// A trigger: taken if the camera is free, ignored if it's busy. Either way
// it's remembered, with the item it saw (or would have seen), which is
// decided when the exposure comes.
void World::cameraTrigger(Camera& camera, double time) {
    if (camera.busy.value() && !camera.timing.busyAt(time)) {
        camera.busy.set(camera.timing.busyUntil(), false); // it became free at the very moment
    }
    Shot shot;
    shot.triggerTime = time;
    shot.taken = camera.timing.trigger(time, camera.spec.busyTime());
    if (shot.taken) {
        camera.busy.set(time, true);
    }
    shot.exposure = time + camera.spec.triggerDelay + camera.spec.exposure / 2.0;
    camera.pending.push_back(shot);
}

// At the end of each step: cameras whose busy time is over are free again,
// and every exposure that happened by now is resolved: which item was in the
// picture, where, and how blurred. Items are moved back from where they are
// now to where they were at the exposure, at their present velocity, as for
// a photocell's trigger.
void World::updateCameras(double stepEnd) {
    const auto& bodies = physics_.bodies();
    for (auto& camera : cameras_) {
        if (camera.spec.kind == CameraKind::Line) {
            takeLines(camera, stepEnd);
            continue;
        }
        if (camera.busy.value() && !camera.timing.busyAt(stepEnd)) {
            camera.busy.set(camera.timing.busyUntil(), false);
        }
        const auto& spec = camera.spec;
        if (spec.conveyor >= conveyors_.size()) {
            camera.pending.clear();
            continue;
        }
        const auto& conveyor = conveyors_[spec.conveyor];
        const auto origin = conveyor.beltPoint(0.0F);
        const auto along = conveyor.direction();
        const auto side = leftOf(along);
        while (!camera.pending.empty() && camera.pending.front().exposure <= stepEnd + 1e-12) {
            auto shot = camera.pending.front();
            camera.pending.pop_front();
            float nearest = std::numeric_limits<float>::max();
            for (const auto& item : items_) {
                const auto velocity = toGlm(bodies.GetLinearVelocity(item.body));
                const auto center = toGlm(JPH::Vec3(bodies.GetPosition(item.body))) -
                                    velocity * static_cast<float>(stepEnd - shot.exposure);
                const auto offset = center - origin;
                if (std::abs(glm::dot(offset, side)) > conveyor.spec.width / 2.0F ||
                    center.y < conveyor.spec.beltHeight) {
                    continue; // not on this belt
                }
                const auto rotation = toGlm(bodies.GetRotation(item.body));
                const float extent = reachOf(item, glm::mat3_cast(rotation), along);
                const float a = glm::dot(offset, along) - spec.along; // from the middle of the picture
                if (std::abs(a) - extent >= spec.fieldOfView / 2.0F || std::abs(a) >= nearest) {
                    continue; // not in the picture, or another one is nearer its middle
                }
                nearest = std::abs(a);
                shot.item = item.id;
                shot.offset = a;
                shot.whole = std::abs(a) + extent <= spec.fieldOfView / 2.0F;
                shot.blur = std::abs(glm::dot(velocity, along)) * static_cast<float>(spec.exposure);
                shot.itemCenter = center;
                shot.itemRotation = rotation;
                shot.itemSize = item.size;
                shot.itemShape = item.shape;
                shot.itemGeometry = item.geometry;
            }
            camera.shots.push_front(shot);
            if (camera.shots.size() > Camera::kShotHistory) {
                camera.shots.pop_back();
            }
        }
    }
}

// Each actuator's stroke moves through the step, and its body is told where
// to be at the end of it. MoveKinematic gives the body the velocity that
// gets it there in one step, so it pushes boxes with that velocity, as a
// real paddle would.
void World::updateActuators(double stepStart, double stepEnd) {
    auto& bodies = physics_.bodies();
    const auto move = [&](JPH::BodyID body, const Pose& pose) {
        bodies.MoveKinematic(body, JPH::RVec3(pose.center.x, pose.center.y, pose.center.z),
                             JPH::Quat(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w),
                             static_cast<float>(kStep));
    };
    for (auto& pusher : pushers_) {
        pusher.stroke.update(stepStart, stepEnd);
        move(pusher.body, pusherPaddle(pusher.spec, conveyors_[pusher.spec.conveyor].spec, pusher.stroke.position()));
    }
    for (auto& diverter : diverters_) {
        diverter.stroke.update(stepStart, stepEnd);
        move(diverter.body,
             diverterArm(diverter.spec, conveyors_[diverter.spec.conveyor].spec, diverter.stroke.position()));
    }
    for (auto& stopper : stoppers_) {
        stopper.stroke.update(stepStart, stepEnd);
        move(stopper.body,
             stopperBlade(stopper.spec, conveyors_[stopper.spec.conveyor].spec, stopper.stroke.position()));
    }
}

// A bin counts each box once, when its centre comes inside the bin.
void World::updateBins() {
    if (bins_.empty()) {
        return;
    }
    const auto& bodies = physics_.bodies();
    for (auto& bin : bins_) {
        const auto inside = binInside(bin.spec, conveyors_[bin.spec.conveyor].spec);
        const auto toLocal = glm::conjugate(inside.rotation);
        for (const auto& item : items_) {
            const auto center = toGlm(JPH::Vec3(bodies.GetPosition(item.body)));
            const auto local = toLocal * (center - inside.center);
            const bool in = glm::all(glm::lessThanEqual(glm::abs(local), inside.size / 2.0F));
            if (in && std::find(bin.inside.begin(), bin.inside.end(), item.id) == bin.inside.end()) {
                bin.inside.push_back(item.id);
                ++bin.count;
                bin.nokCount += item.nok ? 1 : 0;
            }
        }
    }
}

// Each belt ramps its speed towards the target, like a VFD with a ramp time;
// a belt with a drive on the bus moves as its drive turns it.
void World::updateConveyors() {
    for (auto& conveyor : conveyors_) {
        if (!conveyor.zoneSpeed.empty()) {
            // Accumulation: each zone ramps on its own. In local logic the
            // zone cards decide, singulating: one box per zone. A box reaching
            // a zone's eye makes it full; when it leaves the eye, the zone is
            // empty again and the next one has a box arriving. An empty or
            // arriving zone runs (to take a box in, or to bring it to the
            // eye); a full one only passes its box on into an empty zone, or
            // (the last zone) while it's released. So boxes stop at their
            // zones' eyes and never push each other.
            const auto zones = conveyor.zoneSpeed.size();
            if (conveyor.spec.zoneLogic == ZoneLogic::Local) {
                auto& state = conveyor.zoneState;
                for (std::size_t k = 0; k < zones; ++k) {
                    const bool eye = conveyor.eyes[k].output.value();
                    if (eye && !conveyor.eyeBefore[k]) {
                        state[k] = ZoneState::Full;
                    } else if (!eye && conveyor.eyeBefore[k]) {
                        state[k] = ZoneState::Empty; // gone on, into the next zone (or off the end)
                        if (k + 1 < zones) {
                            state[k + 1] = ZoneState::Arriving;
                        }
                    }
                    conveyor.eyeBefore[k] = eye;
                }
                const auto release = parseAddress(conveyor.spec.releaseOutput);
                const bool released =
                    !program_ || !release || release->area != IoAddress::Area::Output || io_.bit(*release);
                for (std::size_t k = 0; k < zones; ++k) {
                    const bool accepts = k + 1 < zones ? state[k + 1] == ZoneState::Empty : released;
                    conveyor.zoneRun[k] = state[k] != ZoneState::Full || accepts;
                }
            }
            const float maxChange = conveyor.spec.acceleration * static_cast<float>(kStep);
            float sum = 0.0F;
            for (std::size_t z = 0; z < zones; ++z) {
                const float target = conveyor.running && conveyor.zoneRun[z] ? conveyor.spec.targetSpeed : 0.0F;
                conveyor.zoneSpeed[z] += std::clamp(target - conveyor.zoneSpeed[z], -maxChange, maxChange);
                sum += conveyor.zoneSpeed[z];
            }
            conveyor.speed = sum / static_cast<float>(zones); // on average, for the panels
            conveyor.travelledBefore = conveyor.travelled;
            conveyor.travelled += conveyor.speed * kStep;
            continue;
        }
        if (conveyor.spec.drive.fitted) {
            const auto& drive = conveyor.spec.drive;
            const double reference = std::max(drive.referenceSpeed, 0.01F);
            conveyor.drive.update(kStep, conveyor.spec.acceleration / reference, drive.quickStop / reference,
                                  drive.coast / reference);
            conveyor.speed = static_cast<float>(conveyor.drive.speed() * reference);
            conveyor.travelledBefore = conveyor.travelled;
            conveyor.travelled += conveyor.speed * kStep;
            continue;
        }
        const float maxChange = conveyor.spec.acceleration * static_cast<float>(kStep);
        const float target = conveyor.running ? conveyor.spec.targetSpeed : 0.0F; // stopped: ramps down
        const float difference = target - conveyor.speed;
        conveyor.speed += std::clamp(difference, -maxChange, maxChange);
        conveyor.travelledBefore = conveyor.travelled;
        conveyor.travelled += conveyor.speed * kStep;
    }
}

void World::updateSpawners() {
    for (std::size_t i = 0; i < spawners_.size(); ++i) {
        const auto& spec = spawners_[i];
        auto& state = spawnerStates_[i];
        if (!spec.enabled || time() + 1e-9 < state.nextSpawn || spec.conveyor >= conveyors_.size()) {
            continue;
        }
        const auto& conveyor = conveyors_[spec.conveyor];
        // Its items' size: from its settings, the jitter on their height
        // drawn below; an STL part's from its file.
        auto size = spec.size;
        std::shared_ptr<const ItemGeometry> geometry;
        if (spec.shape != Shape::Box && spec.shape != Shape::Tin) {
            auto made = itemGeometry(spec.shape, size, spec.stl);
            state.problem = made ? std::string() : made.error();
            if (!made) {
                state.nextSpawn = time() + spec.interval; // try again later (the file may come)
                continue;
            }
            geometry = *made;
            size = geometry->size();
        }
        const auto position = conveyor.beltPoint(spec.offset) + glm::vec3(0.0F, size.y / 2.0F + 0.002F, 0.0F);
        // An infeed waits for a gap: if a box is still where the new one would
        // go (the belt has stopped, or boxes are queueing back to here), it
        // waits, and the rhythm starts again from when it can put it down.
        const auto along = conveyor.direction();
        const auto& bodies = physics_.bodies();
        const bool occupied = std::any_of(items_.begin(), items_.end(), [&](const Item& item) {
            const auto offset = toGlm(JPH::Vec3(bodies.GetPosition(item.body))) - position;
            // In the way along the belt, across it, and in height: a short item
            // (a fish) and a tall one (a bottle) overlap however different
            // their middles' heights are.
            return std::abs(glm::dot(offset, along)) < (size.x + item.size.x) / 2.0F + 0.01F &&
                   std::abs(offset.y) < std::max(size.y, (size.y + item.size.y) / 2.0F) &&
                   std::abs(glm::dot(offset, leftOf(along))) < conveyor.spec.width / 2.0F;
        });
        if (occupied) {
            state.waiting = true;
            continue;
        }
        // Bad boxes are chosen at random, from the same seeded generator: the
        // same ones are bad every run.
        std::uniform_real_distribution<float> chance(0.0F, 1.0F);
        const bool nok = spec.nokFraction > 0.0F && chance(random_) < spec.nokFraction;
        const auto unjittered = size;
        if (spec.heightJitter > 0.0F && spec.shape != Shape::Stl) {
            std::uniform_real_distribution<float> height(-spec.heightJitter, spec.heightJitter);
            size.y = std::max(0.01F, size.y + height(random_));
            if (geometry != nullptr) {
                geometry = itemGeometry(spec.shape, size).value_or(geometry);
                size = geometry->size();
            }
        }
        const auto at = position + glm::vec3(0.0F, (size.y - unjittered.y) / 2.0F, 0.0F); // still resting on the belt
        spawnItem(at, size, nok ? kNokColor : itemColor(spec.shape), conveyor.spec.heading, nok, spec.shape,
                  spec.atBeltSpeed ? conveyor.direction() * conveyor.speedAt(spec.offset) : glm::vec3(0.0F), geometry);
        ++spawned_;
        spawnedNok_ += nok ? 1 : 0;

        std::uniform_real_distribution<double> jitter(-spec.intervalJitter, spec.intervalJitter);
        const double from = state.waiting ? time() : state.nextSpawn;
        state.waiting = false;
        state.nextSpawn = from + std::max(0.05, spec.interval + (spec.intervalJitter > 0.0 ? jitter(random_) : 0.0));
    }
}

// Boxes that fell off the end lie on the floor; after a few seconds they're
// cleared away, so the floor doesn't fill up.
void World::removeFallenItems() {
    std::vector<ItemId> toRemove;
    for (auto& item : items_) {
        const auto position = physics_.bodies().GetPosition(item.body);
        const bool onFloor = position.GetY() < item.size.y; // roughly: lying on the floor
        if (!onFloor) {
            item.onFloorSince = -1.0;
        } else if (item.onFloorSince < 0.0) {
            item.onFloorSince = time();
        } else if (time() - item.onFloorSince > 3.0) {
            toRemove.push_back(item.id);
        }
        if (position.GetY() < -5.0) {
            toRemove.push_back(item.id); // fell through something: just remove it
        }
    }
    for (const auto id : toRemove) {
        removeItem(id);
    }
}

// Each photocell looks at where the boxes are now, at the end of the step.
// The beam's coverage was something else at the start of the step; if it
// crossed the on or off threshold in between, the crossing time is found by
// interpolating between the two, since boxes move evenly within one 1 ms
// step. So switching times are exact to a fraction of a millisecond.
std::vector<std::pair<float, float>> World::beamIntervals(const Conveyor& conveyor, float height, bool onlyNok,
                                                          std::vector<std::size_t>* owners) const {
    const auto& bodies = physics_.bodies();
    const auto origin = conveyor.beltPoint(0.0F);
    const auto along = conveyor.direction();
    const glm::vec3 side{along.z, 0.0F, -along.x};
    const float beamY = conveyor.spec.beltHeight + height;
    // Each item's extent along the belt, if it's in the beam's path: across
    // the belt, and reaching the beam's height.
    std::vector<std::pair<float, float>> intervals;
    for (std::size_t i = 0; i < items_.size(); ++i) {
        const auto& item = items_[i];
        if (onlyNok && !item.nok) {
            continue; // an inspection sensor: good boxes don't trigger it
        }
        const auto center = toGlm(JPH::Vec3(bodies.GetPosition(item.body)));
        const auto rotation = glm::mat3_cast(toGlm(bodies.GetRotation(item.body)));
        const auto half = item.size / 2.0F;
        const auto offset = center - origin;
        const float a = glm::dot(offset, along);
        if (item.geometry != nullptr) {
            // Its real outline: what its cross-section at the beam's height
            // covers along the belt (nothing, if it doesn't reach that high).
            const auto [low, high] = item.geometry->span(rotation, {0.0F, 1.0F, 0.0F});
            if (center.y + low > beamY || center.y + high < beamY ||
                std::abs(glm::dot(offset, side)) >= conveyor.spec.width / 2.0F + reachOf(item, rotation, side)) {
                continue;
            }
            const auto first = intervals.size();
            item.geometry->slice(center, rotation, beamY, along, intervals);
            const float start = glm::dot(origin, along);
            for (auto k = first; k < intervals.size(); ++k) {
                intervals[k].first -= start;
                intervals[k].second -= start;
                if (owners != nullptr) {
                    owners->push_back(i);
                }
            }
            continue;
        }
        const float vertical = reach(item.shape, rotation, half, {0.0F, 1.0F, 0.0F});
        const bool atHeight = center.y - vertical <= beamY && beamY <= center.y + vertical;
        const bool acrossBelt =
            std::abs(glm::dot(offset, side)) < conveyor.spec.width / 2.0F + reach(item.shape, rotation, half, side);
        if (atHeight && acrossBelt) {
            const float extent = reach(item.shape, rotation, half, along);
            intervals.emplace_back(a - extent, a + extent);
            if (owners != nullptr) {
                owners->push_back(i);
            }
        }
    }
    return intervals;
}

// Each zone's photo-eye sees the end of its zone, as a photocell does.
void World::updateZones(double stepStart, double stepEnd) {
    for (auto& conveyor : conveyors_) {
        const auto zones = conveyor.eyes.size();
        for (std::size_t z = 0; z < zones; ++z) {
            auto& eye = conveyor.eyes[z];
            const float zoneLength = conveyor.spec.length / static_cast<float>(zones);
            eye.spec.along = zoneLength * static_cast<float>(z + 1) - std::min(0.03F, zoneLength / 4.0F);
            eye.spec.height = 0.04F;
            eye.spec.input = z < conveyor.spec.zoneIo.size() ? conveyor.spec.zoneIo[z].eyeInput : std::string();
            const auto intervals = beamIntervals(conveyor, eye.spec.height, false);
            eye.sense(beamCoverage(eye.spec.along, eye.spec.beamDiameter, intervals), stepStart, kStep);
            while (eye.nextOutput(stepEnd)) {
            }
        }
    }
}

// Carriages move like actuators; the belt and its guides go where the
// carriage puts them, given to Jolt as a target for the end of the step.
void World::updateCarriages(double stepStart, double stepEnd) {
    auto& bodies = physics_.bodies();
    for (auto& conveyor : conveyors_) {
        if (conveyor.home.carriage == Carriage::None) {
            continue;
        }
        conveyor.carriage.update(stepStart, stepEnd);
        const auto& home = conveyor.home;
        const auto now = carried(home, conveyor.carriage.position());
        conveyor.spec.start = now.start;
        conveyor.spec.heading = now.heading;
        conveyor.spec.beltHeight = now.beltHeight;
        const auto rotation = glm::angleAxis(now.heading, glm::vec3(0, 1, 0));
        const auto move = [&](JPH::BodyID body, glm::vec3 center) {
            bodies.MoveKinematic(body, JPH::RVec3(center.x, center.y, center.z),
                                 JPH::Quat(rotation.x, rotation.y, rotation.z, rotation.w), static_cast<float>(kStep));
        };
        const auto middle = conveyor.beltPoint(now.length / 2.0F);
        move(conveyor.body, middle - glm::vec3(0.0F, kBeltThickness / 2.0F, 0.0F));
        for (std::size_t g = 0; g < conveyor.guides.size(); ++g) {
            const auto& offset = conveyor.guideOffsets[g];
            move(conveyor.guides[g],
                 middle + glm::vec3(0.0F, offset.y, 0.0F) + leftOf(conveyor.direction()) * offset.z);
        }
    }
}

void World::setConveyorSpecs(const std::vector<ConveyorSpec>& specs) {
    for (std::size_t i = 0; i < specs.size() && i < conveyors_.size(); ++i) {
        auto& conveyor = conveyors_[i];
        conveyor.home = specs[i];
        conveyor.carriage.spec = specs[i].carriageStroke;
        // A carriage stays where it has moved to.
        conveyor.spec = carried(specs[i], conveyor.carriage.position());
    }
}

void World::updatePhotocells(double stepStart, double stepEnd) {
    const auto& bodies = physics_.bodies();
    for (auto& photocell : photocells_) {
        const auto& spec = photocell.spec;
        if (spec.conveyor >= conveyors_.size()) {
            continue;
        }
        const auto& conveyor = conveyors_[spec.conveyor];
        std::vector<std::size_t> owners; // which item each interval belongs to
        const auto intervals = beamIntervals(conveyor, spec.height, spec.onlyNok, &owners);
        const auto origin = conveyor.beltPoint(0.0F);
        const auto along = conveyor.direction();
        const auto beamTime = photocell.sense(beamCoverage(spec.along, spec.beamDiameter, intervals), stepStart, kStep);
        if (beamTime) {
            // The box it's about: the one covering the beam's centre, or the
            // nearest to it.
            Trigger trigger;
            float best = std::numeric_limits<float>::max();
            for (std::size_t k = 0; k < intervals.size(); ++k) {
                const auto [from, to] = intervals[k];
                const float distance = spec.along < from ? from - spec.along : spec.along > to ? spec.along - to : 0.0F;
                if (distance < best) {
                    best = distance;
                    trigger.item = items_[owners[k]].id;
                }
            }
            trigger.beamTime = *beamTime;
            photocell.pendingTriggers.push_back(trigger);
        }

        // Output changes whose response time has passed by the end of this step.
        while (const auto edge = photocell.nextOutput(stepEnd)) {
            if (!edge->value || photocell.pendingTriggers.empty()) {
                continue;
            }
            auto trigger = photocell.pendingTriggers.front();
            photocell.pendingTriggers.pop_front();
            trigger.outputTime = edge->time;
            trigger.encoderCount = encoderCountAt(spec.conveyor, edge->time);
            const auto it =
                std::find_if(items_.begin(), items_.end(), [&](const Item& item) { return item.id == trigger.item; });
            if (it != items_.end()) {
                // Where the box was at the output time: back from where it
                // is now, at its present velocity.
                const auto velocity = toGlm(bodies.GetLinearVelocity(it->body));
                const auto center = toGlm(JPH::Vec3(bodies.GetPosition(it->body)));
                const auto rotation = toGlm(bodies.GetRotation(it->body));
                trigger.itemCenter = center - velocity * static_cast<float>(stepEnd - edge->time);
                trigger.itemRotation = rotation;
                trigger.itemSize = it->size;
                trigger.itemShape = it->shape;
                trigger.itemGeometry = it->geometry;
                trigger.speed = glm::dot(velocity, along);
                const float front =
                    glm::dot(trigger.itemCenter - origin, along) + span(*it, glm::mat3_cast(rotation), along).second;
                trigger.positionError = front - spec.along;
            }
            photocell.triggers.push_front(trigger);
            if (photocell.triggers.size() > Photocell::kTriggerHistory) {
                photocell.triggers.pop_back();
            }
        }
    }
}

std::vector<ItemView> World::items() const {
    std::vector<ItemView> views;
    views.reserve(items_.size());
    const auto& bodies = physics_.bodies();
    for (const auto& item : items_) {
        views.push_back(ItemView{item.id, toGlm(JPH::Vec3(bodies.GetPosition(item.body))),
                                 toGlm(bodies.GetRotation(item.body)), item.size, item.color,
                                 toGlm(bodies.GetLinearVelocity(item.body)), item.nok, item.shape, item.geometry});
    }
    return views;
}

} // namespace fsim
