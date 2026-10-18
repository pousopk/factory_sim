#include "sim/world.hpp"
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
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

// How far a box (turned by `rotation`, half its size `half`) reaches from
// its centre along `direction`: each half-size, projected onto it.
float reach(const glm::mat3& rotation, glm::vec3 half, glm::vec3 direction) {
    return std::abs(glm::dot(rotation[0], direction)) * half.x + std::abs(glm::dot(rotation[1], direction)) * half.y +
           std::abs(glm::dot(rotation[2], direction)) * half.z;
}

// A body's user data says what it is: 0 for boxes and the floor, and
// conveyor index + 1 for conveyor belts.
constexpr JPH::uint64 kConveyorTag = 1ULL << 32;

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
    // The belt's surface velocity, if the body is a belt.
    JPH::Vec3 surfaceVelocity(const JPH::Body& body) const {
        const auto tag = body.GetUserData();
        if ((tag & kConveyorTag) == 0) {
            return JPH::Vec3::sZero();
        }
        const auto& conveyor = conveyors_[static_cast<std::size_t>(tag & 0xFFFFFFFFULL)];
        return toJolt(conveyor.direction() * conveyor.speed);
    }

    void apply(const JPH::Body& body1, const JPH::Body& body2, JPH::ContactSettings& settings) const {
        // "Body 2's surface velocity minus body 1's", in world space.
        settings.mRelativeLinearSurfaceVelocity = surfaceVelocity(body2) - surfaceVelocity(body1);
    }

    const std::vector<Conveyor>& conveyors_;
};

World::World() : contacts_(std::make_unique<ConveyorContacts>(conveyors_)) {
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
    const auto index = conveyors_.size();

    // The belt is a thin static box whose top surface is at beltHeight.
    const JPH::Vec3 halfSize(spec.length / 2.0F, kBeltThickness / 2.0F, spec.width / 2.0F);
    const auto center = conveyor.beltPoint(spec.length / 2.0F) - glm::vec3(0.0F, kBeltThickness / 2.0F, 0.0F);
    JPH::BodyCreationSettings belt(makeBox(halfSize), JPH::RVec3(center.x, center.y, center.z),
                                   JPH::Quat::sRotation(JPH::Vec3::sAxisY(), spec.heading), JPH::EMotionType::Static,
                                   layers::kStatic);
    belt.mFriction = 1.0F; // rubber
    belt.mUserData = kConveyorTag | index;
    conveyor.body = physics_.bodies().CreateAndAddBody(belt, JPH::EActivation::DontActivate);

    // Side guides: rails just outside the belt's edges, rising above it.
    const auto left = leftOf(conveyor.direction());
    for (const auto& [wanted, sign] : {std::pair{spec.guideLeft, 1.0F}, std::pair{spec.guideRight, -1.0F}}) {
        if (wanted) {
            const auto rail = conveyor.beltPoint(spec.length / 2.0F) +
                              left * sign * (spec.width / 2.0F + kGuideThickness / 2.0F) +
                              glm::vec3(0.0F, kGuideHeight / 2.0F, 0.0F);
            conveyor.guides.push_back(addStatic(Pose{rail,
                                                     glm::angleAxis(spec.heading, glm::vec3(0, 1, 0)),
                                                     {spec.length, kGuideHeight, kGuideThickness}},
                                                0.2F));
        }
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
    spawners_.push_back(spec);
    spawnerStates_.push_back(SpawnerState{time() + spec.interval});
    return spawners_.size() - 1;
}

ItemId World::spawnItem(glm::vec3 position, glm::vec3 size, glm::vec3 color, float heading, bool nok) {
    JPH::BodyCreationSettings settings(makeBox(toJolt(size / 2.0F)), JPH::RVec3(position.x, position.y, position.z),
                                       JPH::Quat::sRotation(JPH::Vec3::sAxisY(), heading), JPH::EMotionType::Dynamic,
                                       layers::kMoving);
    settings.mFriction = 0.6F; // cardboard
    // A box resting on a belt that has stopped would fall asleep, and Jolt
    // doesn't wake sleeping bodies when the belt starts again.
    settings.mAllowSleeping = false;
    const auto body = physics_.bodies().CreateAndAddBody(settings, JPH::EActivation::Activate);
    const auto id = nextItemId_++;
    items_.push_back(Item{id, body, size, color, -1.0, nok});
    return id;
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
    updateActuators(stepStart, stepStart + kStep);
    updateSpawners();
    physics_.step(static_cast<float>(kStep));
    removeFallenItems();
    ++stepCount_;
    updatePhotocells(stepStart, time()); // the world now is as it is at the end of the step
    updateBins();
}

void World::setPanel(const std::vector<ButtonSpec>& buttons, const std::vector<LampSpec>& lamps) {
    buttons_ = buttons;
    lamps_ = lamps;
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
            conveyor.running = true;
            conveyor.motor.set(time, true);
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
    for (auto& conveyor : conveyors_) {
        conveyor.running = read(conveyor.spec.runOutput).value_or(true); // no run output: it always runs
        conveyor.motor.set(time, conveyor.running);
    }
    for (std::size_t i = 0; i < lamps_.size(); ++i) {
        lampSignals_[i].set(time, lit(i));
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
            const auto center = toGlm(JPH::Vec3(bodies.GetCenterOfMassPosition(item.body)));
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

// Each belt ramps its speed towards the target, like a VFD with a ramp time.
void World::updateConveyors() {
    for (auto& conveyor : conveyors_) {
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
        const auto position = conveyor.beltPoint(spec.offset) + glm::vec3(0.0F, spec.size.y / 2.0F + 0.002F, 0.0F);
        // An infeed waits for a gap: if a box is still where the new one would
        // go (the belt has stopped, or boxes are queueing back to here), it
        // waits, and the rhythm starts again from when it can put it down.
        const auto along = conveyor.direction();
        const auto& bodies = physics_.bodies();
        const bool occupied = std::any_of(items_.begin(), items_.end(), [&](const Item& item) {
            const auto offset = toGlm(JPH::Vec3(bodies.GetCenterOfMassPosition(item.body))) - position;
            return std::abs(glm::dot(offset, along)) < (spec.size.x + item.size.x) / 2.0F + 0.01F &&
                   std::abs(offset.y) < spec.size.y &&
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
        spawnItem(position, spec.size, nok ? kNokColor : boxColor(nextItemId_), conveyor.spec.heading, nok);
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
        const auto position = physics_.bodies().GetCenterOfMassPosition(item.body);
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
void World::updatePhotocells(double stepStart, double stepEnd) {
    const auto& bodies = physics_.bodies();
    for (auto& photocell : photocells_) {
        const auto& spec = photocell.spec;
        if (spec.conveyor >= conveyors_.size()) {
            continue;
        }
        const auto& conveyor = conveyors_[spec.conveyor];
        const auto origin = conveyor.beltPoint(0.0F);
        const auto along = conveyor.direction();
        const glm::vec3 side{along.z, 0.0F, -along.x};
        const float beamY = conveyor.spec.beltHeight + spec.height;

        // Each box's extent along the belt, if it's in the beam's path:
        // across the belt, and reaching the beam's height.
        std::vector<std::pair<float, float>> intervals;
        std::vector<std::size_t> owners; // which item each interval belongs to
        for (std::size_t i = 0; i < items_.size(); ++i) {
            const auto& item = items_[i];
            if (spec.onlyNok && !item.nok) {
                continue; // an inspection sensor: good boxes don't trigger it
            }
            const auto center = toGlm(JPH::Vec3(bodies.GetCenterOfMassPosition(item.body)));
            const auto rotation = glm::mat3_cast(toGlm(bodies.GetRotation(item.body)));
            const auto half = item.size / 2.0F;
            const auto offset = center - origin;
            const float a = glm::dot(offset, along);
            const float vertical = reach(rotation, half, {0.0F, 1.0F, 0.0F});
            const bool atHeight = center.y - vertical <= beamY && beamY <= center.y + vertical;
            const bool acrossBelt =
                std::abs(glm::dot(offset, side)) < conveyor.spec.width / 2.0F + reach(rotation, half, side);
            if (atHeight && acrossBelt) {
                const float extent = reach(rotation, half, along);
                intervals.emplace_back(a - extent, a + extent);
                owners.push_back(i);
            }
        }
        const float before = photocell.covered;
        const float now = beamCoverage(spec.along, spec.beamDiameter, intervals);
        photocell.covered = now;

        // When, within the step, did the coverage pass `threshold`?
        const auto crossing = [&](float threshold) {
            const float fraction = now == before ? 1.0F : (threshold - before) / (now - before);
            return stepStart + kStep * std::clamp(static_cast<double>(fraction), 0.0, 1.0);
        };
        if (!photocell.beam && now > spec.onFraction) {
            photocell.beam = true;
            const double beamTime = crossing(spec.onFraction);
            photocell.pending.push_back(Edge{beamTime + spec.responseTime, true});
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
            trigger.beamTime = beamTime;
            photocell.pendingTriggers.push_back(trigger);
        } else if (photocell.beam && now < spec.offFraction) {
            photocell.beam = false;
            photocell.pending.push_back(Edge{crossing(spec.offFraction) + spec.responseTime, false});
        }

        // Output changes whose response time has passed by the end of this step.
        while (!photocell.pending.empty() && photocell.pending.front().time <= stepEnd + 1e-12) {
            const auto edge = photocell.pending.front();
            photocell.pending.pop_front();
            photocell.output.set(edge.time, edge.value);
            if (!edge.value || photocell.pendingTriggers.empty()) {
                continue;
            }
            auto trigger = photocell.pendingTriggers.front();
            photocell.pendingTriggers.pop_front();
            trigger.outputTime = edge.time;
            trigger.encoderCount = encoderCountAt(spec.conveyor, edge.time);
            const auto it =
                std::find_if(items_.begin(), items_.end(), [&](const Item& item) { return item.id == trigger.item; });
            if (it != items_.end()) {
                // Where the box was at the output time: back from where it
                // is now, at its present velocity.
                const auto velocity = toGlm(bodies.GetLinearVelocity(it->body));
                const auto center = toGlm(JPH::Vec3(bodies.GetCenterOfMassPosition(it->body)));
                const auto rotation = toGlm(bodies.GetRotation(it->body));
                trigger.itemCenter = center - velocity * static_cast<float>(stepEnd - edge.time);
                trigger.itemRotation = rotation;
                trigger.itemSize = it->size;
                trigger.speed = glm::dot(velocity, along);
                const float extent = reach(glm::mat3_cast(rotation), it->size / 2.0F, along);
                const float front = glm::dot(trigger.itemCenter - origin, along) + extent;
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
        views.push_back(ItemView{item.id, toGlm(JPH::Vec3(bodies.GetCenterOfMassPosition(item.body))),
                                 toGlm(bodies.GetRotation(item.body)), item.size, item.color,
                                 toGlm(bodies.GetLinearVelocity(item.body)), item.nok});
    }
    return views;
}

} // namespace fsim
