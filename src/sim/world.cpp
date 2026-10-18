#include "sim/world.hpp"
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <algorithm>
#include <cmath>

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

// A body's user data says what it is: 0 for boxes and the floor, and
// conveyor index + 1 for conveyor belts.
constexpr JPH::uint64 kConveyorTag = 1ULL << 32;

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

glm::vec3 Conveyor::direction() const {
    return {std::cos(spec.heading), 0.0F, -std::sin(spec.heading)};
}

glm::vec3 Conveyor::beltPoint(float along) const {
    return spec.start + direction() * along + glm::vec3(0.0F, spec.beltHeight, 0.0F);
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
    for (const auto& conveyor : conveyors_) {
        bodies.RemoveBody(conveyor.body);
        bodies.DestroyBody(conveyor.body);
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

    conveyors_.push_back(conveyor);
    return index;
}

std::size_t World::addSpawner(const SpawnerSpec& spec) {
    spawners_.push_back(spec);
    spawnerStates_.push_back(SpawnerState{time() + spec.interval});
    return spawners_.size() - 1;
}

ItemId World::spawnItem(glm::vec3 position, glm::vec3 size, glm::vec3 color, float heading) {
    JPH::BodyCreationSettings settings(makeBox(toJolt(size / 2.0F)), JPH::RVec3(position.x, position.y, position.z),
                                       JPH::Quat::sRotation(JPH::Vec3::sAxisY(), heading), JPH::EMotionType::Dynamic,
                                       layers::kMoving);
    settings.mFriction = 0.6F; // cardboard
    // A box resting on a belt that has stopped would fall asleep, and Jolt
    // doesn't wake sleeping bodies when the belt starts again.
    settings.mAllowSleeping = false;
    const auto body = physics_.bodies().CreateAndAddBody(settings, JPH::EActivation::Activate);
    const auto id = nextItemId_++;
    items_.push_back(Item{id, body, size, color, -1.0});
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
}

void World::clearItems() {
    for (const auto& item : items_) {
        physics_.bodies().RemoveBody(item.body);
        physics_.bodies().DestroyBody(item.body);
    }
    items_.clear();
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
    updateConveyors();
    updateSpawners();
    physics_.step(static_cast<float>(kStep));
    removeFallenItems();
    ++stepCount_;
}

// Each belt ramps its speed towards the target, like a VFD with a ramp time.
void World::updateConveyors() {
    for (auto& conveyor : conveyors_) {
        const float maxChange = conveyor.spec.acceleration * static_cast<float>(kStep);
        const float difference = conveyor.spec.targetSpeed - conveyor.speed;
        conveyor.speed += std::clamp(difference, -maxChange, maxChange);
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
        spawnItem(position, spec.size, boxColor(nextItemId_), conveyor.spec.heading);

        std::uniform_real_distribution<double> jitter(-spec.intervalJitter, spec.intervalJitter);
        state.nextSpawn += std::max(0.05, spec.interval + (spec.intervalJitter > 0.0 ? jitter(random_) : 0.0));
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

std::vector<ItemView> World::items() const {
    std::vector<ItemView> views;
    views.reserve(items_.size());
    const auto& bodies = physics_.bodies();
    for (const auto& item : items_) {
        views.push_back(ItemView{item.id, toGlm(JPH::Vec3(bodies.GetCenterOfMassPosition(item.body))),
                                 toGlm(bodies.GetRotation(item.body)), item.size, item.color,
                                 toGlm(bodies.GetLinearVelocity(item.body))});
    }
    return views;
}

} // namespace fsim
