#pragma once
// Jolt.h (its configuration) must come before any other Jolt header, so
// the formatter isn't allowed to sort these.
// clang-format off
#include <Jolt/Jolt.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/PhysicsSystem.h>
// clang-format on
#include <memory>

namespace fsim {

// Object layers: which bodies can collide with which. Static bodies (the
// floor, conveyor frames) never collide with each other, only with moving
// ones (the boxes); moving bodies collide with everything.
namespace layers {
constexpr JPH::ObjectLayer kStatic = 0;
constexpr JPH::ObjectLayer kMoving = 1;
constexpr JPH::uint kCount = 2;
} // namespace layers

// A Jolt physics world, set up for factory_sim: two layers, gravity along -Y,
// and a single-threaded job system, so every run with the same inputs gives
// exactly the same result (reproducible tests and measurements).
class Physics {
public:
    Physics();
    ~Physics();
    Physics(const Physics&) = delete;
    Physics& operator=(const Physics&) = delete;

    JPH::PhysicsSystem& system() { return *system_; }
    JPH::BodyInterface& bodies() { return system_->GetBodyInterface(); }
    const JPH::BodyInterface& bodies() const { return system_->GetBodyInterface(); }

    void setContactListener(JPH::ContactListener* listener) { system_->SetContactListener(listener); }

    // Advances the simulation by dt seconds.
    void step(float dt);

private:
    class BroadPhaseLayers;
    class ObjectVsBroadPhaseFilter;
    class ObjectPairFilter;

    std::unique_ptr<BroadPhaseLayers> broadPhaseLayers_;
    std::unique_ptr<ObjectVsBroadPhaseFilter> objectVsBroadPhase_;
    std::unique_ptr<ObjectPairFilter> objectPairs_;
    std::unique_ptr<JPH::TempAllocatorImpl> tempAllocator_;
    std::unique_ptr<JPH::JobSystemSingleThreaded> jobs_;
    std::unique_ptr<JPH::PhysicsSystem> system_;
};

} // namespace fsim
