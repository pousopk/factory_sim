#pragma once
// Jolt.h (its configuration) must come before any other Jolt header, so
// the formatter isn't allowed to sort these.
// clang-format off
#include <Jolt/Jolt.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/PhysicsSystem.h>
// clang-format on
#include <memory>

namespace fsim {

// Jolt's process-wide setup (its allocator, the factory for its types):
// done once, before anything of Jolt's is made. Physics does it; so does
// anything that makes Jolt shapes on its own (item shapes).
void initialiseJolt();

// Object layers: which bodies can collide with which. Static bodies (the
// floor, conveyor frames) never collide with each other, only with moving
// ones (the boxes); moving bodies collide with everything.
namespace layers {
constexpr JPH::ObjectLayer kStatic = 0;
constexpr JPH::ObjectLayer kMoving = 1;
constexpr JPH::uint kCount = 2;
} // namespace layers

// A Jolt physics world, set up for factory_sim: two layers, gravity along -Y.
//
// Jolt splits a step into jobs (finding collisions, solving each island of
// touching bodies) and runs them on a job system: one thread, or a pool.
// Its results don't depend on how many threads ran the jobs, only on the
// calls made to it and their order, so every run with the same inputs gives
// exactly the same result either way (a test checks it).
class Physics {
public:
    // `threads`: 1 for no extra threads; 0 to use the machine's cores (up to 8).
    explicit Physics(int threads = 0);
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
    std::unique_ptr<JPH::JobSystem> jobs_;
    int threads_ = 1;

public:
    int threads() const { return threads_; } // the threads that run its jobs, the caller's included
    std::unique_ptr<JPH::PhysicsSystem> system_;
};

} // namespace fsim
