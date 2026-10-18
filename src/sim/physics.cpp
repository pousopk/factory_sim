#include "sim/physics.hpp"
#include <Jolt/Core/Factory.h>
#include <Jolt/RegisterTypes.h>
#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <mutex>
#include <thread>

namespace fsim {

namespace {

// Jolt needs some process-wide setup (its allocator, a factory for its
// types) exactly once, before any physics system is created.
void trace(const char* format, ...) {
    va_list args;
    va_start(args, format);
    std::vfprintf(stderr, format, args);
    va_end(args);
    std::fputc('\n', stderr);
}

#ifdef JPH_ENABLE_ASSERTS
bool assertFailed(const char* expression, const char* message, const char* file, JPH::uint line) {
    std::fprintf(stderr, "Jolt assertion failed: %s (%s) at %s:%u\n", expression, message ? message : "", file, line);
    return true; // break: stop the program here
}
#endif

void initialiseJoltOnce() {
    static std::once_flag once;
    std::call_once(once, [] {
        JPH::Trace = trace;
#ifdef JPH_ENABLE_ASSERTS
        JPH::AssertFailed = assertFailed;
#endif
        JPH::RegisterDefaultAllocator();
        JPH::Factory::sInstance = new JPH::Factory(); // lives as long as the process
        JPH::RegisterTypes();
    });
}

constexpr JPH::BroadPhaseLayer kStaticBroadPhase{0};
constexpr JPH::BroadPhaseLayer kMovingBroadPhase{1};

} // namespace

// The broad phase is Jolt's first, coarse collision pass. Each object layer
// maps to a broad phase layer; here simply one each.
class Physics::BroadPhaseLayers final : public JPH::BroadPhaseLayerInterface {
public:
    JPH::uint GetNumBroadPhaseLayers() const override { return 2; }
    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer layer) const override {
        return layer == layers::kStatic ? kStaticBroadPhase : kMovingBroadPhase;
    }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer layer) const override {
        return layer == kStaticBroadPhase ? "static" : "moving";
    }
#endif
};

class Physics::ObjectVsBroadPhaseFilter final : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer layer, JPH::BroadPhaseLayer broadPhase) const override {
        return layer == layers::kMoving || broadPhase == kMovingBroadPhase;
    }
};

class Physics::ObjectPairFilter final : public JPH::ObjectLayerPairFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer a, JPH::ObjectLayer b) const override {
        return a == layers::kMoving || b == layers::kMoving; // static vs static: never
    }
};

Physics::Physics(int threads) {
    initialiseJoltOnce();
    broadPhaseLayers_ = std::make_unique<BroadPhaseLayers>();
    objectVsBroadPhase_ = std::make_unique<ObjectVsBroadPhaseFilter>();
    objectPairs_ = std::make_unique<ObjectPairFilter>();
    tempAllocator_ = std::make_unique<JPH::TempAllocatorImpl>(16 * 1024 * 1024);
    if (threads <= 0) {
        // All but one core (for the window and the rest), and no more than 8:
        // past that, a few hundred boxes don't give each thread enough to do.
        threads = std::clamp(static_cast<int>(std::thread::hardware_concurrency()) - 1, 1, 8);
    }
    threads_ = threads;
    if (threads == 1) {
        jobs_ = std::make_unique<JPH::JobSystemSingleThreaded>(JPH::cMaxPhysicsJobs);
    } else {
        // The pool's own threads, plus the caller, which works too while it waits.
        jobs_ = std::make_unique<JPH::JobSystemThreadPool>(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, threads - 1);
    }

    system_ = std::make_unique<JPH::PhysicsSystem>();
    constexpr JPH::uint maxBodies = 8192;
    constexpr JPH::uint bodyMutexes = 0; // 0 = Jolt's default
    constexpr JPH::uint maxBodyPairs = 8192;
    constexpr JPH::uint maxContactConstraints = 8192;
    system_->Init(maxBodies, bodyMutexes, maxBodyPairs, maxContactConstraints, *broadPhaseLayers_, *objectVsBroadPhase_,
                  *objectPairs_);
}

Physics::~Physics() = default;

void Physics::step(float dt) {
    constexpr int collisionSteps = 1;
    system_->Update(dt, collisionSteps, tempAllocator_.get(), jobs_.get());
}

} // namespace fsim
