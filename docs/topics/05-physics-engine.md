# 5. Plugging in a physics engine

Boxes in factory_sim fall, tip over, slide, queue and land on each other. That's **rigid body physics**, and writing it well (stable stacks, no boxes passing through each other, friction that holds) is years of work. factory_sim uses **Jolt Physics**, the engine behind several big games, and keeps everything that's specific to a factory (belts, sensors, timing) in its own code. This note explains how an engine like Jolt works from the outside, and how factory_sim connects to it (`src/sim/physics.hpp`, `src/sim/world.cpp`).

---

## 1. What a rigid body engine does

Each **body** has a shape (here, always a box), a position and rotation, a velocity and spin, a mass, and a friction coefficient. Every step, the engine:

1. **moves** every body by its velocity and applies gravity;
2. **finds contacts**: which bodies touch or overlap now;
3. **solves** them: computes the pushes and friction forces that stop bodies from passing through each other and make them slide or stick;
4. writes the new positions and velocities back.

What it doesn't do is anything specific: it doesn't know what a conveyor or a sensor is. Those are rules factory_sim adds around the engine.

---

## 2. Static and dynamic

Jolt has **dynamic** bodies (moved by forces: the boxes) and **static** ones (never move: the floor and the belts). Static bodies are cheap: they're never moved or solved, only collided with.

A belt is static. But belts move boxes, so how? Section 4.

---

## 3. Finding contacts quickly: layers and the broad phase

Testing every pair of bodies for contact would cost n² checks per step. Engines split the search:
- the **broad phase** sorts bodies into a tree of bounding boxes, and quickly finds the pairs whose bounding boxes overlap;
- the **narrow phase** does the exact shape test, only on those pairs.

Jolt lets you sort bodies into **object layers** and say which layers can collide. factory_sim has two: static and moving. Static against static never collides (two belts touching is not an event), so those pairs are skipped before any test (`ObjectPairFilter`). Each object layer also maps to a broad phase layer (`BroadPhaseLayers`), so static bodies live in their own tree, which is built once and never updated.

That's a lot of small classes for "boxes collide with everything, belts only with boxes", and they're the price of an engine that has to support games with dozens of kinds of objects. `Physics` hides them: the rest of the code sees `bodies()` and `step()`.

---

## 4. How a static belt moves boxes

A real belt is a loop: its **surface** moves, the conveyor stays put. Jolt can model exactly that. A **contact listener** is called for every contact the engine finds (`OnContactAdded`, `OnContactPersisted`), and may adjust how that contact is solved. factory_sim's (`World::ConveyorContacts`) says: "this belt's surface moves at the belt's speed, in its direction":

```cpp
settings.mRelativeLinearSurfaceVelocity = surfaceVelocity(body2) - surfaceVelocity(body1);
```

The engine's friction solver then does the rest: it drags the box until it matches the surface's speed, exactly as friction does on a real belt. That gives the right behaviour for free: a box dropped on a running belt needs a moment to catch up; a box held back by a stopper slips; a box on a stopped belt stays put.

Which body is a belt? Every Jolt body carries a 64-bit **user data** field. Belts store `kConveyorTag | index`: bit 32 says "this is a conveyor", the low 32 bits say which one. The listener reads it back with two bit operations.

This is the general pattern for extending an engine: **callbacks** at the points where it makes decisions, rather than changing its code.

---

## 5. Things we move: kinematic bodies

Pushers, diverter arms and stopper blades are a third kind of body: **kinematic**. Like static bodies, nothing pushes them; unlike static ones, they move, because *we* move them. Every step, `World::updateActuators` advances each actuator's stroke (our pneumatic model: dead time, then a steady stroke) and calls:

```cpp
bodies.MoveKinematic(body, targetPosition, targetRotation, stepTime);
```

That doesn't teleport the body: it gives it exactly the velocity that gets it to the target by the end of the step. So when the paddle meets a box, the engine sees a body moving at, say, 0.9 m/s, and pushes the box accordingly; a teleported paddle would appear inside the box with no velocity, and the engine would shove the box out sideways, at random.

Where each part goes is one function per actuator (`pusherPaddle`, `diverterArm`, `stopperBlade`), used both for the physics and for drawing, so what you see is exactly what pushes. The stroke timing is ours (it's about air cylinders); the contacts it causes are the engine's.

A kinematic stopper blade rises through the belt, which is static, without any trouble: the engine never computes contacts between two bodies that aren't dynamic.

## 6. Engine details that bit

**Sleeping.** Engines put bodies to sleep when they haven't moved for a while, to save time; contacts with moving bodies wake them. A box on a *stopped* belt falls asleep. Starting the belt doesn't touch the box (it only changes the surface speed in the contact settings), so nothing wakes it, and the box stays put on a moving belt. Boxes are created with `mAllowSleeping = false`. The cost is small: a few hundred boxes always awake.

**Convex radius.** Jolt rounds box edges a little (5 cm by default) for faster, more stable collisions, and requires that rounding to be at most half the box's thinnest side. A 6 cm belt or a 5 mm box would fail an assertion. `makeBox` uses the smaller of the default and half the smallest side.

**One-time setup.** Jolt needs process-wide setup (an allocator, a factory of types) once, before any physics system exists. `initialiseJoltOnce` uses `std::call_once`, so creating several worlds (every test does) sets it up exactly once, even from several threads.

**Same height, wrong turn.** A box reaching a conveyor turned 90 degrees was dragged sideways off the first belt while still almost entirely on it: at the same height, the turned belt's moving surface caught its front edge. Real right-angle transfers put the second belt a little lower; so does `addConveyorAfter` (2 cm). The engine was right; the model of the machine was wrong.

**Fast pushers throw boxes.** A pusher crossing the belt in 0.12 s launched boxes at nearly 4 m/s, right over the bin. The stroke time decides where a pushed box lands; the default is now 0.5 s out.

**Include order.** `Jolt/Jolt.h` must come before any other Jolt header, because it sets up the configuration the others depend on. Formatting the code sorted the includes alphabetically once and broke the build with hundreds of errors; they're now fenced with `// clang-format off`.

---

## 7. Ours versus the engine's

| The engine does | factory_sim does |
|---|---|
| boxes falling, tipping, stacking, colliding | belt speeds and VFD ramps (`updateConveyors`) |
| friction between box and belt | which surface moves and how fast (the contact listener) |
| contacts and their forces | sensors: photocell coverage, exact switching times, encoders |
| | time: fixed 1 ms steps, the backlog, slow motion |
| | spawning boxes, clearing fallen ones |

The split follows one question: **is it about physics in general, or about this factory?** General physics is hard, well-known, and better left to a specialist library. The factory parts are simple, need to be exactly right (a response time is a response time), and are the whole point of the simulator, so they're ours, tested directly, and never fight the engine.

**Speed.** The engine is most of each step's cost. In a Debug build without optimisation, a step with three conveyors and a few boxes took 0.77 ms: too slow for real time. Compiling Jolt with `-O2` even in Debug (`target_compile_options(Jolt PRIVATE $<$<CONFIG:Debug>:-O2>)`) brought it to 0.10 ms. We debug our code, not Jolt's, so there's no reason to run it slowly.

---

## Review questions

**1.** A belt is a static body that never moves. How does it carry boxes?
<details><summary>Answer</summary>

A contact listener tells Jolt, for every contact with a belt, that the belt's surface moves at its speed. The engine's friction drags the box towards that surface speed, as on a real belt.
</details>

**2.** Why can two static bodies never collide in factory_sim, and what does that save?
<details><summary>Answer</summary>

The object layer filter says static never collides with static. Belts touching each other, or the floor, are never tested, and static bodies sit in their own broad phase tree that never needs updating.
</details>

**3.** What would go wrong if boxes could fall asleep?
<details><summary>Answer</summary>

A box on a stopped belt would fall asleep, and when the belt started, nothing would wake it (only the contact's surface speed changes), so it would stay put on a moving belt.
</details>

**4.** The photocell's response time is factory_sim's code, not Jolt's. Why?
<details><summary>Answer</summary>

It's specific to this factory, simple, and has to be exactly right; Jolt knows nothing about sensors. General physics (collisions, friction, stacking) is left to the engine; factory-specific behaviour stays in our code, where it's tested directly.
</details>
