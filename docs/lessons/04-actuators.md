# Lesson 4: Transfers and actuators

Milestone 4 makes the line do something to the boxes: boxes cross from one conveyor to the next (straight on, or turned 90 degrees), **pushers** push them off into a **bin** that counts them, **diverters** steer them off the side, **stoppers** hold them back. Every actuator moves like the air cylinder that drives it in a real plant.

```
valve ────────┐
              ▼
           Stroke: dead time ─► steady travel ─► end sensors (exact times)
              │ position 0..1
              ▼
   pusherPaddle / diverterArm / stopperBlade   ◄── one formula per actuator
              │ pose                                      │
              ▼                                           ▼
   physics: MoveKinematic (pushes boxes)          view: drawn there
```

Try it: `./build/factory_sim`, select the conveyor, add a **Pusher** and then a **Bin** (it goes across from the pusher), select the pusher and press **Fire once** as a box passes.

---

## Part 1: How an air cylinder moves

### 1.1 Dead time, then a steady stroke
Most actuators on a packaging line are pneumatic: a valve lets compressed air into a cylinder, which pushes a rod out. Two things matter for timing:
- **Dead time.** After the valve switches, nothing moves for a moment: the valve's spool has to move, air has to flow through the hoses, and the pressure has to rise enough to overcome friction. Typically 10 to 30 ms.
- **Stroke time.** Then the rod travels at a fairly steady speed, set by flow restrictors. A real cylinder accelerates and is cushioned at the end; a constant speed is a good first model.

`Stroke` (`src/sim/actuators.hpp`) models exactly that. `command(time, extend)` switches the valve; each step, `update` waits out the dead time and then moves the position (0 = retracted, 1 = extended) at `1 / extendTime` strokes per second. Reversed halfway, it turns back from where it is, after the dead time again.

### 1.2 End position sensors
Real cylinders have **reed switches** (magnetic sensors on the outside of the cylinder) that say "fully out" and "fully back". A PLC uses them to know a stroke is done: retract the pusher once it's extended, raise an alarm if it's still not back after a second.

The stroke's `extended()` and `retracted()` are `Signal`s, with **exact** switching times like the photocell's: "retracted" goes off the moment the rod starts moving (after the dead time), "extended" comes on the moment it arrives, computed from the speed, not rounded to the step. A test checks a 15 ms dead time and a 100 ms stroke give the sensor edges at exactly 15 ms and 115 ms.

A bug found by the test: after 50 steps of moving back, the position was 1e-16 instead of 0 (floating point), so it never quite "arrived", and the retracted sensor never switched. The arrival check now allows 1e-9 of slack. In a model of a machine, a sensor that never switches because of the eighteenth decimal is the worst kind of bug: everything *looks* fine.

---

## Part 2: Three actuators, one movement

The three actuators are the same stroke, turned into different motions:

| Actuator | Moving part | What it does |
|---|---|---|
| **Pusher** | a paddle sliding across the belt | pushes a box off the far side |
| **Diverter** | an arm pivoted at the belt's edge, swinging out by its angle | boxes slide along it, off the far side |
| **Stopper** | a blade rising out of a slot in the belt | boxes stop against it while the belt slides under them |

Each has one function that says where its moving part is, at any stroke position (`pusherPaddle`, `diverterArm`, `stopperBlade`). The physics moves the body there, and the view draws it there: the same formula, so what you see is exactly what pushes. ("Derive, don't store", once more.)

### Kinematic bodies
The moving parts are **kinematic** bodies: nothing pushes them, but they move. Every step, `MoveKinematic` gives each one the velocity that takes it to its new pose by the end of the step. When the paddle meets a box, the engine sees a paddle moving at 0.9 m/s and pushes the box with it. Teleporting the paddle instead would put it *inside* the box with no velocity, and the engine would shove the box out in some random direction.

### Things the physics taught
- **A fast pusher throws boxes.** The first pusher crossed the belt in 0.12 s; the box left at almost 4 m/s and flew 1.8 m, right over the bin. On a real line the stroke time is tuned so the product drops into its chute. The default is now 0.5 s out, 0.3 s back.
- **Friction is a design parameter.** The diverter's arm has almost no friction (0.05), so boxes slide along it; the pusher's paddle has more (0.4), so it carries the box; the stopper's blade lets the box stand still while the belt slips under it.

---

## Part 3: Transfers between conveyors

### 3.1 Straight on
A box reaching the end of a conveyor tips over the 2 cm gap onto the next one. Nothing special is needed: both belts carry it, and the engine handles the moment it's on both.

### 3.2 Turned 90 degrees
The first attempt failed in an instructive way: the box was dragged **sideways off the first conveyor** while still almost entirely on it. Both belts were at the same height, so the moment the box's front edge touched the turned belt, that belt's surface (moving sideways, from the box's point of view) caught it.

Real right-angle transfers solve that with **height**: the second belt is a little lower, so the box only drops onto it once it has really left the first one. Then it needs something to stop it carrying on straight: a **guide** on the far side, which it hits and turns along. `addConveyorAfter` now does both for a turn: 2 cm lower, a guide on the far side. The engine was right; it was the model of the machine that was missing a detail every real installation has.

### 3.3 Guides
Any conveyor can have a guide (a rail along an edge) on its left, its right, or both (the inspector's checkboxes). Guides are static bodies, part of the layout: changing them rebuilds the world. A conveyor with a pusher must leave its far side open, or the box has nowhere to go.

---

## Part 4: The bin

A bin is five static boxes (a floor and four walls) beside the conveyor. It counts a box the first time the box's centre is inside it (`World::updateBins`), and remembers which boxes it has counted so a box bouncing around inside isn't counted twice. When a box is removed (cleared from the floor after 3 s), the bin forgets it: its ID is never reused, so it can't be confused with a later box.

---

## Part 5: The layout, again

Actuators and bins join the scene. Their **place** (conveyor, position, side, size, angle) is layout: their bodies were built there, so changing it builds a new world. Their **timing** (dead time, stroke times) and name are settings, applied live. `sameLayout` compares the actuators with timing and name blanked out (`placementOf`), so "same place" is decided by one generic function for all of them, with `if constexpr (requires { spec.stroke; })` handling the bin, which has no stroke.

The header dependency was also untangled: actuators need a conveyor's geometry, and the world needs the actuators. The conveyor's description moved to its own small header (`conveyor_spec.hpp`) that both include, instead of each including the other.

---

## Part 6: C++ and design

### 6.1 The compiler's to-do list
Adding four new kinds of selectable thing (`Pick::Kind::Pusher`, …) made the compiler warn at every `switch` that didn't handle them yet: delete, the inspector, keeping the selection valid, finding the selected conveyor. That's the closed-set argument from the design notes, working: a `switch` without a `default` over an `enum class` is a list the compiler checks for you.

### 6.2 `requires` inside `if constexpr`
```cpp
template <class Spec> Spec placementOf(Spec spec) {
    spec.name.clear();
    if constexpr (requires { spec.stroke; }) {
        spec.stroke = StrokeSpec{};
    }
    return spec;
}
```
One template for four spec types; the `requires` expression asks the compiler "does this type have a `stroke`?", and the branch only exists for those that do. Before C++20 this needed a traits class or overloads.

---

## Part 7: Testing

- `sim/actuators_test.cpp` (7 cases): the stroke's dead time, travel and exact sensor times; a stroke reversed halfway; a pusher pushing a box into the bin, which counts it; a diverter swung out (box off the far side) and swung back (box passes); a stopper holding a box still on a moving belt, then letting it go; a straight transfer; a turned transfer with its automatic guide and lower belt.
- `sim/scene_test.cpp` (2 new cases): actuators added at sensible places (a bin across from the pusher), saved and loaded exactly, removed with their conveyor; timing live, place and guides rebuilding.
- By eye: a line with a pusher and bin, a diverter, a stopper on a turned conveyor; boxes queueing against the stopper, turning at the corner, dropping into the bin.

---

## Review questions

**1.** A pusher has a 15 ms dead time and a 0.5 s stroke. The valve opens at t = 2.000 s. When does "retracted" switch off, and when does "extended" switch on?
<details><summary>Answer</summary>

"Retracted" off at 2.015 s, when the rod starts moving after the dead time; "extended" on at 2.515 s, after the half-second stroke.
</details>

**2.** Why are the paddle and the arm moved with `MoveKinematic`, and not just placed at their new position each step?
<details><summary>Answer</summary>

`MoveKinematic` gives the body the velocity that reaches the new pose by the end of the step, so the engine pushes boxes with the right speed and direction. Placing it would make it jump into the box with no velocity, and the engine would push the box out in an arbitrary direction.
</details>

**3.** A box coming off a conveyor onto one turned 90 degrees, at the same height, slid off the side of the first. What two details of real transfers fixed it?
<details><summary>Answer</summary>

The turned belt sits a little lower (2 cm), so the box only drops onto it once it has left the first belt; and a guide on its far side stops the box and turns it along the new belt.
</details>

**4.** You change a pusher's stroke time while boxes run. Does the run restart? And if you move the pusher 10 cm along the belt?
<details><summary>Answer</summary>

The stroke time is a setting: applied live, no restart. Moving it changes where its body is, which is layout: a new world is built and the run restarts.
</details>
