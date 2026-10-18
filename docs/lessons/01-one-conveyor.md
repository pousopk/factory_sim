# Lesson 1: One conveyor in 3D

Milestone 1 builds the foundation everything else stands on: a simulated world that advances in exact 1 ms steps, a conveyor that carries boxes the way a real belt does, and a 3D view of it with an orbit camera.

```
             every frame (about 60 per second)
real time ──► App::frame ──► World::advance(real time × time scale)
                                 │  as many 1 ms steps as fit
                                 ▼
                              World::step:  ramp belts → spawn boxes → Jolt physics → clear fallen boxes
                                 │
              sceneBoxes(world) ─┘  a list of coloured boxes
                     │
              Renderer::draw ──► OpenGL       panels ──► ImGui
```

Try it: `./build/factory_sim`. Pause, then press **Step 1 ms** a few times: at 0.5 m/s, each box moves exactly 0.5 mm per step.

---

## Part 1: Simulated time

### 1.1 A fixed step, whatever the frame rate
The simulation always advances in steps of exactly **1 ms** (`World::kStep`). The screen draws a frame every 16 ms or so, but that number changes (a slow computer, a busy moment, a different monitor). If the physics used the frame time as its step, a box would move in 16 ms jumps on one computer and 7 ms jumps on another, and every timing effect we want to measure later (a photocell switching, a camera triggering) would depend on the screen.

So each frame, `advance(seconds)` adds the real time that passed to a **backlog** (`pending_`) and runs as many whole 1 ms steps as fit, keeping the remainder for next time. This is called a *fixed timestep with an accumulator*. At 2 m/s, a box moves exactly 2 mm per step, on every computer, every run.

### 1.2 Not falling behind forever
If one step took longer than 1 ms of real time to compute, each frame would leave a bigger backlog than the last, and the program would freeze trying to catch up (sometimes called the *spiral of death*). `advance` runs at most 250 steps per call and then drops the rest. The simulation then runs slower than real time, but the window stays responsive. Long pauses (the window being dragged) are capped at a quarter of a second for the same reason.

### 1.3 Pause, step, slow motion
Because simulated time is separate from real time, the app can scale it: **0.1x** passes a tenth of the real time to `advance`, and **Pause** passes none. The physics doesn't change, only how fast we watch it. **Step 1 ms** calls `step()` once: the smallest moment the simulation knows.

### 1.4 Exactly when is "at 2.0 s"?
A test expected 4 boxes after `run(2.0)` from a spawner every 0.5 s. There were 3. The steps start at 0 ms, 1 ms, … 1999 ms; the spawn due at 2.0 s belongs to the step that **starts** at 2.0 s, which `run(2.0)` hasn't run yet. The test now says so, and checks the 4th box after one more step. Off-by-one questions like this one are where simulations go subtly wrong: deciding exactly what a step covers, and writing it down, prevents them.

---

## Part 2: The physics

### 2.1 Jolt for collisions, our own code for the machines
Jolt Physics simulates rigid bodies: boxes that fall, tip over, collide and stack. Writing that ourselves would be a project on its own. But a conveyor's speed, a VFD's ramp, later a photocell's timing, are **ours**: simple, exactly controlled, and the point of the simulator. That's the hybrid the README decided on.

Jolt is set up for **reproducible** results (`physics.hpp`): one thread (`JobSystemSingleThreaded`) and a fixed step, so the same inputs give exactly the same motion every run. Tests can then check positions with tight tolerances, and a measurement can be repeated.

### 2.2 How a belt carries a box
The belt is a **static** body: it never moves. What moves is its *surface*. Jolt calls a contact listener for every pair of touching bodies, and the listener (`World::ConveyorContacts`) tells it: "this belt's surface moves at the belt's speed, in its direction". Jolt's friction then drags the box along, exactly like a real belt. The box needs a moment to catch up (the tests allow for it), it can slip if it's pushed, and it stops when the belt stops.

Two details that came from real problems:
- **Sleeping is off for boxes.** Jolt puts bodies that haven't moved for a while to sleep, to save time. A box on a stopped belt would fall asleep, and Jolt wouldn't wake it when the belt started again: it isn't a collision, just a surface speed. So boxes never sleep.
- **Thin boxes need small rounding.** Jolt rounds the edges of box shapes (the *convex radius*, 5 cm by default), which can't be more than half the box's thinnest side. A 6 cm belt gets a smaller radius (`makeBox`).

### 2.3 A VFD ramp
Real conveyor motors are driven by a VFD, which ramps the speed up and down instead of jumping. Each step, the belt's speed moves towards its target by at most `acceleration × 1 ms` (`updateConveyors`). Set 1 m/s² and a target of 0.5 m/s: it takes 0.5 s to get there, and the progress bar in the panel shows it.

### 2.4 Units and axes
Everything is in metres, seconds and kilograms. Y is up and the floor is y = 0. A conveyor's `heading` turns it around Y: 0 runs along +X. Writing the units down once, at the top of `world.hpp`, saves a lot of guessing later (is that speed in m/s or mm/s?).

---

## Part 3: Drawing it

### 3.1 The scene is just geometry
`sceneBoxes(world)` (`render/scene.cpp`) turns the world into a list of `DrawBox`es: a centre, a rotation, a size, a colour. A conveyor becomes seven boxes (belt, two side frames, four legs) and each item one more. No OpenGL is involved, so it's tested like the simulation: the belt's top is at the height the physics uses, the legs stand on the floor, a turned conveyor's parts turn with it.

The renderer then only knows how to draw a list of boxes. This split keeps the hard-to-test part (OpenGL) as small and dumb as possible, the same idea as in `ladder_editor`'s layout and rung view.

### 3.2 One cube, many boxes
There's a single **unit cube** in the GPU's memory: 36 corners (6 faces × 2 triangles × 3), each with a normal. Every box is that cube with a **model matrix**: scale it to the box's size, turn it, move it into place. The camera adds a **view** matrix (where the camera is) and a **projection** matrix (perspective). Multiply them together and each corner lands on the screen: `projection × view × model × corner`.

### 3.3 The shaders
Two small GLSL programs run on the GPU:
- **Boxes:** light from a sun plus a soft sky (brighter from above), and darker edges so boxes of the same colour stay readable. On a belt, light **stripes** move with the surface, offset by how far the belt has travelled, so you can *see* its speed.
- **Floor:** a grid every metre and every 10 cm, drawn in the shader with `fwidth` so lines stay one pixel wide at any distance, fading far away.

### 3.4 Keeping the stripes precise
The belt's total travel is a `double` that grows forever. The GPU works in `float`s, which have about 7 significant digits: after a few hours at 2 m/s the travel is tens of kilometres, and a float of that size can't hold millimetres any more, so the stripes would start to jump. The scene sends only `fmod(travelled, spacing)`, which is always less than 25 cm: small numbers keep their precision.

### 3.5 The orbit camera
The camera sits on a sphere around a target point: `yaw` turns it around, `pitch` raises it, `distance` is the sphere's radius (spherical coordinates). Left drag changes yaw and pitch; a drag across the window's height is half a turn, whatever the window's size. Right drag moves the target in the plane of the screen, scaled so the point under the mouse follows it. The pitch stops just short of straight up or down, where "up" stops meaning anything and the view would flip. It's all `glm` maths, tested without a window.

---

## Part 4: C++ concepts

### 4.1 Move-only owners of OpenGL objects
A `Shader` owns an OpenGL program, a number handed out by the driver. Copying a `Shader` would give two objects that both delete that number. So copying is deleted and moving transfers ownership (`std::exchange(other.program_, 0)` leaves the moved-from shader owning nothing). It's the same RAII as `unique_ptr`, for a resource that isn't memory.

### 4.2 Member order matters
In `World`, `conveyors_` is declared **before** `contacts_`, and the comment says why: the contact listener holds a reference to the conveyors, so they must exist first and be destroyed last. C++ builds members in declaration order and destroys them in reverse.

### 4.3 Includes that can't be sorted
Formatting the code sorted `physics.hpp`'s includes alphabetically, which put `Jolt/Core/...` before `Jolt/Jolt.h`, and the build broke with hundreds of errors: `Jolt.h` sets up the configuration every other Jolt header needs. The includes are now inside `// clang-format off` / `// clang-format on`, with a comment saying why. Some orders are part of the code.

### 4.4 A fixed seed
Box spacing can be jittered at random. The random generator (`std::mt19937`) starts from a fixed seed (42), so "random" spacing is the same sequence every run. Random enough to look natural, repeatable enough to test and to compare two runs.

---

## Part 5: Testing

- `sim/world_test.cpp` (12 cases): a box carried at the belt's speed; the heading turning the direction; a stopped belt holding a box still; the VFD ramp; the belt's travelled distance; a box falling off the end and being cleared away; `advance()` carrying the remainder and capping its steps; `run()`'s exact step count; the spawner's timing (and the 2.0 s boundary); boxes queueing without passing through each other; clearing items.
- `render/scene_test.cpp` (4 cases): the belt's top at the physics' height, legs on the floor, frames above the belt, stripes moving exactly as far as the belt, a turned conveyor.
- `render/orbit_camera_test.cpp` (4 cases): the target in the middle of the view, half a turn per window height, the pitch limit, zoom limits, panning in the view's plane.
- By eye: the app running, boxes riding the belt, falling off the end, tipping and landing on each other.

---

## Review questions

**1.** The screen runs at 60 frames per second and the physics at 1000 steps per second. How many steps does a frame run, and what happens to the leftover fraction?
<details><summary>Answer</summary>

16 or 17: a frame of 16.7 ms runs 16 whole steps and keeps 0.7 ms in the backlog, which adds to the next frame's time. Over many frames, exactly one step per simulated millisecond runs.
</details>

**2.** Why is the belt a static body whose *surface* moves, instead of a body that actually moves?
<details><summary>Answer</summary>

A belt is a loop: the surface moves, the conveyor stays where it is. A moving body would drive away. Telling Jolt the surface velocity at each contact makes friction carry the box exactly as a real belt does, slipping and catching up included.
</details>

**3.** Boxes have sleeping turned off. What would go wrong with it on?
<details><summary>Answer</summary>

A box on a stopped belt would fall asleep after a moment. Starting the belt again doesn't touch the box (it only changes the surface speed in the contact), so Jolt wouldn't wake it, and the box would stay put on a moving belt.
</details>

**4.** At 0.1x time scale, does a box move 0.5 mm or 0.05 mm per simulation step on a 0.5 m/s belt?
<details><summary>Answer</summary>

0.5 mm. The time scale only changes how many steps run per real second; each step is still exactly 1 ms of simulated time, so the physics is identical.
</details>
