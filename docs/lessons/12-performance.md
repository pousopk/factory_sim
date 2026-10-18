# Lesson 12: Looks and performance

Milestone 12 set out to make drawing faster (instancing), add shadows and names in 3D, and make Release the normal build. The first measurement changed the plan: the slow part wasn't the drawing. This lesson is mostly about that, measuring before optimising, and then about the graphics.

---

## Part 1: Measure first

### 1.1 The first number
A stress scene, six 8 m belts with a box every 0.22 s on each, about 430-600 boxes, run in Release. The main panel now shows what drawing costs, and the frame said:

| | |
|---|---|
| drawing | 594 boxes, 594 draw calls, CPU 0.76 ms, GPU 1.11 ms |
| simulation | **250 steps this frame, 250 ms per frame** |

Drawing took under 2 ms. The frame took 250 ms because the simulation couldn't keep up with real time: every frame it asked for more steps than it could do, hit the cap of 250, and the window froze for a quarter of a second each time. Making the drawing ten times faster would have saved about 1 ms of those 250.

### 1.2 A benchmark you can run again
`tools/bench.cpp` (`factory_sim_bench`) fills the busy line for 20 simulated seconds and times 2 more:

```
541 items, 1 physics threads: 2.765 ms per 1 ms step (0.36x real time)
```

Each 1 ms step took 2.8 ms of computing, so the simulation ran at a third of real time. A benchmark you can run again turns "it feels slow" into a number that a change either improves or doesn't.

### 1.3 Where the time goes
`valgrind --tool=callgrind` on a short run of the benchmark:

| | |
|---|---|
| `Physics::step` (Jolt) | **99 %** |
| of which solving contacts | 55 % |
| of which finding collisions | 34 % |
| everything of ours (sensors, spawners, cameras, scan) | about 1 % |

There's no point tuning our own loops: they're 1 %. Jolt is the cost, and inside Jolt it's contacts: 540 boxes each resting on a belt, many touching their neighbours.

---

## Part 2: Faster physics

### 2.1 Threads, and staying repeatable
Jolt splits a step into jobs: finding collisions in parts of the world, and solving each **island**, a group of bodies touching each other. Those jobs can run on several threads. factory_sim had used one thread since milestone 1, for a reason: every run must give the same result, or tests and measurements mean nothing.

Jolt's documentation lists the conditions for a deterministic simulation: the same calls in the same order, and the same binary. The number of threads isn't among them. That's a claim worth checking rather than trusting, so a test runs the sorting demo for 10 s on 1 thread and on 8, and compares every box's position and velocity **with `==`, not `Approx`**. They're the same floats, bit for bit.

`Physics` now takes a thread count (0 means automatic: the cores minus one, at most 8) and uses Jolt's `JobSystemThreadPool`. The conveyor contact listener runs on those threads too. It only reads the conveyors' speeds, which nothing changes during a step, so it's safe.

```
1 thread:   2.765 ms per step    0.36x real time
2 threads:  1.959 ms             0.51x
4 threads:  1.270 ms             0.79x
8 threads:  1.045 ms             0.96x
15 threads: 1.018 ms             0.98x
```

The speed-up stops at about 2.7×, near 8 threads. Some of a step can't be split (the broad phase's bookkeeping, the steps between jobs), and that part sets the limit: Amdahl's law. Beyond 8 threads there's nothing left to share out, which is why the automatic choice stops there.

### 2.2 Fewer solver iterations: measured, and not taken
Jolt solves contacts iteratively, 10 velocity iterations per step by default. With 6 the step was 20 % faster, with 4 about 30 %. But the busy line then ended with 535 or 540 boxes instead of 541: the physics behaved differently. A simulator that exists to be accurate shouldn't trade accuracy for speed unless it has to, so this was measured and left out. A change that makes things faster also has to be checked for what else it changes.

### 2.3 When it still can't keep up
A busier line can always outgrow the computer. Before, the app then froze each frame trying to catch up. Now each frame has a **time budget** of about 25 ms for the simulation. From the recent cost of a step (smoothed), the app works out how many steps fit, and does no more. The line then simply runs slower than real time, the window stays responsive, and the main panel says **behind**. The physics is the same, only behind the wall clock. That's what any real-time system needs: when you can't meet the deadline, degrade gracefully instead of falling over.

---

## Part 3: Instanced drawing

### 3.1 One call per box
Every box used to be drawn on its own: set about ten uniforms (its matrix, size, colour, flags), then `glDrawArrays`, so 594 boxes meant 594 draw calls. Each call costs the CPU time in the driver, whatever the box's size.

### 3.2 One call per kind of box
With **instancing**, the boxes' details move into a buffer: one 108-byte record per box (placement matrix, size, colour with alpha, stripe and flag values), uploaded once per frame. Then `glDrawArraysInstanced(..., count)` draws the cube `count` times, and the vertex shader reads the record for its copy. Two things make that work:
- `glVertexAttribDivisor(location, 1)` makes those attributes advance **once per copy** instead of once per vertex.
- A `mat4` attribute takes four attribute locations, one per column.

The boxes go into the buffer in four groups (solid cubes, solid cylinders, see-through cubes, see-through cylinders), and each group is one draw call. Before drawing a group, the attribute pointers are moved to its first record. (`glDrawArraysInstancedBaseInstance` would do that more neatly, but it's OpenGL 4.2, and this renderer is 3.3.)

The buffer is replaced every frame with `glBufferData` ("orphaning"): the driver hands over fresh memory, and the GPU can keep drawing last frame's boxes from the old one without either waiting.

### 3.3 What it changed

| | before | instanced |
|---|---|---|
| draw calls | 594 | 2 (floor, cubes), or 3 with shadows |
| CPU per frame | 0.76 ms | 0.16 ms |
| GPU per frame | 1.11 ms | 1.2 ms |

The CPU time fell by 4-5×. The GPU time didn't move, and that's expected: the GPU draws the same triangles and pixels either way. Instancing removes per-call overhead, which lives on the CPU side.

---

## Part 4: Shadows

### 4.1 Shadow mapping
The sun casts shadows in two passes:
1. **From the sun:** the solid boxes are drawn with an orthographic camera looking along the sunlight, into a 2048 × 2048 **depth texture** (a framebuffer with only a depth attachment). Each texel holds the distance from the sun to the nearest surface it sees.
2. **From the camera, as usual:** each pixel also works out where it is as the sun sees it (`uLightSpace`), and compares its own distance from the sun with the stored one. If something nearer is stored, the pixel is in shadow.

The sun's camera is fitted around the boxes (their extent, down to the floor), so the 2048 texels cover the line and not the whole 120 m floor. For a 10 m line, a texel is about 5 mm.

### 4.2 The usual problems
- **Shadow acne:** a surface compared against its own stored depth shadows itself in stripes, from rounding. Two cures together: `glPolygonOffset` pushes the stored depths slightly away, and the shader subtracts a small **bias**, larger on surfaces the sun grazes.
- **Jagged edges:** one texel is either in or out. The depth texture uses *compare mode* (`GL_COMPARE_REF_TO_TEXTURE`, read with `sampler2DShadow`), so each lookup returns how lit the point is, already filtered between four texels by the hardware. The shader takes nine such lookups around the point (PCF: percentage-closer filtering) and averages them, which gives soft edges.
- **Lights casting shadows:** LEDs and beams are drawn "unlit", and the depth pass discards them.

### 4.3 What it costs
About 1.2 ms of GPU time per frame on this machine: a second pass over the solid boxes, plus nine shadow lookups per pixel. Within a 16.7 ms frame at 60 Hz that's affordable, and **View > Shadows** turns them off.

---

## Part 5: Names in 3D

Every device can show its name at its place in the view (**View > Names in 3D**). `sceneLabels` (in `render/scene`, tested without a window) gives a point just above each device. The app projects that point the way the GPU projects a vertex:
1. multiply by the view-projection matrix;
2. divide by w (points behind the camera have w ≤ 0 and are skipped);
3. turn the -1..1 result into window coordinates.

ImGui then writes the text on its *background* draw list: over the 3D view, under the windows. The selected device's name gets a blue background.

---

## Part 6: Release by default

The build is now Release unless asked otherwise, because a simulator should run in real time. Debug, with AddressSanitizer and UBSan, is still one option away (`-DCMAKE_BUILD_TYPE=Debug`), for chasing a bug. Jolt was already compiled with optimisation in Debug builds (lesson 3). The tests pass in both.

---

## Part 7: Testing

- **`sim/world_test.cpp`**: the sorting demo on 1 thread and on 8 gives bit-identical positions and velocities, and the same bin count.
- **`render/scene_test.cpp`**: every named device has a label, the right one, at its place.
- **Measured:**
  - `factory_sim_bench` with 1-15 threads;
  - the renderer's CPU and GPU time and draw calls, with and without instancing and shadows;
  - callgrind's breakdown of a step.
- **By eye:** shadows (the belt's and the bin's on the floor, soft-edged, no acne), names over the belts, and the "behind" warning on the busy line.

---

## Review questions

**1.** The frame took 250 ms and drawing 2 ms. Why would making the drawing ten times faster not have helped?
<details><summary>Answer</summary>

It would have saved under 2 ms of 250. The time went into the physics: 99 % of each step was in Jolt, and the app was doing 250 steps a frame trying to catch up. You speed up what the measurement says is slow.
</details>

**2.** How do we know the physics gives the same result on 8 threads as on 1?
<details><summary>Answer</summary>

Not only from Jolt's documentation (its conditions for determinism don't include the thread count), but from a test that runs the same scene both ways and compares every position and velocity exactly, with `==`.
</details>

**3.** Instancing cut the CPU time from 0.76 to 0.16 ms but left the GPU time at about 1.2 ms. Why?
<details><summary>Answer</summary>

Instancing removes the per-draw-call overhead, which is CPU and driver work. The GPU still draws the same triangles and fills the same pixels.
</details>

**4.** What is "shadow acne", and what are the two cures used here?
<details><summary>Answer</summary>

A lit surface shadowing itself in stripes, because its own depth, rounded into the shadow map, compares as "behind" itself. The cures: `glPolygonOffset` in the depth pass pushes the stored depths slightly away, and the shader subtracts a bias that's larger on surfaces the sun grazes.
</details>
