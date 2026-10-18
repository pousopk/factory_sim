# factory_sim

A 3D factory simulator with a Dear ImGui interface: conveyors carrying objects, sensors that behave like real ones, and actuators, so you can see and measure how a line behaves before building it.

## Decisions

- **Graphics:** 3D rendering with a Dear ImGui interface.
- **Physics:** a hybrid. Our own code for conveyors, sensors and timing; Jolt Physics for object collisions.
- **Building the line:** snap-together building (conveyors attach end to end, sensors and cameras sit along a conveyor), with an ImGui inspector panel for fine-tuning. The scene saves to a file automatically, never edited by hand. 3D drag handles (ImGuizmo) may come later.
- **Control:** built-in logic first, with an I/O interface designed from the start so `soft_plc` or a real PLC can take over.
- **PLC link:** lockstep first: `soft_plc` runs inside the simulator on the same virtual clock (one PLC cycle every 10 simulation steps), so runs are reproducible and can go faster than real time, and a program written in `ladder_editor` controls the line. A live S7 link over Snap7 (to `soft_plc --s7` or a real S7) comes after.

## Goals

- **Conveyors** with real speed (m/s), VFD-style acceleration ramps, roller diameter and an encoder producing real pulses.
- **Objects** with size, mass and friction, spawned at configurable rates and gaps; they queue, collide, get pushed and fall off.
- **Sensors with realistic behaviour:** photocells (response time, hysteresis, minimum object size), proximity sensors, laser distance (range, resolution, noise), light curtains, encoders, and a camera station (field of view, exposure, motion blur, object position at trigger time).
- **Actuators:** pneumatic pushers with stroke time, diverters, stoppers, reject gates.
- **Fixed physics timestep** (e.g. 1 ms), separate from rendering, so timing effects are measurable: at 2 m/s an object moves 2 mm per ms.
- **Measurements:** trigger position errors, timing plots and event logs.
- **PLC link:** a real S7 PLC (via Snap7), or later `soft_plc`, controls the line.

## Reference scenario: tins and a shared camera trigger

Based on a real project. Tins pass on a conveyor, and one photocell triggers four cameras at once. When two tins are too close together, a camera is still busy with the first frame (exposure + readout) when the second trigger arrives, so it ignores that trigger. Three cameras took the photo; the fourth missed it.

The cameras were GigE, triggered by the PLC. The fourth camera was slower because of where the sensor sat: the PLC delayed its trigger with a timer so the tin would be in its field of view. That delay shifted its busy window into the next tin's trigger.

The simulator should reproduce this and answer: at a given conveyor speed and tin spacing, which camera misses which tin, and what is the minimum gap that works?

## Milestones

1. One conveyor in a 3D scene with boxes moving at real speed, plus an orbit camera
2. Photocell with response time and encoder pulses; exact switching times and the trigger position error shown in 3D
3. Snap-together line building, the inspector panel and undo; scene saved automatically
4. Transfers between conveyors, pushers, diverters, stoppers and a reject bin
5. I/O map and built-in control logic (a "sort NOK boxes" demo)
6. PLC link: `soft_plc` in lockstep (programs from `ladder_editor` control the line), then live S7 over Snap7
7. Logic analyser: signal timelines, trigger error histograms, event log
8. Camera station and the tin scenario: field of view, exposure and readout, which camera misses which tin

## Building

```bash
cmake -S . -B build          # Debug by default, with AddressSanitizer + UBSan
cmake --build build -j
ctest --test-dir build --output-on-failure
./build/factory_sim
```

The first configure downloads its dependencies (Jolt Physics, glm, Dear ImGui, ImPlot, {fmt}, doctest). The application needs GLFW and OpenGL 3.3 development files (`libglfw3-dev`, `libgl-dev`); without them, configure with `-DFACTORY_SIM_BUILD_APP=OFF` to build the simulation and its tests only. For smooth frame rates, build Release (`-DCMAKE_BUILD_TYPE=Release`): the sanitizers make the Debug build several times slower.

## Using it

```bash
./build/factory_sim                 # the scene in factory_sim_scene.json (a new one if it doesn't exist)
./build/factory_sim my_line.json    # another scene file
```

The scene is saved after every change; there's no Save button and the file isn't meant to be edited by hand.

**The view:** left drag turns it around the point it looks at, right (or middle) drag moves that point, the wheel zooms. A click selects a conveyor, photocell or spawner; Del deletes it, Esc deselects.

**Building the line** (the **Line** panel): a new conveyor can follow the selected one, straight on or turned left or right; it then starts where the other ends and moves with it. **New conveyor** adds one on its own, beside the others. **Photocell** and **Spawner** go on the selected conveyor. Everything in the line is listed there too, to select.

**The Inspector** edits the selection: a conveyor's place (or, if it follows another, its turn, and **Detach**), size, speed, ramp and encoder; a photocell's conveyor, position, height, beam, thresholds and response time; a spawner's conveyor, rate, jitter, box size and position. Speeds and sensor settings apply while boxes keep running; changing a conveyor's place or size restarts the run. Ctrl+Z / Ctrl+Y undo and redo; dragging a slider is one step.

**The main panel** runs simulated time: **Pause**, then **Step 1 ms** to watch a box move exactly speed x 1 ms per step, or a time scale of 0.1x to watch it in slow motion with exactly the same physics. **Restart** starts the run again.

**The Triggers window** lists each time the selected photocell switched on: when the beam was covered and when the output switched (exact to a fraction of a millisecond), how far past the beam the box's front edge was then, the same as a PLC scanning every few milliseconds would see it, and the encoder count. A see-through box shows where the last triggered box was at its output time.

## Layout

| Path | Contents |
|---|---|
| `src/sim/` | The simulation: Jolt set up for repeatable results (`physics.hpp`), conveyors, items and time (`world.hpp`), photocells and encoders (`sensors.hpp`), signals with exact switching times (`signal.hpp`); the scene, its snapping, editing and file (`scene.hpp`) |
| `src/edit/` | Undo and redo of the scene |
| `src/render/` | What to draw and what a click hits (`scene.hpp`) and from where (`orbit_camera.hpp`), all plain geometry; the OpenGL renderer and shaders |
| `src/app/` | The application: window, frame loop, panels |
| `tests/` | doctest unit tests, mirroring `src/`; they run the simulation headless |
| `docs/lessons/` | What each milestone teaches, with review questions |
| `docs/topics/` | One subject at a time across the whole project: the GPU and OpenGL, 3D maths, shaders, numbers and time, the physics engine, design notes |
