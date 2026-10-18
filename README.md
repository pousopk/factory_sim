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
9. Files: Open, Save As, recent scenes, the untitled scene, and a remembered window layout
10. Analog I/O and drives: words in the I/O image, a drive on the bus (PROFIdrive telegram 1), potentiometers
11. Sensors: inductive and capacitive proximity switches, a laser distance sensor, a light curtain
12. Looks and performance: the physics on several threads, instanced drawing, shadows, names in 3D
13. Line equipment: accumulation conveyors with zones, turntables, lifts and merges
14. Editing in 3D: dragging things in the view, placing by clicking on a belt, copy and paste
15. Camera images: what area and line-scan cameras see, with motion blur and noise
16. A better window: docked panels around the 3D view, a toolbar and status bar, a proper font and theme
17. Finding your way: the Line panel as a tree, hover tooltips in the 3D view, items you can select

## Building

It builds `soft_plc` from source, from a folder next to it, so clone both side by side:

```bash
git clone https://github.com/pousopk/soft_plc.git
git clone https://github.com/pousopk/factory_sim.git
cd factory_sim
```

```bash
cmake -S . -B build          # Release by default
cmake --build build -j
ctest --test-dir build --output-on-failure
./build/factory_sim
```

For chasing bugs, a Debug build adds AddressSanitizer and UBSan (several times slower):

```bash
cmake -S . -B build-debug -DCMAKE_BUILD_TYPE=Debug
```

`./build/factory_sim_bench` measures how long a simulation step takes on a busy line (about 540 boxes), or on a scene file you give it, with `threads=N` to compare thread counts.

The first configure downloads its dependencies (Jolt Physics, glm, Dear ImGui, ImPlot, {fmt}, nlohmann/json, tl::expected, doctest), and uses `soft_plc` from `../soft_plc` next to it (or `-DFACTORY_SIM_SOFT_PLC_DIR=...`), which brings Snap7. The application needs GLFW and OpenGL 3.3 development files (`libglfw3-dev`, `libgl-dev`); without them, configure with `-DFACTORY_SIM_BUILD_APP=OFF` to build the simulation and its tests only.

## Using it

```bash
./build/factory_sim                 # the scene opened last (or the untitled one)
./build/factory_sim my_line.json    # that scene (a new one there if the file doesn't exist)
```

The scene is saved after every change; there's no Save button and the file isn't meant to be edited by hand. The **File** menu has **New**, **New from a demo**, **Open...** (Ctrl+O), **Open recent** and **Save as...** (Ctrl+Shift+S). A new scene, or a demo, is *untitled*: it's still saved after every change, to a scratch file, until Save as gives it a name and a folder; replacing an edited untitled scene asks first. Save as keeps a soft_plc program path that's relative to the scene pointing at the same file from the new folder. **View** shows and hides each window. factory_sim keeps its own files in `~/.config/factory_sim` (or `$XDG_CONFIG_HOME/factory_sim`): the recent scenes, the window layout, and the untitled scene.

**The window.** The 3D view sits in the middle, with the panels docked around it: building on the left (Control, Line, Inspector), watching on the right (I/O, Triggers, PLC variables, Cameras), time underneath (Logic analyser, Camera view, Operator panel). Drag a panel's tab to move it, dock it elsewhere or float it; the arrangement is remembered, and **View > Reset layout** puts it back. The toolbar runs time; the status bar says whether the simulation keeps up with real time, what drawing costs, and the file. **Help > Mouse and keys** (F1) lists them all. The font is Roboto (from Dear ImGui's fonts, Apache 2.0 licence), built into the program.

**The view:** left drag turns it around the point it looks at (dragging the selected thing moves it instead, see *Editing in the view*), right (or middle) drag moves that point, the wheel zooms. A click selects what's under the mouse; Del deletes it, Esc deselects.

**Building the line** (the **Line** panel): a new conveyor can follow the selected one, straight on or turned left or right; it then starts where the other ends and moves with it (a turned one sits 2 cm lower and gets a guide on its far side, so boxes drop onto it and turn). **New conveyor** adds one on its own, beside the others. **Photocell**, **Spawner**, **Pusher**, **Diverter**, **Stopper**, **Bin** and the other devices go where you then click on a belt. Everything in the line is listed there too, to select.

**Actuators** move like air cylinders: a dead time after the valve switches, then a steady stroke, with end position sensors (the LEDs). A **pusher** pushes boxes off the other side of the belt, into a **bin** that counts them; a **diverter** arm swings out and guides boxes off the other side; a **stopper** blade rises out of the belt and holds boxes back while the belt slides under them. Until milestone 5 adds control logic, they're operated by hand in the inspector: **Extend**, **Retract**, or **Fire once** (out, and back as soon as it's fully out).

**The Inspector** edits the selection: a conveyor's place (or, if it follows another, its turn, and **Detach**), size, speed, ramp and encoder; a photocell's conveyor, position, height, beam, thresholds and response time; a spawner's conveyor, rate, jitter, box size and position. Speeds and sensor settings apply while boxes keep running; changing a conveyor's place or size restarts the run. Ctrl+Z / Ctrl+Y undo and redo; dragging a slider is one step.

**Control and I/O.** Every device is wired to addresses written as in soft_plc: a photocell to an input bit (`%IX0.0`), an actuator's valve to an output bit and its end sensors to input bits, a conveyor's encoder count to an input double word (`%ID8`) and, optionally, its motor to an output bit. New devices get the next free address; the inspector changes them. The **I/O** window lists every address with its live value. The **Operator panel** has buttons (momentary or latching, normally open or normally closed) and lamps, each on an address. The **Control** panel's **program** picks what drives the outputs every 10 ms: nothing (actuators by hand, conveyors always running) or the **built-in sorting**: when an inspection sensor sees a bad box, a sorter notes the encoder count, fires the pusher once the belt has moved the set distance, and pulls it back when it's extended. **Add sorter** wires one to a sensor and a pusher and works out the distance from the layout. **File > New from a demo > Sorting** builds a whole sorting line. Spawners can make a share of their boxes bad (red), and a photocell can be set to see only bad boxes, as an inspection system would.

**More sensors** (Line panel: Proximity, Laser, Light curtain). A **proximity switch** sits beside the belt or above it and switches when an item is within its switching distance (sn) in front of its face: an **inductive** one sees only metal (tins, not cardboard), a **capacitive** one sees anything, cardboard only much closer; its reach is drawn as a faint disc. A **laser distance sensor** above the belt measures how far the surface under its spot is (the belt or an item's top), in steps of its resolution, with noise, every few milliseconds, as an analog word (0..27648 over its range, 32767 out of range). A **light curtain** has beams across the belt one above the other: a *clear* bit (on while no beam is broken) and a word with the number of broken beams, once per scan cycle. Spawners can vary their items' heights. **File > New from a demo > Sensors** runs boxes of different heights and tins past all of them; the logic analyser plots the laser's measurements.

**Drives and analog values.** The I/O image has 64 bytes each way: bits in bytes 0-7, words and double words from byte 8, shown in the I/O window as a TIA watch table would (`16#047F`). A conveyor can have a **drive on the bus** (inspector, *Drive on the bus*), driven the way an S7 drives a SINAMICS: PROFIdrive standard telegram 1, the control word STW1 and speed setpoint NSOLL_A from the PLC (16#4000 = 100 % of the reference speed), the status word ZSW1 and actual speed NIST_A back. Inside it runs the PROFIdrive state machine (switching on inhibited, ready, switched on, operation enabled, fault), a ramp, and the three stops: OFF1 down the ramp, OFF2 coasting, OFF3 the quick-stop ramp. **Trip it** raises a fault that the PLC acknowledges with STW1 bit 7. The inspector shows both words bit by bit. Without a control program, the drive runs at the target speed, as a belt without one always does. The operator panel can have **potentiometers**: analog input words from 0 to 27648, like a Siemens analog input module. **File > New from a demo > Conveyor with a drive** runs `examples/drive.st` on soft_plc: press Start and turn the Speed potentiometer. The logic analyser's speed plot shows each drive's setpoint under the belt's speed. A live S7 link exchanges the first **bytes** of each area (16 by default, as `soft_plc --s7` has).

**A PLC in control.** With **soft_plc (lockstep)**, soft_plc runs an ST file (ladder_editor's **Export ST** writes one) as the control program: every 10 ms scan is one soft_plc cycle on the *simulated* clock, so timers keep up at any time scale and every run gives the same result. With a **monitor port**, ladder_editor goes online with it (`ladder_editor --connect=5020 project`): power flow on the rungs while the simulated line runs, values, forcing, and **Download**, which changes the running program without stopping it. With **S7 PLC (live)**, a PLC running on its own (`soft_plc --s7=1102 program.st`, or a real S7) controls the line over S7: the simulator writes its inputs and reads its outputs every scan, in real time. **File > New from a demo > Conveyor** (or `./build/factory_sim examples/conveyor_line.json`) runs ladder_editor's conveyor example on a line wired to its addresses: press **Start** on the operator panel.

**The logic analyser** (View > Logic analyser) has three tabs. **Timelines**: every digital signal of the line (motors, photocells, valves, end sensors, buttons, lamps) stacked on one time axis, with the belts' speeds under them; **follow** scrolls with the simulation, and with it off the wheel zooms and dragging pans through the history; the markers **A** and **B** measure the time between any two moments; **Signals...** picks which traces to show. **Trigger errors**: histograms of a photocell's trigger position errors, as the sensor gives them and as a PLC scanning every few milliseconds (a slider) would see them, with mean, deviation and range. **Event log**: every switch with its exact time and address, newest first, filtered by name; **Export CSV** writes it to `events.csv` next to the scene file.

**Cameras and the tin line.** A **camera** (added over the selected conveyor) looks at a stretch of belt, its field of view, drawn on the belt. A rising edge on its trigger output takes a picture: after its own trigger delay it exposes (things moving smear by speed x exposure: the inspector shows it in pixels), then reads the picture out; until then it's **busy** (its LED, and its ready input off) and **ignores** any trigger. Its field of view flashes white for a picture and red for an ignored trigger. Spawners can make **tins** (upright cylinders) and let items arrive **at belt speed**, as from an infeed. The program **built-in camera triggering** triggers each camera a set time after a photocell sees an item, delayed either by **one timer** (an item arriving while it runs gets no trigger) or by **a queue, per item**; **Add trigger** works out the delay from the layout. **File > New from a demo > Tins and cameras** builds the reference scenario: tins at 1 m/s, one photocell, three cameras at the photocell and a fourth 15 cm on, triggered 150 ms later by one timer. The **Cameras** window shows, tin by tin, which camera took it (and how far from the middle of the picture), and which missed it because it was busy or never got a trigger. Its **Gap study** runs the line's timing alone (the real photocell logic and control program, scan by scan, the cameras' busy times, tins placed exactly) for every gap from 0, a few times across the scan, and gives the smallest gap from which every camera takes every tin: 87.5 mm in the demo (K1-K3 alone: 10 mm). `examples/tin_cameras.st` is the same triggering in ST for soft_plc, with identical results.

**Line equipment** (conveyor inspector, and the Line panel's *Join it*). A conveyor can be split into **accumulation zones**, each with its own motor and photo-eye, as motor-driven-roller conveyors are: run by the zone cards themselves (zero-pressure accumulation: one box per zone, a full zone passes its box on only into an empty one, the last zone while the PLC's *release* output is on), or by the PLC, one motor output per zone. A conveyor can sit on a **carriage**: a **turntable** turns it about its middle (90 degrees, or any angle), a **lift** raises it to another level; the carriage moves like an air cylinder, on one output, with a sensor at each end. A conveyor can **join** another from the side, partway along it, a little higher: a **merge**. **File > New from a demo > Line equipment** runs `examples/equipment.st` (an accumulation conveyor, a turntable and a lift, each sequenced by a `CASE` state machine); **Merge** runs `examples/merge.st` (the side line waits for gaps in the main line).

**What the cameras see** (View > Camera view). An **area camera** is drawn from its own position and lens (its working distance and field of view set the lens angle; the picture's x runs along the travel): **Live**, as if it took a picture every frame, and **Last picture**, the one it took when triggered, with the items put back where they were at its exposure. A **line-scan camera** (kind *line scan*) has one row of pixels across the belt and takes a line at a fixed rate or every so many encoder pulses; its lines stack up into an image as products pass. Neither models brightness; both model what goes wrong: **motion blur** (the scene averaged over the exposure while things move) and **noise** (grey levels). The inspector shows a line camera's pixel size across and along, and how much longer or shorter products will look when they don't match: free-running, it depends on the belt's speed; from the encoder, it doesn't. **File > New from a demo > Line-scan and area cameras** has one of each.

**Finding your way.** The **Line panel** is a tree: each conveyor (a dot when its belt runs), with what follows or joins it, its zones and carriage, and under it every device on it with its state, kind, place and address. The filter box keeps the rows whose name matches (a conveyor's name keeps all of it). Clicking a row selects it, as clicking in the view does. In the **3D view**, whatever is under the mouse is lit a little, and a **tooltip** says what it is, where, and its I/O as the PLC sees it now (`%IX0.0 = 1`, `%IW20 = 16#3A2C`). **Items** can be selected too: the inspector shows their size, place and speed, and **Del** (or *Remove it*) takes one off the line, for a jam you made by hand.

**Editing in the view.** Drag the *selected* thing with the left button: a device slides along its conveyor (1 cm steps), a conveyor standing on its own moves across the floor (5 cm steps), taking those that follow or join it along; hold **Shift** for 1 mm steps. While dragging, the new layout is shown and the run waits; letting go is one undo step. **R** (Shift+R) turns the selected conveyor by 90 degrees. The Line panel's device buttons (Photocell, Pusher, Laser, ...) wait for a click on a belt and put the device there; Esc cancels. **Ctrl+C** / **Ctrl+V** copy and paste the selection (onto the belt under the mouse, if there is one), **Ctrl+D** duplicates it: a copy gets the next free name and its own free addresses, so it doesn't fight the original for its outputs.

**Performance.** The physics runs on several threads (up to 8), with exactly the same results as on one. When a line is too busy to simulate in real time, the status bar says *behind*, and the simulation runs slower than real time instead of freezing the window. The status bar also shows what drawing costs: boxes, draw calls, CPU and GPU time. **View** turns the **shadows** and the **names in 3D** on and off.

**The toolbar** runs simulated time (Space runs and pauses): **Pause**, then **Step 1 ms** to watch a box move exactly speed x 1 ms per step, or a time scale of 0.1x to watch it in slow motion with exactly the same physics. **Restart** starts the run again.

**The Triggers window** lists each time the selected photocell switched on: when the beam was covered and when the output switched (exact to a fraction of a millisecond), how far past the beam the box's front edge was then, the same as a PLC scanning every few milliseconds would see it, and the encoder count. A see-through box shows where the last triggered box was at its output time.

## Layout

| Path | Contents |
|---|---|
| `src/sim/` | The simulation: Jolt set up for repeatable results (`physics.hpp`), conveyors, items and time (`world.hpp`), photocells and encoders (`sensors.hpp`), proximity switches, laser distance sensors and light curtains (`field_sensors.hpp`), cameras (`camera.hpp`), drives (`drive.hpp`), signals with exact switching times (`signal.hpp`), pneumatic actuators and bins (`actuators.hpp`), the I/O image and addresses (`io.hpp`), control programs, the built-in sorter and camera triggering (`control.hpp`); the scene, its snapping, editing and file (`scene.hpp`) |
| `src/analysis/` | Measurements, without drawing: the logic analyser's traces, step shapes, event log and CSV, statistics of trigger errors (`analysis.hpp`); the gap study of the tin line (`tin_study.hpp`) |
| `src/edit/` | Undo and redo of the scene, the settings (recent scenes), the folder browser behind Open and Save as, editing in the view as geometry (`placing.hpp`: the belt under the mouse, dragging, placing, copy and paste) |
| `src/plc/` | The PLC link: soft_plc in lockstep (`soft_plc_program.hpp`), a live PLC over S7 (`s7_link.hpp`) |
| `examples/` | The conveyor line and ladder_editor's conveyor program exported to ST; the tin line's camera triggering in ST; a drive driven over PROFIdrive telegram 1 (`drive.st`); the line equipment's and the merge's sequences (`equipment.st`, `merge.st`) |
| `src/render/` | What to draw and what a click hits (`scene.hpp`) and from where (`orbit_camera.hpp`), all plain geometry; the OpenGL renderer and shaders |
| `src/app/` | The application: window, frame loop, panels (`app.cpp`), the logic analyser (`analyser.cpp`), the cameras' panels (`cameras.cpp`), shared widgets |
| `tools/` | `factory_sim_bench`, the simulation step benchmark |
| `tests/` | doctest unit tests, mirroring `src/`; they run the simulation headless |
| `docs/lessons/` | What each milestone teaches, with review questions |
| `docs/topics/` | One subject at a time across the whole project: the GPU and OpenGL, 3D maths, shaders, numbers and time, the physics engine, design notes |

## License

MIT: see [LICENSE](LICENSE).
