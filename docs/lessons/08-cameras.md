# Lesson 8: Cameras and the tin line

Milestone 8 builds the reference scenario from the README. Tins pass on a conveyor and one photocell triggers four cameras. When two tins come too close, the fourth camera misses one. The simulator now answers the scenario's two questions:
- which camera misses which tin;
- what is the smallest gap that works.

```
 spawner          B1 (photocell)     K1 K2 K3            K4
   │  tins at 1 m/s   │              (at B1)          (15 cm on)
 ──○──○────○──○───────┼─────────────────[▒▒]────────────[▒▒]──►
                      │                  ▲                ▲
                      └── PLC: K1-K3 at once ─┘           │
                          K4 after a 150 ms timer ────────┘
```

Try it: in `./build/factory_sim`, click **Load the tin demo**. Watch the **Cameras** window: K4's column shows `MISSED: no trigger` now and then, while K1-K3 never miss. Then open its **Gap study** tab and press **Study**.

---

## Part 1: A triggered camera

### 1.1 Busy, and deaf meanwhile
An industrial camera triggered by a PLC goes through a fixed sequence for each picture (`src/sim/camera.hpp`):

```
trigger ─┐
         ├ trigger delay ┤ exposure ├──── readout ────┤
         │◄─────────────────── busy ─────────────────►│
```

- **Trigger delay:** the camera's own setting (GenICam calls it `TriggerDelay`). It waits this long before exposing.
- **Exposure:** the sensor collects light. Anything that moves smears by speed × exposure. At 1 m/s and 0.5 ms that's 0.5 mm. With 1280 pixels over 200 mm, that's 3.2 pixels, which the inspector shows in yellow. A real line would use a shorter exposure or a strobe.
- **Readout:** the picture leaves the sensor and goes over the network. A 5-megapixel picture over Gigabit Ethernet takes about 40 ms.

A trigger that arrives while the camera is busy is **ignored**. There's no picture and no error, unless something checks the camera's "ready" line. The model has that line too: the camera's **ready input**, off while it's busy. `CameraTiming` holds only this timing (when the camera accepts a trigger, when it's free again). The `Camera` in the World adds the picture: which item was in view at the middle of the exposure, how far from the middle of the picture, and the blur.

### 1.2 Who it would have seen
For an ignored trigger, the World still works out which tin the camera *would* have seen. That's how the Cameras window can say "K4 missed tin 37" instead of only "K4 missed one".

### 1.3 Tins
Tins are Jolt **cylinders** standing on their axis (`Shape::Tin`). For the photocell, what matters is how far an item reaches along the belt from its centre:
- for a box, its half-sizes projected onto that direction;
- for an upright tin, its radius in any horizontal direction.

`reach` does both, so the photocell, the trigger error and the cameras all work for tins unchanged.

Tins also **arrive at belt speed** (`atBeltSpeed`), as from an infeed conveyor. Put down standing still, each tin needs about 0.2 s of friction to catch up with a 1 m/s belt. The spawner waits for room meanwhile, so it would have stretched every short gap the scenario is about.

---

## Part 2: Three places for a delay

K4 sits 15 cm past the others, so it needs its trigger 150 ms later at 1 m/s. That delay can live in three places, and each fails differently. Below, T is the time from one tin to the next.

### 2.1 One timer in the PLC (the real project, and the demo)
The photocell's edge starts a timer, and the timer's end fires the trigger. That's the quick way to write it, and `examples/tin_cameras.st` shows it in ST:

```
IF newTin.Q THEN
    IF k4Waiting THEN k4Lost := k4Lost + 1;  (* the timer is busy: this tin is lost *)
    ELSE k4Waiting := TRUE; END_IF;
END_IF;
k4Delay(IN := k4Waiting, PT := T#150ms);
k4 := k4Delay.Q;
```

**One timer can only time one tin.** A second tin that arrives while it runs gets no trigger at all. The camera never even sees a trigger to ignore, which is why the table says "no trigger" rather than "busy". K4 misses a tin whenever T is shorter than the timer's 150 ms, rounded up to the scan (see 2.4). In the demo, T varies between 120 and 280 ms, and K4 lost 14 of 64 tins in one run.

### 2.2 The delay in the camera
The PLC triggers K4 at once, and the camera waits 150 ms itself. Now the camera is busy for delay + exposure + readout = 190.5 ms per picture. The README's story, "the delay shifted its busy window into the next tin's trigger", is exactly this: the camera's busy time is the delay plus the picture. It needs tins at least 190.5 ms apart (up to a scan more), about 125 mm of gap. That's *worse* than the PLC timer, because the readout is added on top.

### 2.3 A queue: one trigger per tin
`DelayMethod::PerItem` remembers a trigger time for each tin ("now + 150 ms") and fires them in order. It's the sorter's encoder queue from lesson 5, in time instead of pulses. Every tin gets its own trigger however close it follows, and K4 only has to keep up with its own readout, like the others. With this method, the gap study gives K4 the same small gap as K1-K3.

The general rule: **when items overlap in time, the state has to be per item.** One timer, one flag or one variable holds one item. That's the bug behind many "it misses one now and then" reports on real lines. Another fix that needs no new logic is a second photocell at K4.

### 2.4 Timers end on a scan
The program looks at its timer once per scan, so a 150 ms timer started at a scan ends exactly 15 scans later. A 151 ms timer ends 16 scans later, at 160 ms, and the tin is 9 mm past the middle of the picture at 1 m/s. That's why `triggerFor`, which works out a delay from the layout, **rounds it to whole scans**. A delay under half a scan becomes none, since 2 ms would really be 10.

There's one more scan effect. In the scan where the timer finishes, a new edge is still lost, because the timer was running when the program looked at the edge. So K4 needs T *more* than 150 ms, and the edges themselves are seen up to a scan late. The study's answer for K4 is 160 ms: 160 mm pitch at 1 m/s, minus the 73 mm tin, gives 87.5 mm of gap.

### 2.5 Where the photocell stops working
K1-K3 never miss down to a 10 mm gap. Below that, the gap between two tins passes the beam in less than a scan, so the PLC never sees the photocell go off, and two tins become one long pulse with one trigger. It's the same scan-sampling limit as lesson 5, now deciding the line's maximum rate: about 720 tins a minute at 1 m/s.

### 2.6 Where the tin is in the picture
K1-K3 see each tin between −4 and +5 mm from the middle of the picture. That's the spread from lesson 7: the photocell's edge falls anywhere in the 10 ms scan, which is 0 to 10 mm at 1 m/s. K4 has the same spread, because its timer starts at the same scan.

---

## Part 3: The gap study

### 3.1 The right level of detail
The full simulation answers "which camera missed which tin" for the tins that happen to come by. "What gap works?" needs every gap, so `studyGap` (`src/analysis/tin_study.hpp`) runs the line's **timing alone**. It places tins exactly `gap` apart, moving at the belt's speed, and feeds them through:
- the scene's own photocells, through the very same code as the World: `Photocell::sense` (hysteresis, crossing time, response time), moved out of the World for this;
- the scene's control program, scan by scan, through an I/O image;
- the cameras' `CameraTiming`.

There's no physics, because tins riding a belt move at its speed and physics would add nothing to the timing. The result is fast: a sweep of 41 gaps takes 28 ms in a Release build. The Debug build's sanitizers make it about 50 times slower.

The gap doesn't decide where the edges fall in the scan. So each gap runs 5 times, with the tins shifted by a fifth of a scan each time, and counts as working only if every run works. `minimumGap` then walks down from the largest gap while it still works, so its answer is a gap from which **every** larger one works, not just a lucky one.

### 3.2 The same code, not a copy
The study is only worth something if it does what the line does. Three choices make sure it does:
- The photocell's switching moved into `Photocell::sense` / `nextOutput`, which both the World and the study call.
- The program is made by a `ProgramMaker`, the same one `buildWorld` takes. The study runs the built-in triggering, or **soft_plc running `tin_cameras.st`**, without knowing which.
- A test runs the full simulation with evenly spaced tins at two gaps, and checks that the study agrees on which cameras miss.

### 3.3 On another thread
In the Debug build, a sweep of 101 gaps takes a few seconds. So the app runs it with `std::async(std::launch::async, ...)` on a copy of the scene, and checks each frame whether the `std::future` is ready (`wait_for(0s)`). The window keeps drawing and says "Studying..." meanwhile. Two details make this safe:
- The study gets its **own copy** of the scene and makes its own programs. It shares nothing with the running World.
- soft_plc programs are made **without their monitor port**, because the running World's soft_plc holds it.

A future from `std::async` waits for its thread when it's destroyed, so closing the app mid-study waits for the study to finish rather than leaving a thread behind.

---

## Part 4: soft_plc, and a nanosecond

`examples/tin_cameras.st` is the triggering in ST: an `R_TRIG` for the tin's edge, a one-scan pulse for K1-K3, and one `TON` for K4. A test runs the tin demo once with the built-in program and once with soft_plc running this file. Every camera's shots must match, trigger time for trigger time, and soft_plc's `k4Lost` must equal the built-in count.

The first run differed: one trigger came 10 ms later under soft_plc. factory_sim gives soft_plc the simulated time as a `std::chrono` duration, and converted it with `duration_cast`, which **truncates**. 8.19 s as a double is 8.1899999999999995, which truncated to whole nanoseconds is 1 ns short. A 150 ms timer started at 8.04 s then saw 149.999999 ms at 8.19 s and ended a scan late. `std::chrono::round` fixes it. The lesson: converting between floating-point time and integer ticks needs rounding, and two implementations of the same logic will expose a mismatch, because a 1 ns difference turns into a whole 10 ms scan.

---

## Part 5: C++ and design

### 5.1 Timing apart from imaging
`CameraTiming` is ten lines with no dependencies, so the study can use it without the World, and its test is four lines. The `Camera` in the World wraps it with what needs the World: the signals, the pictures and the items in them. It's the same split as lesson 7's analysis and drawing, one level down.

### 5.2 Jolt shapes and `Ref`
A tin is a `JPH::CylinderShape`, around its own y axis like the rendered cylinder. `makeItemShape` returns a `JPH::Ref<JPH::Shape>`, Jolt's reference-counted pointer. Returning a `Ref<BoxShape>` from it didn't compile: `Ref<Derived>` doesn't convert to `Ref<Base>` implicitly the way `std::shared_ptr` does, so the conversion is written out.

### 5.3 A second mesh
The renderer had one mesh, a unit cube scaled per box. Tins add a unit cylinder of 32 flat sides. Its normals point straight out from the axis, so the lighting makes the flat sides look round. A `round` flag on `DrawBox` picks the mesh. The shader's darker edges needed a round version: on a cylinder, the edges are the rims, so a lid measures the distance to the round edge and the side measures the distance to the top or bottom.

### 5.4 Shared widgets
The camera panels went into their own file, `cameras.cpp`, and needed the sliders, address fields and colours that lived privately in `app.cpp`. They moved to `app/widgets.hpp`. The colours are `inline const` variables (C++17), which can live in a header without one definition per file that includes it.

### 5.5 A bug found by reading
`buildWorld` added every bin twice: two identical loops, one after the other. Nothing failed. The second bins sat exactly on the first, the panels only looked at the first ones, and the sorting test counted those. It turned up while reading the function to add cameras to it, and now has a test (`world->bins().size() == 1`). Duplicated code that "works" is still worth reading.

---

## Part 6: Testing

- **`sim/camera_test.cpp`** (9 cases):
  - a busy camera ignores a trigger and takes the next one once it's free;
  - one-timer triggering loses an item that comes while the timer runs;
  - per-item triggering gives every item its trigger, one scan long, with the output off in between;
  - tins stand upright and trigger the photocell;
  - on the tin line, K1-K3 take every tin, whole and within 15 mm of the middle, while K4 loses some;
  - a delay in the camera makes it ignore triggers;
  - cameras and triggers round-trip through the scene file;
  - removing a conveyor removes its cameras, and a new camera gets free addresses;
  - each bin is built once.
- **`analysis/tin_study_test.cpp`** (3 cases):
  - K4's smallest working gap with one timer is about 85 mm, and K1-K3's is under 30 mm;
  - per-item triggering brings K4 down to the others' gap, and a camera delay raises it to about 125 mm;
  - the study agrees with the full simulation.
- **`plc/plc_test.cpp`** (1 new case): soft_plc running `tin_cameras.st` triggers the cameras exactly as the built-in program does, loses the same tins, and the gap study gives the same answer with either.
- **By eye:** the tin line running, the field-of-view flashes, the Cameras window's table, and the gap study's plot.

---

## Review questions

**1.** A camera's readout is 40 ms and its exposure 0.5 ms. Tins come 30 ms apart, and every tin gets its own trigger. What happens?
<details><summary>Answer</summary>

The camera takes every other tin. After a picture it's busy for 40.5 ms, so the trigger 30 ms later is ignored, and the one 60 ms later is taken. The fix has to give the camera more time: wider gaps, or a camera with a faster readout.
</details>

**2.** With one timer in the PLC for K4, why does the Cameras window say "no trigger" for K4's misses rather than "busy"?
<details><summary>Answer</summary>

The second tin's edge comes while the timer is still running for the first. The program has nowhere to keep a second start time, so that tin's trigger is never sent. K4 is free and would have taken it, but it never got a trigger.
</details>

**3.** Moving K4's 150 ms delay from the PLC timer into the camera's own trigger delay made the smallest gap larger, from about 88 to about 125 mm. Why?
<details><summary>Answer</summary>

In the camera, the delay is part of the busy time: delay + exposure + readout = 190.5 ms per picture. With the PLC timer, only the timer's 150 ms (plus the scan) mattered, and the readout overlapped with the next tin's timing.
</details>

**4.** A PLC with a 10 ms scan should delay a trigger by 151 ms. Why does `triggerFor` set 150?
<details><summary>Answer</summary>

The program checks its timer once per scan, so the timer can only end on a scan. 151 ms would end at the 16th scan, at 160 ms, 9 ms late. 150 ms ends at exactly the 15th, the closest it can get.
</details>
