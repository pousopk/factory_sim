# Lesson 13: Line equipment

Milestone 13 adds the equipment that turns a set of conveyors into a line:
- **accumulation conveyors**, where boxes queue without pushing;
- **turntables** and **lifts**, which move a section of conveyor and the box on it;
- **merges**, where two lines become one.

Each comes with what makes it work on a real line: the zone logic, the sequence a PLC runs, and the gap control.

```
  spawner ──► C1 [zone 1][zone 2][zone 3][zone 4] ──► C2 turntable ──┐ turns 90°
                accumulates, one box per zone         (BT)           ▼
                                                        C3 ───────────► C4 lift ──► C5 (0.4 m higher)
                                                             (B3)       (BL)         (B5)
```

Try it: **File > New from a demo > Line equipment**, and select C1 to watch its zones fill and empty. Then **Merge**.

---

## Part 1: Accumulation

### 1.1 Why
A line rarely runs at one steady rate. A turntable takes several seconds per box, a palletiser stops to change layers, a jam stops everything downstream. Boxes arriving meanwhile have to wait somewhere. On a plain belt they'd queue nose to tail with the belt sliding under them, pushing on each other: **line pressure**. That crushes soft products, tips tall ones, and can push the front box past the place it should stop.

### 1.2 Zones
A **motor-driven-roller (MDR)** conveyor is split into zones, typically 0.5 to 1 m, each with its own motor roller and a photo-eye near its end. Each zone can stop on its own, so a box can wait in its zone while the zone behind keeps its own box apart. In factory_sim, `zones` > 1 makes a conveyor work that way: each zone ramps its own speed, and the physics gives a box the speed of the zone **under its centre**. The contact callback looks it up (`zoneAt`).

### 1.3 Zero-pressure logic: singulation
The zones are usually run by their own cards, without the PLC. The rule is **zero-pressure accumulation (ZPA)**: one box per zone. Each card knows whether its zone is:
- **empty**;
- **arriving**: the zone behind has passed a box on;
- **full**: a box at its eye.

A zone runs while it's empty or arriving, to take a box in or bring it to its eye. A full zone runs only to pass its box on into an **empty** zone; the last zone passes its box on when the PLC's **release** output says so.

My first version used a common simplification: "a zone runs if its eye is clear or the next zone is running". The test showed two boxes stopped in one zone. While a zone carried its box towards its eye, it counted as "running", so the zone behind handed it a second box. Singulating needs the state: a zone that's carrying a box isn't free just because it's moving.

### 1.4 What the test showed about stopping
The boxes don't stop exactly at their eyes. At 0.5 m/s, slowing at 4 m/s², a box travels 31 mm after its eye says "stop". That's why eyes sit a few centimetres before the zone's end: the box still stops inside its zone. The test checks each box's front stopped between its eye and its zone's end.

### 1.5 What the zone logic can't fix
The demo's spawner kept putting boxes down onto the first zone while it was full and stopped. No zone logic can singulate boxes *dropped onto* a full zone. On a real line, the full first zone would stop whatever feeds it. The demo's infeed comes in bursts (8 s ± 4 s apart), faster than the turntable's cycle for a while, so C1 fills up, then slower, so it drains.

### 1.6 Or the PLC runs the zones
In **PLC** mode, each zone's motor is an output and its eye an input, and the program decides. That's for special cases, like releasing a whole train of boxes at once ("slug release"); local logic is the normal choice. It also took the test to find that every zone had been given the **same** addresses: `freeAddress` didn't know about zone I/O yet, so it kept handing out the same "free" address.

---

## Part 2: Turntables and lifts

### 2.1 A conveyor on an actuator
A turntable is a short conveyor that turns. A lift is one that goes up and down. Both are **a conveyor section on a carriage**, and the carriage moves like the actuators from lesson 4: a dead time, then a steady travel, with a sensor at each end. So a conveyor gets a `Stroke` for its carriage, driven by one output ("turn" or "up"), with "at the end" and "at home" inputs.

`carried(home, position)` works out where the conveyor is for a carriage position from 0 to 1:
- a turntable is turned about the middle of its belt;
- a lift is raised by `liftHeight`.

Each step, the World moves the belt and its guides there. They're **kinematic** bodies, like the pushers' paddles: Jolt doesn't move them, the World tells it where they must be by the end of the step, and they carry and push whatever is on them. A box on a turntable turns with it by friction, and a test checks that it stays put on the table and ends up turned 90° with it.

### 2.2 Geometry is design too
A 0.5 × 0.4 m turntable turning about its middle sweeps a circle of √(0.25² + 0.2²) = 0.32 m. With the usual 2 cm gap to the conveyor before it, its corners would swing 5 cm into that conveyor and hit the next box waiting there. The demo leaves 8 cm (0.33 m from the table's middle), and the inspector shows the sweep for any turntable. Real turntables have exactly this problem, and their layout drawings show the swept circle.

### 2.3 A sequence, as a CASE
The PLC has to choreograph a turntable: take a box, stop, turn, send it off, turn back. `examples/equipment.st` does it as a **state machine**, a `CASE` on a step number, the ST equivalent of a GRAPH sequence in TIA:

```
0: home, empty: belt runs, C1 may release one box   -> a box at BT: step 1
1: stop the belt, turn                               -> turned: step 2
2: run the box off onto C3                           -> table clear for 1 s: step 3
3: turn back                                         -> at home: step 0
```

Each step sets its outputs and waits for one condition, the same pattern as any machine sequence. Three details matter:
- **One box at a time from C1:** in step 0, `release` stays on only until the last zone's eye goes off (an `F_TRIG`): one box has left C1, and no second one follows it into the gap while the table turns.
- **"Clear for 1 s", not just "clear":** a box between the table and C3 can uncover the eye for a moment. A `TON` on "not covered" waits until the box has really gone.
- **Resetting a timer when leaving its step:** calling it with `IN := FALSE` makes it ready for the next cycle.

The lift is the same four steps, going up and down instead of turning. C3 holds a box at its end (B3) while the lift is away: `c3Run := NOT (c3End AND lift <> 0)`.

---

## Part 3: Merges

### 3.1 Joining from the side
A conveyor can **join** another: at right angles, its end at the other's edge partway along it, 2 cm higher so boxes drop onto it. The main belt's friction turns them into its direction, and a guide on the main belt's far side stops them from crossing it. Like snapping (lesson 3), the join is placement worked out from the other conveyor (`placeConveyors`), so moving the main line moves the side one with it.

### 3.2 Gaps
Two lines into one needs a rule for who goes first. In `examples/merge.st` the main line has **priority**: the side conveyor sends a box only when there's a gap. B1, before the merge, sees each main-line box; a `TOF` keeps "busy" on for 2.2 s after a box leaves B1, the time its back needs to clear the merge (1.0 m at 0.5 m/s, plus a margin). A box at the end of the side conveyor (B2) waits there with its belt stopped until then:

```
passing(IN := mainBox, PT := T#2200ms);
sideRun := NOT (sideWaiting AND passing.Q);
```

The first try used 2.6 s with boxes every 3 s on the main line. The side line almost never got a gap, and the test that counted boxes leaving the side line found it starved. Gap control is a trade between safety margin and throughput.

### 3.3 Two boxes, one pulse
The merge test first counted boxes at the main line's end and came up two short. Watching for fallen boxes every second showed none had fallen. The two had merged nose to tail behind main-line boxes, and the photocell at the end saw each pair as one long box. A counter after a merge can be wrong for exactly this reason, so real merges are often followed by a faster belt that pulls boxes apart again.

---

## Part 4: C++ and design

### 4.1 State that the physics doesn't have
Zone states (empty, arriving, full) aren't physics. They're what the zone cards *know*, built from their eyes' edges: a rising edge makes a zone full; a falling edge makes it empty and the next one arriving. The World keeps that state per zone (`zoneState`, `eyeBefore`), the way a real card keeps it in its firmware. The eyes themselves reuse the photocell code: `beamIntervals` was pulled out of `updatePhotocells` so photocells and zone eyes share it.

### 4.2 A design pose and a current pose
A conveyor on a carriage has two placements: where it was **designed** (`home`, from the scene) and where it **is** (`spec`, moved by the carriage). Live edits from the inspector change `home`, and `setConveyorSpecs` keeps the carriage's current position. Everything that asks "where is this belt?" (sensors, the contact callback, the renderer) reads `spec` and so follows the carriage: a photocell on a lift goes up with it.

### 4.3 What counts as layout
Changing a conveyor's number of zones or its kind of carriage changes which bodies exist and how (static or kinematic), so `sameLayout` treats it like moving a conveyor: a new world. A carriage's timing or a zone's address is live.

---

## Part 5: Testing

- **`sim/equipment_test.cpp`** (6 cases):
  - accumulation: one box per zone, stopped between its eye and its zone's end, never touching, then released one zone at a time;
  - PLC-run zones;
  - a turntable turning its belt and the box on it about its middle;
  - a lift carrying its box up;
  - a joined conveyor's placement, saving, and staying put when the main line is removed;
  - zones and carriages saved, and changing the layout.
- **`plc/plc_test.cpp`** (2 new cases):
  - `equipment.st` on soft_plc for 60 s: boxes go round the turntable and up the lift, and none falls anywhere it shouldn't;
  - `merge.st` for 90 s, watched every second: no box knocked off, and the side line's boxes get through.
- **By eye:**
  - the equipment demo with boxes queued in C1's zones, one waiting at the end of C3 for the lift, one on C5 at the top;
  - the merge with side boxes waiting at B2 while a main-line box passes.

---

## Review questions

**1.** Why does a zero-pressure zone only pass its box on into an *empty* zone, rather than into one that's running?
<details><summary>Answer</summary>

A running zone may be carrying its own box towards its eye. Passing another box into it puts two boxes in one zone, and when it stops, they're together. Only an empty zone can take a box and keep one box per zone.
</details>

**2.** A turntable is 0.6 × 0.4 m. How far from its middle must the conveyor before it end, for the table to turn freely?
<details><summary>Answer</summary>

More than half its diagonal: √(0.3² + 0.2²) = 0.36 m, plus a margin.
</details>

**3.** In the turntable's sequence, why does step 2 wait for the eye to be clear for 1 s instead of just clear?
<details><summary>Answer</summary>

A box moving off the table can uncover the eye for a moment before it has really left. Turning back then would catch the box half on and half off. The `TON` waits until the eye has stayed clear long enough for the box to be gone.
</details>

**4.** After a merge, a photocell counts fewer boxes than went through. What happened, and what do real lines do about it?
<details><summary>Answer</summary>

Boxes from the side line merged nose to tail behind main-line boxes, and the photocell saw each pair as one long box. Real lines often put a faster belt after a merge to pull the boxes apart, so sensors downstream see them separately.
</details>
