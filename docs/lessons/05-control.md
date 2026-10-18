# Lesson 5: I/O and control

Milestone 5 wires the line to a control program, the way a machine is wired to a PLC: every sensor and actuator gets an **address**, a scan copies inputs into an **I/O image**, a **program** reads it and writes outputs, and the outputs drive the valves and motors. The first program is built in: a **sorter** that pushes bad boxes into a bin, using a sensor, the encoder and a pusher.

```
every 10 ms (a scan):
  devices ──► inputs   %IX0.0 BQ sees a bad box     %ID8 encoder count     %IX0.1 P1 extended ...
                 │
              program  (built-in sorting; soft_plc in milestone 6)
                 │
  devices ◄── outputs  %QX0.0 P1 valve              %QX0.1 C1 motor        lamps ...
```

Try it: `./build/factory_sim`, **Load the sorting demo**, and watch the red boxes go into the bin. The **I/O** window shows each address's value as it happens.

---

## Part 1: Addresses and the I/O image

### 1.1 How a PLC sees a machine
A PLC doesn't know what a photocell is. It knows that input bit 0.0 is on. The wiring diagram and the I/O list are what give that bit a meaning ("BQ: bad box detected"). factory_sim works the same way: every device carries the **address** it's wired to, in soft_plc's notation:

| Device | Address | Direction |
|---|---|---|
| a photocell's output | `%IX0.0` (input byte 0, bit 0) | input |
| an actuator's valve | `%QX0.0` | output |
| its extended / retracted sensors | `%IX0.1`, `%IX0.2` | inputs |
| a conveyor's encoder count | `%ID8` (input bytes 8-11) | input |
| a conveyor's motor (optional) | `%QX0.1` | output |
| panel buttons / lamps | `%IX1.0` / `%QX1.0` | input / output |

New devices get the **next free** address (`freeAddress` in `scene.cpp`): bits from byte 0, encoder counts from byte 8 (two double words fit in the 16 bytes). Addresses are stored in the scene, so they don't change when another device is added or removed: a program written against them stays right.

### 1.2 The image
`IoImage` (`src/sim/io.hpp`) is 16 input bytes and 16 output bytes, exactly like soft_plc's process image: bits address one bit of a byte, words and double words are **big-endian** (`%ID8 = 0x12345678` puts 0x12 in byte 8), as on a Siemens S7. That's not an accident: in milestone 6, soft_plc's own image is filled from these bytes and copied back, unchanged.

---

## Part 2: The scan

Every 10 ms (`World::kScanSteps` steps), at the start of a step, the World does what a PLC's operating system does (`World::scan`):

1. **Read inputs.** Clear the input image, then write each device's current state at its address: photocells, end sensors, encoder counts (32 bits, wrapping), buttons.
2. **Run the program.** `program->scan(io, time)`.
3. **Write outputs.** Each valve follows its output bit (`stroke.command(time, bit)`), each conveyor with a run output follows it.

Two consequences, both real:
- **A sensor's change reaches the program at the next scan.** A photocell switching at 12.4315 s is seen at 12.440 s. At 0.5 m/s, that's up to 5 mm of travel, the "at scan" column in the Triggers window, which now varies from box to box because the spawner's jitter puts boxes at random moments relative to the scan.
- **Outputs hold between scans.** The valve opens at the scan that set it, not when the program "thought" of it.

Without a program, nothing drives the outputs: actuators are moved by hand in the inspector, and conveyors always run.

### Normally closed buttons
A stop button is wired **normally closed**: its contact is closed (the input is TRUE) while nobody presses it, and opens when pressed. If the wire breaks, the input goes FALSE, which looks like "stop pressed", and the machine stops: the failure is safe. The panel's buttons model that (`normallyClosed`): an NC button reads TRUE until pressed. The ladder_editor example's `stop` and `eStop` are written for exactly that.

---

## Part 3: The sorter

### 3.1 The logic
The built-in sorter (`BuiltInSorting` in `control.cpp`) is written the way a PLC program would be, scan by scan, knowing only addresses:

```
rising edge of the NOK sensor        →  remember "fire at encoder count + distance"
count has reached the first value    →  valve on, forget that value
valve on and the pusher is extended  →  valve off
```

The remembered counts are a **queue** (a FIFO, or "shift register" in PLC terms): several bad boxes can be between the sensor and the pusher at once, and each gets its own firing point, in order.

### 3.2 Why the encoder, not a timer
"Fire 3.9 s after the sensor" works only if the belt runs at exactly the speed it was tuned for. Belts ramp up and down, slip, and get their speed changed. The encoder measures **distance**: "fire 12 826 pulses after the sensor" (the demo's setting) puts the box at the same place whatever the speed did in between. Every sorting, labelling and vision system on a conveyor tracks products this way.

### 3.3 Counters wrap
The count is a 32-bit value that wraps around (after 4.3 billion pulses, about 660 km of belt at 6 519 pulses/m, it goes back to 0). "Has the count reached the target?" is therefore computed on the **difference**, read as a signed number: `int32_t(count - target) >= 0`. That's true just after the target even across the wrap, as long as targets are less than 2^31 pulses ahead. Comparing `count >= target` directly would fire a box whose target lies past the wrap at once, long before it arrives: just before the wrap, the count is huge and the target small. It's the same arithmetic TCP uses for sequence numbers.

### 3.4 Where to fire
The sensor switches when a box's **front** edge reaches it. The paddle should meet the box's **middle**, and it isn't instant: after firing, there's the dead time and half the stroke before the paddle is halfway across the belt, and the box moves on meanwhile. So `sorterFor` computes:

```
distance = (pusher − sensor) + box length / 2 − speed × (dead time + stroke / 2)
```

and converts it to pulses. The time part depends on the belt's set speed: change the speed and the sorter needs a new distance. Real sorters compensate exactly like this (it's often called a **lead** or **advance** setting), and the scan adds up to one cycle of uncertainty on top: a 30 cm paddle for a 20 cm box leaves room for it.

### 3.5 The result
The demo runs 60 boxes in 60 s, a third of them bad at random (from the seeded generator, the same ones every run). The test checks that every bad box that had time to reach the pusher is in the bin, and **no good one**: in one run, 15 bad, 14 in the bin, and the 15th was spawned too late to get there.

---

## Part 4: Bad boxes, and seeing them

Spawners make a share of their boxes **bad** (`nokFraction`), drawn red. Something has to tell the program which ones: on a real line, an inspection camera or a colour sensor. Until milestone 8 builds a camera, a photocell can be set to **see only bad boxes** (`onlyNok`): it stands in for "the inspection system's NOK output". The bin now counts good and bad boxes separately, which is how the test knows the sorter never rejected a good one.

---

## Part 5: C++ and design

### 5.1 A seam
`ControlProgram` has one function: `scan(IoImage&, double time)`. Anything that reads and writes the image can control the line. That's the point where soft_plc will plug in (milestone 6), and the reason the built-in sorter works through addresses instead of reaching into the World's photocells and pushers: if it can run against the image, so can a PLC program. Topic 6 (design notes) has a section on choosing such seams.

### 5.2 `dynamic_cast` for a status panel
The main panel shows each sorter's "fired / tracking" counts. Only `BuiltInSorting` has them, so the panel asks `dynamic_cast<const BuiltInSorting*>(world_->program())` and shows them if that's what's running. It's the one place the UI needs to know which program it is; everything else goes through the interface.

### 5.3 Matching text loosely
Scripted edits to the code kept failing when clang-format had reflowed a line since it was read. The editing helper now falls back to matching with any whitespace between the same tokens. A small tool, but the lesson generalises: when one program edits another's output, compare meaning, not layout.

---

## Part 6: Testing

- `sim/control_test.cpp` (6 cases): addresses parsed and written as soft_plc does (and bad ones refused); big-endian words; next free addresses for new devices (and running out of double words); a scan reading devices and normally closed buttons; a program running every 10 ms and driving a conveyor's motor through its run output; the sorting demo for 60 s, every bad box that could arrive in the bin and no good one.
- By eye: the demo running, the sorter's counts, the I/O table's live values, the triggers' scan error spreading out.

---

## Review questions

**1.** A photocell switches at 7.4315 s. When does the program see it, and how far has a box at 0.5 m/s moved meanwhile?
<details><summary>Answer</summary>

At the next scan, 7.440 s: 8.5 ms later, so the box has moved 4.25 mm.
</details>

**2.** Why does the sorter count encoder pulses instead of waiting a fixed time after the sensor?
<details><summary>Answer</summary>

Pulses measure distance: the box is at the pusher after the same number of pulses whatever the belt's speed did in between (ramps, slips, speed changes). A fixed time is only right at exactly the speed it was tuned for.
</details>

**3.** The encoder count wraps from 4 294 967 295 to 0. A box is waiting to be fired at count 100. Why does `int32_t(count − target) >= 0` still fire it at the right moment?
<details><summary>Answer</summary>

Just before the wrap, count − 100 is a huge unsigned number which, read as signed, is negative: not yet. After the wrap, at count 100, the difference is 0: fire. Comparing `count >= target` directly would have fired at once, since 4 294 967 295 ≥ 100.
</details>

**4.** A stop button is wired normally closed. What does the program read while nobody presses it, and why is that the safe choice?
<details><summary>Answer</summary>

TRUE (the contact is closed). If the wire breaks, the input reads FALSE, the same as "stop pressed", so a broken wire stops the machine instead of making the stop button useless.
</details>
