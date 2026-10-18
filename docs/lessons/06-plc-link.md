# Lesson 6: A PLC controls the line

Milestone 6 puts a real PLC program in charge of the simulated line. The first three projects meet here: a program drawn in **ladder_editor**, run by **soft_plc**, drives a conveyor in **factory_sim**, and ladder_editor watches it online.

```
ladder_editor ── Export ST ──► conveyor.st
      │                            │ loaded by
      │                            ▼
      │  monitoring      factory_sim: each 10 ms scan
      └── (port 5020) ──►   inputs ──► soft_plc: one cycle ──► outputs
          power flow,       (I/O image ⇄ soft_plc's process image, on simulated time)
          forcing,
          Download (online change)
```

Try it: `./build/factory_sim examples/conveyor_line.json`, press **Start** on the control panel, then in another terminal `ladder_editor --connect=5020 ../ladder_editor/examples/conveyor` and watch the rungs follow the belt.

---

## Part 1: Lockstep

### 1.1 One scan, one cycle
Milestone 5 built a seam: a control program sees the line only through the I/O image, once per 10 ms scan. `SoftPlcProgram` (`src/plc/soft_plc_program.hpp`) plugs soft_plc into it. Each scan:

1. the simulator's 16 input bytes are copied into soft_plc's process image (`%I`);
2. soft_plc runs **one cycle** of the ST program;
3. soft_plc's output bytes (`%Q`) are copied back into the I/O image.

The copy is byte for byte: milestone 5 made the I/O image's layout exactly soft_plc's (same bit numbering, big-endian words), so `%IX0.3` in factory_sim is `%IX0.3` in the ST program, with no translation.

How the copy is wired is itself a nice fit: soft_plc's scan cycle reads inputs and writes outputs through an **I/O driver** (`IoDriver`, its own seam for hardware). factory_sim gives it a driver that reads from and writes to the simulator's image (`LockstepIo`), and runs `ScanCycle::runOnce` once per scan. soft_plc doesn't know it's in a simulator.

### 1.2 Simulated time
The cycle is given the **simulated** time as its clock (`TimePoint{} + time`), not the computer's. That's what lockstep means: a TON of 2 s in the program counts 2 simulated seconds, so:
- at 0.1x, the program's timers slow down with everything else;
- at 10x, or in a test running 60 simulated seconds in a fraction of one, they keep up exactly;
- **Step 1 ms** steps the program too (it runs on every tenth step);
- every run gives the same result: the line and the program are one deterministic system.

A test checks it: 200 scans are 1.99 simulated seconds, and a 2 s timer is still off; one more scan and it's on, whatever the real time was.

### 1.3 STOP
A runtime error in the program (a division by zero) puts soft_plc in STOP, as in soft_plc itself: its outputs go off, and the panel shows the error with its line and column. The line keeps running with every output off: motors stopped, valves closed. That's what a real machine does when its PLC stops, and it's worth seeing in a simulator before seeing it on a machine.

---

## Part 2: Going online with the simulated PLC

### 2.1 The monitoring port
With a **monitor port**, the soft_plc inside factory_sim listens for soft_plc's monitoring protocol, and answers requests **between cycles**, as soft_plc does (lesson 5 of soft_plc). So ladder_editor's **Connect** works exactly as it does with a standalone soft_plc:
- power flow on the rungs as boxes pass the light barriers;
- values in the watch table and on box pins;
- forcing an input (the screenshot for this lesson started the belt by forcing `start`);
- **Download**: the online change from ladder_editor's milestone 11 (`LOAD`). Edit a rung, Download, and the running program changes between two cycles, keeping its values, while the belt keeps running.

The only difference is the clock: requests are answered between *simulated* cycles. At 1x that's every 10 ms of real time; paused, the PLC isn't running, so the editor waits until you press Run or Step.

### 2.2 The port, and the order of destruction
A monitoring server owns a TCP port. When the layout changes, the World is rebuilt, and its soft_plc with it. The natural `world_ = buildWorld(...)` builds the new world *before* the assignment drops the old one, so the new server would try to listen on a port the old one still holds. `App::rebuildWorld` therefore resets the old world first: the same "destroy before you recreate" rule as any resource that can only be held once (a file lock, a serial port, a GPU context).

---

## Part 3: A live PLC over S7

### 3.1 Real time, no alignment
**S7 PLC (live)** connects to a PLC that runs on its own: `soft_plc --s7=1102 program.st`, or a real S7. Every scan, `S7Link` (`src/plc/s7_link.hpp`) writes the simulator's inputs to the PLC's input area (`EBWrite`, %I) and reads its outputs (`ABRead`, %Q), with Snap7's client.

This is the opposite trade-off to lockstep:

| | Lockstep (soft_plc inside) | Live (S7) |
|---|---|---|
| clocks | one (simulated) | two (simulated and the PLC's) |
| speed | any time scale, single steps | real time only |
| repeatable | exactly | no: cycles drift against each other |
| PLC | soft_plc, ST | anything that speaks S7, including a real one |
| timing | ideal | network latency and jitter, like on a real line |

Lockstep is for developing and measuring; live is for testing the real thing, timing warts included.

### 3.2 Failing safe
If the PLC isn't there, or the connection drops, the outputs read as **all off**, and the panel says why. Connecting blocks, so it's tried at most once a second: a dead PLC mustn't freeze every scan. A test checks both directions against soft_plc's own S7 server, and the "nobody there" case.

---

## Part 4: The conveyor demo

`conveyorDemo` builds the line ladder_editor's conveyor example expects, wired to its addresses:

| Program | Address | In factory_sim |
|---|---|---|
| `start`, `stop`, `eStop`, `resetButton` | `%IX0.0`-`%IX0.2`, `%IX0.5` | control panel buttons (Stop and E-stop normally closed; E-stop latching) |
| `boxAtStart`, `boxAtEnd` | `%IX0.3`, `%IX0.4` | photocells near the belt's start and end |
| motor (`%QX0.0`) | `%QX0.0` | the conveyor's run output, and a lamp |
| `warningLamp`, `batchFull`, `tooFast` | `%QX0.1`-`%QX0.3` | lamps |

The ST file is ladder_editor's **Export ST** of that project, kept in `examples/conveyor.st`. A test runs it: Start makes the belt run and hold itself, the program counts boxes at the end barrier, Stop stops it.

To see the jam detection: let a box stop in the start barrier (press Stop just as it gets there), wait 8 s, and the Jam lamp blinks; Reset clears it once the end barrier is free.

### An infeed that waits
With the belt stopped, the spawner used to drop box after box on the same spot, until they flew apart. A real infeed waits for a gap: the spawner now skips its turn while its spot is taken, and its rhythm starts again from when it can put a box down.

---

## Part 5: C++ and design

### 5.1 Two seams meeting
The lockstep link is two seams plugged into each other: factory_sim's `ControlProgram` (what controls the line) and soft_plc's `IoDriver` (where the PLC's I/O comes from). Neither project was written for the other, yet joining them took one small class each way. That's the payoff of choosing seams where they belong.

### 5.2 Keeping dependencies inside
`SoftPlcProgram` and `S7Link` hold soft_plc's and Snap7's objects behind a pimpl or an opaque handle: their headers name none of those types. The app includes `plc/soft_plc_program.hpp` without seeing soft_plc's headers, and `factory_sim_plc` links soft_plc and Snap7 privately.

### 5.3 A program maker
`buildWorld` lives in the core library, which doesn't link soft_plc, but has to give the world its program. It takes a **program maker** (`std::function<std::unique_ptr<ControlProgram>(const Scene&)>`), defaulting to the built-in one; the app passes one that knows the PLC kinds too. The dependency points the right way: the core library asks "make me a program", and the code that knows soft_plc answers.

---

## Part 6: Testing

- `plc/plc_test.cpp` (7 cases): a scan is one cycle (inputs in, outputs out, variables readable); timers on simulated time; STOP with outputs off, and a compile error with its line; a monitoring client on another thread reading a value and loading a new version (an online change) while the simulation runs; an S7 link writing inputs to and reading outputs from soft_plc's own S7 server; no PLC there, all outputs off; ladder_editor's conveyor program running the simulated conveyor (start, hold, count, stop).
- `sim/world_test.cpp` (1 new case): a stopped belt gets one box from its spawner, not a pile, and more once it runs.
- By eye: the conveyor demo running under soft_plc, started over the monitor port, with ladder_editor connected to it showing power flow.

---

## Review questions

**1.** The simulation runs at 0.1x. Does the program's 8 s jam timer take 8 or 80 real seconds?
<details><summary>Answer</summary>

80 real seconds: in lockstep, soft_plc's clock is the simulated time, which runs at a tenth of real time. In simulated time it's still 8 s, so the program behaves exactly as at 1x.
</details>

**2.** Why must the old world be destroyed before the new one is built, when the program has a monitor port?
<details><summary>Answer</summary>

The old world's soft_plc holds the TCP port. A new one built first would try to listen on a port that's still taken, and fail. Destroying the old world first frees the port.
</details>

**3.** Name one thing the live S7 link can do that lockstep can't, and one thing it can't do.
<details><summary>Answer</summary>

It can test a real PLC (any S7, not just soft_plc), with real network timing. It can't run faster or slower than real time, single-step, or give exactly the same result twice: the two clocks aren't aligned.
</details>

**4.** The S7 connection drops. What do the simulated valves and motors do, and why?
<details><summary>Answer</summary>

They go off: the link reads all outputs as off when it has no PLC. Off is the safe state, as on a real machine whose PLC stops or loses its I/O.
</details>
