# Lesson 2: Sensors that behave like real ones

Milestone 2 adds the first sensors: a **photocell** that switches when a box crosses its beam, with the hysteresis and delay of a real one, and an **encoder** on the conveyor's drive roller. Every switch is recorded with its **exact** time, and each trigger says where the box really was when the signal arrived: the **trigger position error**, the number that matters for cameras, pushers and anything that acts on a moving box.

```
box front edge ──► beam covered 60 % ──► + response time ──► output ON ──► next PLC scan ──► program sees it
                   beamTime              (sensor delay)      outputTime    (up to one cycle)
                   │                                          │              │
position error:    0.1 × beam ø past the centre    + speed × response    + speed × scan wait
```

Try it: `./build/factory_sim`. The **Triggers** window fills up every 1.2 s. Set the response time to 10 ms or the belt to 2 m/s and watch the error column grow.

---

## Part 1: The photocell

### 1.1 A beam has a size
A photocell's beam isn't a line: it's a spot a few millimetres across, and the receiver measures how much light gets through. So the model asks, every step, **what fraction of the beam is covered** by boxes (`beamCoverage` in `sensors.cpp`): each box's extent along the belt is clipped to the beam, overlapping boxes count once, and the result is between 0 and 1.

A box only counts if it's actually in the beam's path: across the belt (within its width) and reaching the beam's height. A flat box lower than the beam passes underneath it.

### 1.2 Hysteresis
The sensor switches **on** when more than 60 % of the beam is covered and **off** when less than 40 % is. Between the two, it keeps its state. Without that gap, an edge sitting right at the threshold (a box vibrating, a wobbly label) would make the output chatter on and off.

Two consequences that are easy to get wrong (the first version of a test got the second one wrong):
- **A minimum object size.** A box shorter than 60 % of the beam's diameter can never cover enough of it: with an 8 mm beam, anything under 4.8 mm is invisible.
- **The pulse isn't shorter.** "On" happens when the front edge is 0.1 × d past the beam's centre (60 % covered); "off" when the rear edge is *also* 0.1 × d past it (only 40 % still covered). So the output stays on for exactly the box's length at its speed, shifted by 0.1 × d. Symmetric thresholds move the window, they don't shrink it.

### 1.3 The response time
A real photocell's electronics filter the received light before switching, which takes time: typically 0.25 to 2 ms. The model delays the output by the response time: every change of the beam's state is queued (`pending`) and applied to the output when its time comes. A box at 2 m/s moves 2 mm per millisecond of response time.

---

## Part 2: Exact times in a stepped simulation

### 2.1 Between two steps
The simulation knows where everything is at 0 ms, 1 ms, 2 ms… A front edge reaches the beam somewhere *between* two steps. Rounding the switch to the next step would add up to 1 ms of error: 2 mm at 2 m/s, as big as the effects we want to measure.

So each step compares the coverage at its start and at its end. If it crossed a threshold in between, the crossing time is found by **linear interpolation**:

```
coverage
  1 ┤            ● end of step (0.9)
0.6 ┤ ─ ─ ─ ─ ╱─ ─ ─ ─   threshold
    │        ╱
  0 ┤ ●─────╯            start of step (0.2)
    └─────────────── time
      t        t+1 ms
      crossing = t + 1 ms × (0.6 - 0.2) / (0.9 - 0.2)
```

Within one millisecond a box moves evenly, so the line is a very good approximation. The tests check it: with 1 ms steps, the switching time comes out within **50 µs** of the time worked out by hand.

### 2.2 Signals keep their times
`Signal` (`signal.hpp`) stores each change as an `Edge {time, value}`, with the exact time as a `double`. It answers "what was the value at time t?" with a binary search over its changes. Milestone 7's logic analyser will draw these timelines; milestone 6's PLC link will sample them at each cycle.

### 2.3 Where the box was "then"
When the output switches, the step has already moved on a little (the output time is somewhere inside the step). The trigger record goes **back** from where the box is at the end of the step, at its current velocity, to where it was at the output time. The see-through ghost in the view is drawn there, and the position error is measured from it: how far the box's front edge was past the beam's centre when the signal arrived.

---

## Part 3: The encoder

An encoder on the drive roller gives pulses as the belt moves: `pulsesPerRevolution` per turn of a roller of `rollerDiameter`, so **pulses per metre = PPR / (π × D)**. With 1024 pulses and a 50 mm roller: 6519 pulses/m, 0.153 mm per pulse, 3259 pulses per second at 0.5 m/s.

The model keeps the belt's total travel and turns it into a count (`encoderCount`). A trigger records the count at the **output time**, interpolating the travel inside the step, not at the step: that's how a camera system or a pusher tracks a box. "Fire the pusher 1500 pulses after the photocell" works whatever the belt's speed, even while it ramps up or down, because the encoder measures distance, not time.

---

## Part 4: What a PLC adds

A PLC doesn't see the output change: it reads its inputs **once per cycle**. The trigger reaches the program at the next cycle start after the output switches, and the box keeps moving meanwhile. The **at scan** column adds that wait for a PLC with the chosen cycle time.

In the example, every box shows exactly the same scan error. That's not a bug: the boxes come every 1.2 s, an exact multiple of the 10 ms scan, so every box meets the scan at the same moment. On a real line, boxes come at random moments relative to the scan, and the extra error is spread anywhere between 0 and one full cycle: at 2 m/s and 10 ms, **up to 20 mm**. That's why fast triggers (cameras, rejectors) are wired to hardware inputs, not read by the PLC program. Add jitter to the spawner and watch the column spread out.

---

## Part 5: C++ and design

### 5.1 One helper for one idea
"How far does a turned box reach along a direction?" (each half-size, projected onto it) was written twice, once to see whether a box is in the beam and once to measure its front edge. It's now one function, `reach()`. Two copies of a formula drift apart; one name for it also documents what it means.

### 5.2 Drawing without OpenGL, again
The photocell's housing, reflector, beam and LED are four more `DrawBox`es from `photocellBoxes()`, tested without a window: the beam at the right height, across the belt, the LED changing colour with the output. The renderer only learned two new things: see-through boxes (drawn after the solid ones, blended, without writing depth so they don't hide each other) and unlit ones (a beam or an LED is a light, not a surface).

### 5.3 A raw newline in a string
While adding the tooltips, a Python edit turned the `\n` inside a C++ string into a real line break, and the compiler complained about a "missing terminating \" character". When code is generated or edited by another program, escapes are the first thing to check.

---

## Part 6: Testing

- `sim/sensors_test.cpp` (7 cases): coverage (clipping, overlaps); the switching time within 50 µs of the hand-worked time, though steps are 1 ms; output delay and position error for three response times; hysteresis giving a pulse exactly as long as the box; objects too short or too low not seen; pulses per metre and the count after 1 m; the encoder count at the output time, not the step.
- `sim/world_test.cpp` (1 new case): a signal's exact change times and its value at any time.
- `render/scene_test.cpp` (1 new case): the photocell's parts, the beam's height, the LED following the output.
- By eye: the beam, the LED, the ghost, and the trigger table matching the numbers worked out by hand (1.30 mm = 0.8 mm for the threshold + 0.5 mm for 1 ms at 0.5 m/s).

---

## Review questions

**1.** An 8 mm beam switches on at 60 % covered. What's the smallest box it can see, and why?
<details><summary>Answer</summary>

4.8 mm (60 % of 8 mm) along the belt. A shorter box can never cover enough of the beam to pass the "on" threshold, however it's placed.
</details>

**2.** The photocell has 60 % / 40 % hysteresis. Does a 20 cm box at 0.5 m/s give an output pulse shorter than 0.4 s?
<details><summary>Answer</summary>

No, exactly 0.4 s. It switches on when the front edge is 0.1 × d past the beam's centre, and off when the rear edge is also 0.1 × d past it. The hysteresis shifts the pulse by 0.1 × d; it doesn't shorten it.
</details>

**3.** Why is the switching time interpolated, instead of taken as the step where the coverage changed?
<details><summary>Answer</summary>

Rounding to the step adds up to 1 ms of error, 2 mm at 2 m/s, which is as large as the effects being measured. Within one step a box moves evenly, so interpolating the coverage between the step's start and end gives the crossing time to within tens of microseconds.
</details>

**4.** At 2 m/s, a photocell with a 1 ms response is read by a PLC with a 10 ms cycle. What's the largest possible position error, ignoring the beam's threshold?
<details><summary>Answer</summary>

22 mm: 2 mm from the response time (2 m/s × 1 ms) plus up to 20 mm waiting for the next cycle (2 m/s × 10 ms).
</details>
