# Lesson 11: More sensors

Milestone 11 adds the sensors the README's goals still listed:
- **inductive and capacitive proximity switches**;
- a **laser distance sensor**, the first analog sensor;
- a **light curtain**.

Each one behaves like the real thing in the way that matters to a PLC program: what it sees, how far, how often, and what it gives the PLC.

```
                    laser BL1 (analog word: distance)
                         │
   tins and boxes ──►  ──┼──────────────  ║║ light curtain BC1 (clear bit, beams broken)
 ═══════[BI1]══[BK1]═════╧═══════════════ ║║
       inductive  capacitive
       (metal)    (anything, cardboard closer)
```

Try it: **File > New from a demo > Sensors**, then **View > Logic analyser**. BI1 pulses for tins only, BK1 for every item, and the curtain's "clear" drops for each. The laser's plot steps between the belt (500 mm away) and the tops of the items.

---

## Part 1: Proximity switches

### 1.1 Inductive and capacitive
A proximity switch has a **switching distance**, sn: an object in front of its face, closer than sn, switches it.
- **Inductive** switches make a high-frequency field with a coil. Metal in the field takes energy from it (eddy currents), and the switch notices. Cardboard, plastic or glass don't do that, so an inductive switch can't see them at all. It's the standard sensor for metal: tins, cylinders' end positions, a cam on a shaft.
- **Capacitive** switches make an electric field, and anything denser than air changes it. They see metal, cardboard, plastic, a liquid behind a tank wall, but a datasheet's sn is for metal, and lighter materials are seen only much closer. Here cardboard counts at 40 % of sn.

`rangeFactor(spec, shape)` holds those rules: tins are metal (1.0); cardboard boxes are 0 for inductive and 0.4 for capacitive.

### 1.2 Distance across, coverage along
The switch looks at two things:
- **Across, the distance**: from the face to the item's near side (beside the belt), or to its top (from above). The item has to be within sn × the material's factor, and 10 % further once the switch is on. That's the hysteresis datasheets give, so an item right at the limit doesn't make it chatter.
- **Along the travel, how much of the face is covered**: as an item passes, it covers more and more of the face. The switch comes on at half covered and goes off below 30 %, after its response time.

The second part is exactly what a photocell does with its beam. So a `Proximity` holds a `Photocell` as its `detector`, whose "beam" is the face. The World works out which items are close enough, and the photocell's code does the switching: exact interpolated switching times, hysteresis and response time. One test checks the switch-on time against the arithmetic: the tin's front at the middle of the face, plus 1 ms.

### 1.3 Mounting matters
In the demo the lane is 100 mm wide between guides, and the switches are M30s on the left guide with sn = 15 mm:
- **Tins** (73 mm) pass 13.5 mm from the face: inside 15 mm, so both switches see them.
- **Boxes** (90 mm wide, cardboard) pass 5 mm from it: inside the capacitive switch's 6 mm (40 % of 15), so only the capacitive one sees them.

A proximity switch on a wide belt, with items wandering across it, would see some items and miss others. That's why on real lines they sit at a guide that keeps items close, or measure something that's always in the same place.

---

## Part 2: The laser distance sensor

### 2.1 What it gives the PLC
It looks straight down and measures the distance to whatever is under its spot: the belt, or an item's top. The PLC gets an **analog word**:
- the measuring range (here 250 to 500 mm) as **0 to 27648**, Siemens' nominal analog range from lesson 10;
- **32767**, the overflow value, when nothing is in range.

Its mounting height minus the distance is the item's height, the classic use: checking heights, telling products apart, measuring stacks.

### 2.2 Measuring like a real one
A real laser sensor doesn't report a perfect, continuous value. Three things shape what it says:
- **Measuring rate:** a new value every `measureTime` (2 ms). In between, the word holds the last one.
- **Resolution:** values come in steps (0.5 mm).
- **Noise:** each measurement is off by a random amount (0.2 mm standard deviation), from a seeded generator, so runs repeat.

With noise smaller than the step, the readings of a still surface sit on one value and its neighbours. A test checks the readings are all on the 0.5 mm grid, average to the true distance, and spread more than zero but less than noise plus a step. A PLC program reading it would filter it (an average over a few readings) before comparing it with a limit, and this sensor makes that need visible.

One count of the word is (500 − 250) / 27648 = 0.009 mm, far finer than the sensor's 0.5 mm resolution. The word's resolution isn't the measurement's. That's true of real analog signals too.

---

## Part 3: The light curtain

A light curtain is a row of photocell beams, one above the other, between an emitter column and a receiver column. It **scans** its beams one after another (only one is lit at a time, so they can't see each other's light), and gives a result once per **scan cycle**, 5 ms here.

It gives the PLC two things:
- **clear**: on while no beam is broken. That's how safety light curtains work: their OSSD outputs are *on* when it's safe, so a broken wire or a dead curtain looks like "not clear" and stops the machine. The same principle as the normally closed stop button in lesson 5.
- **beams broken**, as a word, as a *measuring* curtain gives it: with 10 mm between beams, the count tells an item's height, or with a curtain lying flat, its width.

A test checks a 90 mm box breaks the 9 beams at 10 to 90 mm, and that "clear" only changes on the curtain's own 5 ms cycle. A PLC reading it every 10 ms sees a curtain that itself only looks every 5 ms: two cycles, neither in step with the other, the same layering as lesson 5's photocell and scan.

---

## Part 4: Varied heights

A height measurement needs something to measure, so spawners have a **height jitter**: each item's height varies at random (seeded), around the set height. The item is still placed resting on the belt: when its height changes, its centre moves by half the change.

---

## Part 5: C++ and design

### 5.1 Reuse by composition
A proximity switch *has a* photocell (its switching), rather than *being* one or copying its code. The World sets the photocell's settings from the switch's (the face as the beam, 50 % / 30 % thresholds, the response time), gives it the coverage, and reads its output. The switching code, with its exact interpolated times, exists once, and its tests cover both sensors.

### 5.2 One pass over the items
`updateFieldSensors` first gathers each item's position, rotation, half-size and shape once (`Seen`), then every sensor works from that list. Asking Jolt for each body's position again for every sensor would mean a lock and a lookup each time. Gathering what's needed once is cheap and keeps the sensor loops plain.

### 5.3 A test helper that hid a lesson
The first proximity timing test was 39 ms off. The test put the tin down standing still on a moving belt, so it slid until friction got it up to speed, about 0.1 s and 20 mm behind where the arithmetic placed it. The fix was in the test: put items down moving with the belt, as the sensors demo's spawners do. It's lesson 8's point about tins again: what happens at the infeed changes the timing downstream.

### 5.4 One switch for the material
`rangeFactor` is the only place that knows "inductive sees metal, capacitive sees cardboard closer". The geometry code asks it and multiplies. A new material (glass, water), or a correction factor from a datasheet (stainless steel ≈ 0.7 for inductive), would change one function, not the geometry.

---

## Part 6: Testing

- **`sim/field_sensors_test.cpp`** (8 cases):
  - inductive sees tins and not boxes, capacitive both, with the range deciding;
  - a switch above the belt sees tops within reach;
  - the exact switching times at the face;
  - the laser's distance and its word, with overflow;
  - its noise and resolution;
  - the light curtain's count, its "clear" and its scan cycle;
  - the sensors demo: the inductive switch counts fewer items than the capacitive one, the curtain sees them all, and the laser sees different heights;
  - saving, free addresses, and removal with a conveyor.
- **By eye:**
  - the timelines (BI1 for tins, BK1 for everything, the curtain dropping for each item) and the laser's distance plot;
  - in 3D, the laser's beam down to a tin, the curtain's beams with the broken ones missing, and the switches on the guide.

---

## Review questions

**1.** An inductive switch with sn = 15 mm sits beside the belt. A cardboard box passes 3 mm from its face. Does it switch? And a capacitive one?
<details><summary>Answer</summary>

The inductive one doesn't: it only sees metal, at any distance. The capacitive one does: cardboard is seen within 40 % of 15 mm = 6 mm, and the box is 3 mm away.
</details>

**2.** The laser's range is 250 to 500 mm and it's mounted 500 mm above the belt. A box 100 mm high passes under it. What word does the PLC read, roughly?
<details><summary>Answer</summary>

The distance is 400 mm: (400 − 250) / 250 × 27648 ≈ 16589. With noise and the 0.5 mm resolution, a few counts either side.
</details>

**3.** Why is a light curtain's "clear" output on when nothing is in it, rather than on when something is?
<details><summary>Answer</summary>

So any fault, such as a broken wire, a dead curtain or no power, reads as "not clear" and stops the machine, the way a normally closed stop button does. An output that came on only to say "something's here" would read a broken wire as "all clear".
</details>

**4.** Why does a `Proximity` contain a `Photocell` instead of having its own switching code?
<details><summary>Answer</summary>

Along the travel, a proximity switch switches exactly as a photocell does: by how much of its face (the "beam") is covered, with thresholds, hysteresis and a response time. Reusing the photocell keeps one tested implementation of the exact, interpolated switching for both.
</details>
