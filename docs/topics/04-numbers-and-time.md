# 4. Numbers and time

A simulator that measures millimetres and microseconds lives or dies by its numbers. Floating point arithmetic is fast and usually good enough, but it has edges, and factory_sim runs into several of them. This note explains each one and what the code does about it.

---

## 1. float and double

A `float` has 24 bits of precision: about **7 significant digits**. A `double` has 53: about **16**. "Significant" is the important word: the precision follows the number's size.

| Value | float steps there | So a float can't tell apart |
|---|---|---|
| 1 m | 0.00000012 m | nothing that matters |
| 1 000 m | 0.00006 m | 0.06 mm |
| 100 000 m | 0.008 m | 8 mm |

factory_sim uses `float` for geometry and speeds (sizes, positions, m/s: a factory is metres to tens of metres, and GPUs and Jolt work in floats) and `double` for **time** and anything that grows forever.

---

## 2. Things that grow forever

### Time is counted, not added up
The obvious way to keep time is `time += step` every step. After a million steps, each addition has rounded a little, and the errors pile up. factory_sim counts **whole steps** in an integer and computes the time from it:

```cpp
double time() const { return static_cast<double>(stepCount_) * kStep; }
```

An integer never drifts, and one multiplication rounds once. After a day of simulated time (86.4 million steps), the time is still exact to well under a microsecond.

### A belt's travel, and the stripes
`Conveyor::travelled` is a `double`: at 2 m/s it reaches 172 km in a day, where a float couldn't tell millimetres apart any more. But the GPU works in floats. So the scene sends the shader only the travel **modulo the stripe spacing** (`std::fmod(travelled, kStripeSpacing)`), always under 25 cm, where a float is precise to a fraction of a micrometre. The stripes would otherwise start to stutter after a few hours.

The general trick: keep big, growing quantities in doubles (or integers), and hand the float-only parts of the system only **small, relative** numbers.

### The encoder count
The encoder's count is `floor(travelled × pulses per metre)`, an `int64_t`. It's derived from the travel every time, not incremented per step, for the same reason as the time: one rounding instead of millions.

---

## 3. Comparing numbers that were computed

`0.1 + 0.2 == 0.3` is false in floating point. Any comparison between computed values needs a tolerance, chosen for what the numbers mean.

**The fixed timestep** (`World::advance`) runs a step while the backlog holds at least one: `pending_ + 1e-9 >= kStep`. Without the 1e-9, a frame of exactly 10.5 ms could run 9 steps instead of 10, because 0.0105 − 9 × 0.001 might come out as 0.000999999… The tolerance is a nanosecond: far below anything the simulation cares about, far above the rounding error.

**Spawning** uses the same idea (`time() + 1e-9 < state.nextSpawn`), and so does the PLC scan in the trigger table (`ceil(outputTime / scan − 1e-9)`), so an output exactly on a scan boundary counts as seen at that scan, not the next one.

**Tests** compare with `doctest::Approx`, and the tolerance says something: the photocell's switching time is checked to within **50 µs**, which is the real claim (sub-step precision), not "about equal".

---

## 4. A round trip that must be exact

The scene is saved to JSON and read back, and a test checks the result is `==` to the original. That forced a decision: the file stores the conveyor heading in **radians**, as the code uses it, not degrees. Converting `heading × 180/π` on save and back on load can change the last bit of a float, and then the loaded scene is no longer equal to the saved one: the undo history would see a change that never happened. A file a person reads would prefer degrees; a file the program reads back exactly needs the value as it is. Since the scene file isn't meant to be edited by hand, exactness won.

---

## 5. Interpolating inside a step

Between two steps, nothing is known. When the photocell's coverage crosses its threshold, the crossing time is found by **linear interpolation** between the step's start and end:

```
fraction = (threshold − before) / (now − before)
crossing = stepStart + 1 ms × fraction
```

Two numeric details:
- If `now == before`, the division would be by zero. The code checks for it (`now == before ? 1.0F : …`), even though a threshold crossing implies the coverage changed: a guard costs nothing and protects against the case nobody thought of.
- The fraction is clamped to [0, 1]: rounding can push it a hair outside, and a crossing time outside its own step would make an output switch before its cause.

The error of the interpolation itself comes from assuming the box moves evenly within the step. In 1 ms a box barely accelerates, so the tests find the crossing within tens of microseconds of the exact time.

---

## 6. Determinism

"Same inputs, same outputs" is what makes a simulation testable and a measurement repeatable. factory_sim gets it by removing every source of variation:
- **one thread** for the physics (`JobSystemSingleThreaded`): with several threads, the order in which contacts are solved could change from run to run, and with it the last bits of every position;
- **a fixed step**, independent of the frame rate;
- **a fixed random seed** for jittered spawning (`std::mt19937 random_{42}`): "random" spacing that's the same sequence every run.

What it does **not** promise: the same results across different compilers, optimisation levels or CPUs. Optimisers may reorder floating point operations or fuse a multiply and an add into one instruction, changing the last bits. Jolt has a "cross-platform deterministic" build mode for that, which factory_sim doesn't use. Since milestone 3, Jolt is also compiled optimised in Debug builds, so Debug and Release are closer than before, but nothing guarantees they match exactly. The tests therefore use tolerances that are about physics (a box at the belt's speed ± 2 %), not about luck.

---

## Review questions

**1.** Why is the simulated time `stepCount × 1 ms` instead of adding 1 ms every step?
<details><summary>Answer</summary>

Adding rounds a little every step, and the errors pile up over millions of steps. Counting whole steps in an integer is exact, and the single multiplication rounds once.
</details>

**2.** The belt has travelled 150 km. Why do the stripes still move smoothly?
<details><summary>Answer</summary>

The travel is kept in a double (precise to nanometres at that size), and the shader only gets the travel modulo 25 cm, a small number a float holds to well under a micrometre.
</details>

**3.** Without the 1e-9 in `advance()`, what could happen with a frame of exactly 10.5 ms?
<details><summary>Answer</summary>

After nine steps, the backlog might come out as 0.000999999… instead of 0.001, just below one step, so only 9 steps would run instead of 10; the missing step would come a frame later, making the motion uneven.
</details>

**4.** Why does the scene file store the heading in radians?
<details><summary>Answer</summary>

Converting to degrees and back can change the last bit of the float, so a saved and reloaded scene wouldn't be exactly equal to the original, and the undo history would see a change that didn't happen. Radians are stored exactly as the code uses them.
</details>
