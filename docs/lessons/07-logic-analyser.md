# Lesson 7: The logic analyser

Milestone 7 adds the instrument a commissioning engineer reaches for when a machine does something odd and nobody can say *when*: a **logic analyser**. It shows every digital signal of the line on one time axis, the belts' speeds under them, two markers to measure between any two moments, histograms of how far the trigger position errors spread, and a log of every switch that can be saved as CSV.

```
Timelines        C1 motor     ▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔
                 BQ inspection        ▁▁▁▁▔▔▔▁▁▁▁▁▁▁▁▁▁
                 P1 valve     ▁▁▁▁▁▁▁▁▁▁▁▔▔▔▔▁▁▁▁▁▁▁▁▁
                 P1 extended  ▁▁▁▁▁▁▁▁▁▁▁▁▁▁▔▁▁▁▁▁▁▁▁▁
                              │A          │B           A-B = 515.0 ms
                 C1 speed     ───────────────────────
Trigger errors   histogram: the sensor's error, and the same as a 10 ms scan sees it
Event log        21.480000  P1 valve  %QX0.0  0     ... Export CSV
```

Try it: `./build/factory_sim`, **File > New from a demo > Sorting**, then **View > Logic analyser**.

---

## Part 1: What gets recorded

### 1.1 Edges, not samples
A digital signal changes rarely: a photocell switches twice per box, a valve twice per push. Since milestone 2 a `Signal` stores only its **changes** (an `Edge`: exact time, new value), and its value at any moment is the last change before it. Milestone 7 gives every digital thing in the line a Signal:

| Trace | Recorded when | New in M7 |
|---|---|---|
| photocell output | the output switches (exact, between steps) | |
| actuator extended / retracted | the cylinder reaches an end (exact) | |
| actuator **valve** | `Stroke::command` switches it (at a scan) | yes |
| conveyor **motor** | the run output changes (at a scan) | yes |
| panel **button** | the button is pressed or released | yes |
| panel **lamp** | the lamp's output changes (at a scan) | yes |

`traces(world)` (`src/analysis/analysis.hpp`) lists them all, each with its name and address, in a fixed order.

Storing changes instead of samples is also how many logic analysers stretch their memory ("transitional" storage). A minute of a photocell at one box per second is 120 edges; sampled every millisecond it would be 60 000 values, almost all saying "same as before". And edges keep the exact time: a sample every 1 ms would round each switch to the millisecond, which is the very thing milestone 2 avoided.

### 1.2 Samples, for what changes all the time
A belt's speed is an analog value that changes continuously during a ramp. Edges don't fit it, so each conveyor keeps a **sample** per scan (`speedSamples`: time and m/s, every 10 ms). That's the other mode of every analyser and oscilloscope: sample at a fixed rate, fast enough for what you want to see (a ramp of 0.5 s gets 50 points).

### 1.3 Bounded memory
A simulation can run for hours, so nothing may grow forever:
- a Signal keeps its last 4 096 edges (`Signal::kHistory`);
- a conveyor keeps its last 12 000 speed samples (two minutes of scans).

Both are `std::deque`s used as a ring: push at the back, pop at the front when full. A deque does both in constant time without moving the rest, which a `std::vector` can't do at the front. The history you can look back over is therefore "the last 4 096 switches" per signal: for a sensor at one box per second, half an hour.

---

## Part 2: Timelines

### 2.1 Drawing a step
A plotting library draws lines between points. A digital signal isn't a line: it holds its value, then jumps. `steps(signal, from, to)` turns the edges into the corners of that shape: its value at `from`, then **two points at each edge** (the old value and the new one, at the same time, giving a vertical jump), then its value at `to`. A line through those corners is the square wave. Only the edges inside the window are used, so the cost depends on what's visible, not on the whole history.

The traces are stacked: trace *k* is drawn between *k* and *k* + 0.7 on one y axis, with each trace's name as the tick label. One plot with many traces, instead of many plots, keeps them perfectly aligned and costs ImPlot one plot setup.

### 2.2 One time axis for both plots
The speed plot sits under the traces and must show exactly the same span. ImPlot can **link** an axis to two `double`s you own (`SetupAxisLinks(ImAxis_X1, &viewFrom_, &viewTo_)`): each plot reads its range from them and writes back any zoom or pan. Two plots linked to the same two doubles move together. The variables live in `App`, so the view survives from frame to frame.

With **follow** on, the app sets them every frame to the last *span* seconds, so the timelines scroll with the simulation. With follow off, they're left alone: pause, and zoom (wheel) and pan (drag) through what happened.

### 2.3 Markers
Two draggable lines, **A** and **B**, and the time between them: the logic analyser's oldest and most useful feature. Drag A onto the valve's rising edge and B onto *P1 extended*'s, and it reads 515 ms: the 15 ms dead time and the 0.5 s stroke from milestone 4, measured on the running line rather than read from the inspector. Drag B onto *BQ inspection* instead and you measure how long after the sensor the sorter fires, which is the encoder distance from lesson 5 turned into time at this belt speed.

---

## Part 3: How far trigger errors spread

### 3.1 The sensor, and the scan
Lesson 2 measured each trigger's **position error**: how far past the beam the box's front was when the output switched. Lesson 5 showed that a PLC sees the output only at its next scan, while the box keeps moving. The **Trigger errors** tab shows both as histograms, with their mean, standard deviation and range:

- **sensor**: in the demo every trigger has the same error, 1.30 mm. At a constant speed, every box covers the beam the same way and the response time is fixed, so there's nothing to vary. A histogram of one bar.
- **at the PLC scan**: the same triggers, plus however far the box moved until the next scan. The output switches at a random moment within the 10 ms cycle (the spawner's jitter makes sure), so the extra delay is anywhere from 0 to 10 ms: 0 to 5 mm at 0.5 m/s.

A delay spread evenly over a cycle is a **uniform distribution**. Its mean is half the cycle (2.5 mm here), its standard deviation the width divided by √12 (5 / 3.46 = 1.44 mm). With only a few dozen triggers, the histogram is lumpy and the numbers come close but not exactly. That spread, not the sensor, is what usually decides whether a triggered camera catches the product in the middle of the picture: milestone 8 builds on it.

### 3.2 What if, without running again
The **PLC scan** slider changes the cycle the histogram assumes, and the histogram changes at once. That works because the simulation recorded the sensor's *exact* output times: `positionErrorsAtScan` works out, after the fact, when a PLC with that cycle would have seen each one (`ceil(outputTime / scan) × scan`) and how far the box had travelled by then. Try 2 ms: the spread shrinks to 1 mm. Recording the exact facts, and applying the model afterwards, makes every "what if" free. (It assumes the scans start at time 0; a real PLC's scans have no fixed phase against the line, which only shifts where each trigger falls in the cycle.)

### 3.3 One range for both
Both histograms use the same range and number of bins, so their bars line up and compare. The range is the extremes of both sets plus a margin: without the margin, the sensor's set (all 1.30 mm) would have a range of zero width and no bins at all.

---

## Part 4: The event log

Every edge of every trace, newest first, with its exact time and address, filtered by part of a name. **Export CSV** writes all of it to `events.csv` next to the scene file:

```
time_s,signal,address,value
17.016152,"BQ inspection",%IX0.0,1
17.360000,"P1 valve",%QX0.0,1
```

Two details:
- **Ordering at the same instant.** A valve, its lamp and the motor can all switch at the same scan, at exactly the same time. `eventLog` sorts with `std::stable_sort`, which keeps equal elements in the order they came in, so those switches always appear in the traces' order instead of an order that could change from run to run.
- **Quoting.** Signal names are the user's, and could hold a comma or a quote. CSV's rule (RFC 4180) is to put such a field in quotes and double any quote inside it: `say "hi", B1` becomes `"say ""hi"", B1"`. Every name is quoted, so a spreadsheet never splits one.

---

## Part 5: C++ and design

### 5.1 Data apart from drawing
`src/analysis/` has no ImGui and no ImPlot: it turns the world into traces, step shapes, events and statistics, and returns plain vectors. `src/app/analyser.cpp` only plots what it's given. So everything that could be wrong (a step in the wrong place, an event out of order, a bad CSV, a wrong deviation) is tested without a window, and the window code is too simple to hide much. It's the same split as `render/scene.hpp` (what to draw) and the renderer (how).

### 5.2 Immediate mode all the way
ImPlot, like Dear ImGui, is **immediate mode**: there are no plot objects to create and update; each frame the code says "a plot, these axes, this line through these points", and ImPlot draws it. The analyser therefore rebuilds its traces, steps and event log every frame from the world. That sounds wasteful but costs little (a few thousand edges at most, and only the visible ones are turned into steps), and it removes a whole class of bugs: there's no copy of the data to fall out of date when the layout changes or the world is rebuilt. `Trace` holds a pointer to a Signal inside the World; since traces are rebuilt every frame, that pointer never outlives the World it points into.

### 5.3 A library that changed its API
ImPlot 1.0 dropped `SetNextFillStyle` and friends: an item's style is now an `ImPlotSpec` argument, built from pairs of property and value: `ImPlotSpec(ImPlotProp_FillColor, colour, ImPlotProp_FillAlpha, 0.6F)`. The constructor is a **variadic template** that takes any number of arguments, with a `static_assert` that their count is even, so a missing value is a compile error rather than a wrong colour. The first version of the analyser was written against the older API, and the compiler said so. The project pins ImPlot to a tag (`GIT_TAG v1.0` in `CMakeLists.txt`), so the API can only change when someone moves the pin, never by itself on a fresh checkout.

### 5.4 Small standard tools
- `std::span<const double>` for `statistics`: it takes a vector, an array or part of one, without copying and without a template.
- `std::minmax_element` finds both ends in one pass, and a structured binding names them: `const auto [low, high] = ...`.
- `std::stable_sort` for the event log (4).

---

## Part 6: Testing

- `analysis/analysis_test.cpp` (6 cases): every signal is a trace with its name and address; a signal's step shape (start value, two points per change, end value, and a flat line with no change inside); the sorter's chain measured on the sorting demo (the valve opens at a scan, and *extended* follows exactly dead time + stroke later); the event log in time order and as CSV, quoting included; statistics of a spread; position errors, the sensor's constant and the scan's spread of up to 5 mm.
- By eye: the timelines following the sorting demo, the markers, the histograms, the event log and its export.

---

## Review questions

**1.** Why does a Signal store its changes instead of its value every millisecond?
<details><summary>Answer</summary>

A digital signal changes rarely, so the changes take far less memory (120 edges a minute for a photocell at one box a second, instead of 60 000 samples), and each change keeps its exact time, where a sample every 1 ms would round it to the millisecond.
</details>

**2.** To draw a square wave with a line plot, why does `steps` put two points at each change?
<details><summary>Answer</summary>

A line between the old value at one change and the new value at the next would slope. Two points at the same time, the old value and the new one, make a vertical jump, and the line between changes stays flat.
</details>

**3.** The sensor's trigger error is always 1.30 mm, at 0.5 m/s. What range do the errors at a 10 ms PLC scan fall in, and why are they spread when the sensor's aren't?
<details><summary>Answer</summary>

1.30 to 6.30 mm. The output switches at a random moment within the scan cycle, and the PLC sees it at the next scan, 0 to 10 ms later: 0 to 5 mm more travel at 0.5 m/s.
</details>

**4.** Three signals switch at the same scan. Why does the event log use `std::stable_sort`?
<details><summary>Answer</summary>

Their times are equal, so the sort alone doesn't decide their order. A stable sort keeps them in the order they came in (the traces' order), so the log, and the CSV, are the same every run.
</details>
