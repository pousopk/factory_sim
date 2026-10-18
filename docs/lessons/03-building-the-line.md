# Lesson 3: Building the line

Milestone 3 turns the fixed demo into something you can build: conveyors that snap together, photocells and spawners placed on them, an inspector, undo, and a scene saved after every change. The main idea underneath is to separate **what the line is** (the scene) from **what's happening on it** (the world).

```
             panels edit                      every frame
Inspector ──────────────► Scene ──────────────────────────► World
Line panel                 │  (a value)      same layout?  yes: update settings live (boxes keep running)
click in 3D                │                               no:  build a new world (the run restarts)
                           ├─► undo history (snapshots)
                           └─► factory_sim_scene.json (after every change)
```

Try it: start `./build/factory_sim`, click the conveyor, press **Left** in the Line panel, then lengthen the first conveyor in the inspector: the new one moves with it.

---

## Part 1: Scene and world

### 1.1 Two different things
Until now the `World` was everything: the conveyors' sizes, the sensors' settings, the boxes, their speeds, the time. But those are different kinds of information:
- **The design**: three conveyors, this long, a photocell here. It's what you save, undo, and come back to tomorrow.
- **The run**: where each box is right now, the simulated time, each photocell's triggers. It's thrown away when you restart.

So the design is now a `Scene` (`src/sim/scene.hpp`): plain vectors of specs, with `operator==` defaulted. The `World` is **built from it** (`buildWorld`) and never edited directly by the panels. It's the same split as `ladder_editor`'s project (saved, undone) and soft_plc's running program.

### 1.2 Live, or rebuild?
When the scene changes, the world has to follow. Two cases:
- **Settings that don't move anything**: a belt's speed, a photocell's response time or position, a spawner's rate. The running world takes them as they are (`applyLive`): the boxes keep moving, the photocell keeps its triggers.
- **The layout**: a conveyor's place or size, or adding and removing conveyors. The physics bodies were made for the old layout, and Jolt can't stretch a static box; boxes may be sitting where a belt no longer is. So a new world is built, and the run restarts.

`sameLayout(a, b)` decides which: same number of conveyors, each with the same placed start, heading, length, width and height. Everything else is live. It's the same question soft_plc's online change answers for variables ("what can carry over?"), at a coarser level.

### 1.3 Photocells keep their state, or not
A live change replaces each photocell's settings but keeps its state: its output, its pending switches, its trigger history. Except when the photocell moved to **another conveyor**: then it's really a different sensor, and starts afresh (`World::setPhotocells`). Choosing what "the same thing" means is part of every live update.

---

## Part 2: Snapping

### 2.1 Placement that follows
A conveyor can **follow** another: then its start and direction aren't stored at all, they're worked out from the end of the conveyor before it (`placeConveyors`):
- **straight on**: the same direction, starting 2 cm past the end (the transfer gap);
- **turned left or right**: 90 degrees, its centre line just past the end, starting half the first belt's width back so a box coming off lands fully on it.

Because the position is *derived*, moving or lengthening the first conveyor carries the whole chain, with no code to "update the others". It's "derive, don't store" again: a stored copy of the position would have to be kept in step with the conveyor before it, and sooner or later it wouldn't be.

### 2.2 Always an earlier one
A conveyor may only follow one that comes **before** it in the list. Then one pass from first to last places everything, because every conveyor's predecessor is already placed when it's reached. It also makes loops impossible (A follows B follows A), so there's no need to detect them. The file loader enforces it ("follows conveyor 3, which doesn't come before it").

### 2.3 Removing a link in the chain
Deleting a conveyor that others follow: they **keep their place**, now on their own (`removeConveyor` bakes their derived start and heading into their spec before cutting the link). Everything after it in the list moves down by one index, so every reference to a conveyor index (photocells, spawners, other conveyors) is shifted, and what stood on the deleted conveyor goes with it.

Indices are simple and fast, but this is their price: every delete has to fix them up. Stable IDs (a number that never changes) avoid that, at the cost of a lookup for every reference. For a scene of tens of objects, indices and one careful function are the simpler choice, and the tests check every case.

---

## Part 3: Undo and the file

### 3.1 Snapshots, committed when an edit ends
`SceneHistory` (`src/edit/scene_history.hpp`) keeps whole scenes, like `ladder_editor` keeps whole rungs: a scene is a few hundred bytes. The panels edit the current scene in place, every frame while a slider moves. The app calls `commit()` only when **no widget is active** (`ImGui::IsAnyItemActive()`): the slider was let go, the button was clicked. If the scene changed since the last commit, that's one step. So dragging a length from 3 to 4 m is one undo step, not sixty.

### 3.2 Saved after every change
On every commit the scene is written to its file, through a temporary file and a rename, so a crash never leaves half a file. There's no Save button to forget. If the file can't be read when the app starts (a newer format, a broken edit), it's copied aside as `.broken` before anything overwrites it: losing someone's scene silently is the worst thing a "save automatically" feature can do.

### 3.3 Exactly the same after a round trip
A test saves a scene to JSON, reads it back and compares it with `==`. That caught a design choice before it became a bug: storing the heading in **degrees** (nicer to read) would mean converting `heading × 180/π` and back, which can change the last bit of a float, and then the loaded scene isn't equal to the saved one. The file stores radians, exactly as the code does.

---

## Part 4: Clicking things in 3D

### 4.1 A ray from the camera
A click is a point on the screen; what's under it is somewhere in 3D. `OrbitCamera::rayThrough` turns the pixel into a **ray**: the pixel's position in normalized device coordinates (−1 to 1), taken back through the inverse of `projection × view`, once on the near plane and once on the far plane. The ray goes from the first point through the second.

### 4.2 Which box does it hit?
Every `DrawBox` now knows what it belongs to (a `Pick`: conveyor 2, photocell 0, …). `pickAt` tests the ray against every selectable box and keeps the nearest hit. The test itself is the **slab method** (`hitDistance`): turn the ray into the box's own axes (where the box is a simple axis-aligned block), and for each axis find where the ray enters and leaves the two faces; the ray is inside the box between the latest entry and the earliest exit, if there is such an interval.

The same list of boxes is used to draw and to pick, so what you see is exactly what you can click.

### 4.3 A click isn't a drag
Left drag turns the camera; a left click selects. ImGui remembers how far the mouse moved while the button was down (`MouseDragMaxDistanceSqr`): under 3 pixels, the release is a click.

---

## Part 5: Measuring before optimising

With three conveyors, the Debug build couldn't keep up with real time: 175 ms per frame. Before changing anything, a small benchmark measured one step without the window: **0.77 ms** per 1 ms step in Debug, **0.03 ms** in Release. So the code was fine; the Debug build was the problem.

Two attempts:
- **Jolt's no-lock interface** (our simulation is single-threaded, so the locks are useless): no measurable change. Reverted: a change that doesn't earn its place shouldn't stay, however reasonable it sounds.
- **Compiling Jolt optimised even in Debug** (`target_compile_options(Jolt PRIVATE $<$<CONFIG:Debug>:-O2>)`): **0.10 ms** per step, 7.6 times faster. Our own code keeps its debug information and sanitizers; the physics library, which we don't debug, runs at full speed.

The lesson is the order: measure, find where the time goes, change one thing, measure again, keep only what helped.

---

## Part 6: Testing

- `sim/scene_test.cpp` (10 cases): straight and turned placement; a moved first conveyor carrying the chain; removing a conveyor (followers keep their place, indices shift, what was on it goes); first free names; live versus rebuild; a live change keeping boxes, time and triggers; an exact JSON round trip; old files with missing fields and broken files with clear errors; saving through a temporary file.
- `edit/scene_history_test.cpp` (3 cases): a slider drag as one step; undo of an edit not yet committed; a new change forgetting what was undone.
- `render/scene_test.cpp` (2 new cases): the ray-box test (turned boxes included); picking the nearest selectable thing and highlighting its parts.
- `render/orbit_camera_test.cpp` (1 new case): the ray through the window's centre points at the target.
- By eye: a three-conveyor line loaded from a file, the spawner's gate, the chain in the Line panel.

---

## Review questions

**1.** You change a photocell's response time while boxes run. Does the run restart? And if you make the belt 20 cm longer?
<details><summary>Answer</summary>

The response time doesn't restart anything: it's a setting, applied to the running world live (`applyLive`), and the photocell keeps its triggers. A longer belt changes the layout, so `sameLayout` is false and a new world is built: the run restarts with no boxes.
</details>

**2.** C2 follows C1. Where is C2's start stored, and what happens to it when C1 gets longer?
<details><summary>Answer</summary>

It isn't stored: `placeConveyors` works it out from C1's end every time. When C1 gets longer, C2's start moves with C1's end, with no extra code.
</details>

**3.** Why may a conveyor only follow one that comes earlier in the list?
<details><summary>Answer</summary>

So a single pass from first to last can place every conveyor (its predecessor is always placed already), and so loops (A follows B, B follows A) are impossible by construction.
</details>

**4.** You drag the length slider from 3 m to 5 m over two seconds. How many undo steps is that, and what makes it so?
<details><summary>Answer</summary>

One. The scene changes every frame while the slider is held, but `commit()` only runs when no widget is active, after the slider is let go, and then the whole change is one step.
</details>
