# Lesson 21: Placing anywhere

Until now every device belonged to a conveyor. It sat at a place along the belt, on its left or right, in the one way that kind of device is mounted. On a real machine things are bolted where they fit: a photocell on a bracket set back from the belt, a bin under a chute, a camera over two lines.

Milestone 21 gives every device a **placement** and adds **3D handles** (a gizmo) to move and turn anything in the view. It's built so that geometry you construct later (frames, plates, chutes) can be placed the same way.

Try it: select a pusher, drag its arrows (W) or rings (E), then press *Take it off* in the inspector.

---

## Part 1: One placement for every device

### 1.1 Parent, offset, turn
A placement has a **parent**:
- **a conveyor:** the device is at its place along the belt (its `along`), mounted as that kind of device usually is, and moves when the conveyor moves. An **offset and a turn** in the belt's frame there move it from that usual place: 5 cm up, set back, turned 20 degrees. Without them, it's **as usual**.
- **the world:** it stands on its own, and its offset and turn say where.

The **belt's frame** at a place along it has its origin on the belt's centre line at its surface, x along the travel, y up, z to the right. Everything about a device on a belt is measured from there.

### 1.2 The device's shape stays as it was
The key decision is that nothing about a device's own shape changed:
- a pusher's cylinder, rod and paddle;
- a bin's floor and walls;
- a camera's body and field of view.

Each is still worked out **as if the device were where it usually is**, by the same functions as before. The placement then adds one **transform** that moves the whole result.

For a device on a conveyor, moved from its usual place, the transform is

    T = B · N · B⁻¹

where B is the belt's frame at the device's place, and N its offset and turn. Read from the right:
1. B⁻¹ takes a point from the world into the belt's frame.
2. N moves it there.
3. B takes it back to the world.

So "5 cm up" means up from the belt, and "turned" means about the device's own place, whichever way the belt runs.

A device placed as usual has no transform at all: `Mounting::identity` is true and nothing is multiplied. So every scene from before gives **bit for bit** the same results. All 143 earlier tests, the tin study's 1.30 mm and the demos, are unchanged.

### 1.3 Standing on its own: a stand-in belt
A pusher's stroke reaches across the belt's width, and a bin stands on the floor beside the belt. Without a belt, what does "across" mean? Such a device is worked out on a **stand-in belt**:
- as wide as `span` and as high as `height`, the belt it was taken off;
- running along +x, with its frame at the origin.

Then the transform moves that frame to where it stands. Taking a device off its belt copies the belt's width and height, so it stays exactly where it was. A bin's floor, worked out on the floor below the stand-in belt, stays on the floor; raising the bin with the gizmo lifts it all.

### 1.4 One function, used everywhere
The geometry functions (`pusherPaddle`, `binWalls`, `nozzlesOf` and the others) now apply the placement themselves. So the physics bodies, the drawing, the air jet's streams and a bin's counting all agree on where a device is, with no call site changed.

The drawing works differently. It draws a whole device as usual (`pusherPaddleOnBelt`, ...) and then moves all its parts at once, so a part isn't moved twice.

---

## Part 2: The gizmo

### 2.1 Our own
The gizmo is our own code, rather than a library: about 200 lines of maths (`edit/gizmo.hpp`, tested without a window) and 200 of drawing and dragging (`app/gizmo_view.cpp`). It's drawn with ImGui's background draw list, over the 3D view and under the panels.

### 2.2 Keeping its size
A handle 0.3 m long is huge close up and invisible from afar. The gizmo is sized each frame to look about 90 pixels long. At distance d, with a vertical field of view φ, a view h pixels high shows 2 d tan(φ/2) metres, so

    size = 90 × 2 d tan(φ/2) / h

### 2.3 Picking a handle
Picking happens on the screen, not in 3D:
1. Each handle is projected to pixels: an arrow's two ends, or a ring's 48 points.
2. The mouse picks the nearest one within 8 pixels of its outline.

Thin lines are easy to hit this way, whatever their depth.

### 2.4 Moving along an arrow
The mouse ray and the arrow's line are two lines in space. Where the ray passes nearest the arrow is a point on it, at

    t = (b e − c d) / (a c − b²)
    a = A·A, b = A·D, c = D·D, d = A·w, e = D·w, w = origin − ray start

for the arrow's direction A and the ray's direction D. The move is how far t has gone since the drag began, in whole centimetres (millimetres with Shift).

When you look straight along the arrow, a c − b² goes to zero: every point of it is under the mouse at once. The drag then waits rather than jumping.

### 2.5 Turning on a ring
The mouse ray meets the ring's plane at a point. The turn is the angle, about the ring's axis, from where it met the plane when the drag began:

    angle = atan2(axis · (from × to), from · to)

That's the signed angle between the two vectors, positive by the right-hand rule. It's snapped to 15 degrees (1 with Shift). Seen edge-on, where the ray meets the plane jumps about wildly, so the drag waits for a better angle.

### 2.6 From the gizmo's frame back to a placement
A drag gives a new frame for the selection. `placeAt` turns that frame back into a placement:
- **on a belt:** the part of the move **along the belt** goes into `along`, so the device slides along its belt as before and stays exact for sensing. The rest becomes its offset and turn.
- **standing on its own:** the frame is its placement.
- **a conveyor:** it moves across the floor, rises (its belt height), and turns only about the vertical, about its middle.

Each frame of the drag starts again from the scene **as it was when the drag began**. A thousand small moves would otherwise pile up a thousand roundings.

Rounding also leaves debris: a move up and back down again can leave 0.00002 mm of offset. `placementAt` treats anything under 0.05 mm, or a hundredth of a degree, as no nudge, so the device is as usual again and senses exactly.

As with every drag, the view shows a preview, and letting go is one undo step.

---

## Part 3: What works where

| Device | On its belt as usual | Moved, or standing on its own |
|---|---|---|
| pushers, diverters, stoppers | yes | yes: their bodies are where they're drawn |
| air jets | yes | yes: their streams leave the nozzles where they are |
| bins | yes | yes: they count what lands in them |
| spawners | yes | yes: items drop where they stand (still, if not on a belt) |
| cameras | yes | their picture shows what they look at; their triggers and line images are milestone 22 |
| photocells, proximity switches, lasers, light curtains | yes | drawn where placed, **not sensing yet** (milestone 22) |

The sensors are the hard part. Their maths is in belt terms ("the beam crosses the belt here, at this height"), which is what makes their switching times exact to a fraction of a millisecond. Milestone 22 gives them a version that works in their own frame.

---

## Part 4: C++

### 4.1 Breaking a header cycle
`PhotocellSpec`, in `sensors.hpp`, needs a `Placement`, and `conveyor_spec.hpp` includes `sensors.hpp`. If the placement header needed `ConveyorSpec`, the headers would include each other. So the data (`Placement`, `Frame`) is in `placement.hpp`, which needs only glm. Everything that needs a belt (`mounting`, `beltFrame`, `placementAt`) is in `mounting.hpp`.

### 4.2 One visitor for eleven kinds
Eleven device kinds, one placement each. Writing attach, detach, placeAt and the inspector eleven times would be eleven chances for a mistake. Instead `withDevice(scene, pick, f)` finds the device a pick names and calls `f(spec, along)` with it. `f` is a generic lambda (`[&](auto& spec, float& along)`), compiled once per kind by the compiler. The same visitor works on a const scene for reading and a mutable one for writing, because it's a template over the scene's type.

### 4.3 A reference to a temporary
A test first crashed with `std::bad_alloc`:

    const auto& belt = placeConveyors(scene)[0];

`placeConveyors` returns a vector by value. That temporary dies at the end of the statement, and `belt` refers into freed memory. GCC warned (`-Wdangling-reference`); the fix is a copy (`const auto belt = ...`).

Binding a `const&` to a temporary extends the temporary's life only when the reference binds to the temporary itself, not to something inside it.

---

## Part 5: Testing

**`edit/gizmo_test.cpp`** (7 cases):
- an arrow's drag is where the mouse ray passes nearest it, and there's no answer looking along it;
- a ring's drag is the signed angle on its plane, and there's none edge-on;
- the mouse on each arrow picks it, nothing far off is picked, and the gizmo grows with distance to keep its size on the screen;
- a pusher moved along its belt slides along it and stays as usual; moved up, it's nudged; moved back, as usual again; its paddle follows;
- detaching leaves a pusher's paddle and a bin's inside exactly where they were; a bin moved 2 m stays on the floor; attaching puts it back near where it was;
- a conveyor on its own moves, rises and turns a quarter about its middle;
- placements survive JSON; removing a conveyor takes what's on it but not what stands on its own; a spawner on its own drops items where it stands.

**Everything before:** all 143 earlier tests unchanged, in both builds.

**By eye:**
- a pusher nudged up and turned on its belt, with the arrows on it along its own turned axes;
- a bin standing on its own, moved and turned, with its rings;
- a drag about x snapped in 15 degree steps.

---

## Review questions

**1.** Why is a device's shape worked out where it would usually be, then moved, instead of each kind of device knowing how to be placed anywhere?
<details><summary>Answer</summary>

So there's one placement mechanism for every kind, and for whatever geometry comes later. The existing, tested shape code stays as it was. A device placed as usual isn't moved at all, so old results are exactly the same.
</details>

**2.** You drag a pusher 30 cm along its belt, then 5 cm up. What changes in its placement?
<details><summary>Answer</summary>

The 30 cm along the belt goes into its `along` (it slides along its belt as before). The 5 cm up becomes its offset. It's no longer as usual, so it's drawn and pushes 5 cm higher.
</details>

**3.** Why does a drag start again from the scene as it was when the drag began, every frame?
<details><summary>Answer</summary>

Each frame's move would otherwise be added to the last frame's result, rounding included. Over a long drag the roundings pile up, and a device dragged out and back wouldn't land exactly where it began.
</details>

**4.** What goes wrong looking straight along an arrow while dragging it, and what does the gizmo do?
<details><summary>Answer</summary>

The mouse ray is parallel to the arrow, so every point of the arrow is equally near it: the closest point is undefined (the denominator a c − b² is zero). The gizmo ignores the mouse until the view isn't along the arrow any more.
</details>
