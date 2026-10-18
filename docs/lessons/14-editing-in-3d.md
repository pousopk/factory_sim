# Lesson 14: Editing in 3D

Until now, a line was built from the panels: buttons to add things, sliders to move them. Milestone 14 lets you build it in the view:
- **drag** the selection (a device along its belt, a conveyor across the floor);
- **place** a new device by clicking on a belt;
- **copy and paste** anything.

```
 select ─► drag it ──────────► let go
           (a preview of the    (one undo step;
            new layout; the      the world follows)
            run waits)

 Line panel: [Laser] ─► click on a belt ─► a laser there, wired to free addresses
 Ctrl+C, Ctrl+V ─► a copy with a new name and its own addresses
```

---

## Part 1: Constrained dragging instead of a gizmo

### 1.1 The choice
The plan said "drag handles (ImGuizmo)": a gizmo with arrows for the three axes and rings for rotating, as in CAD and game editors. Looking at the scene changed that. Almost everything in a line sits **on a conveyor, at a place along it**: a photocell is "1.5 m along C1", not a point in space. A free 3D gizmo would mostly let you pull devices off their belts, and then the model would have to put them back.

So the drags follow the model:
- a **device** slides **along its conveyor**, and that's the only thing it can do;
- a **conveyor** standing on its own slides **across the floor** at its height. **R** turns it by 90°.

Conveyors that follow or join another aren't draggable: their place comes from the one they're attached to, so dragging *that* one moves them along. No new library was needed, and every drag produces a value the scene already understood.

### 1.2 From the mouse to "along"
The mouse gives a **ray** from the camera through the pixel (`OrbitCamera::rayThrough`, from lesson 1). Two things are worked out from it, in `edit/placing.hpp` and tested without a window:
- **`onPlane(ray, height)`**: where the ray meets a horizontal level. For a conveyor, that's its belt height. Nothing if the ray runs level or looks up.
- **`alongNearest(conveyor, ray)`**: where the mouse "is" along a conveyor, even off the belt. It's the ray's point at belt level, projected onto the belt's centre line.

My first version used the textbook "closest point between two lines" (the ray and the centre line). The test caught it: from an oblique view it was 24 cm off the point under the mouse, because the closest point of the two lines isn't where the eye sees the mouse on the belt. The closest-line method is still used as a fallback, for a camera almost level with the belt, where the belt-level point runs off to infinity.

### 1.3 Grabbing
A drag remembers the **grab offset**: where along the belt the mouse was when the button went down, less where the device was. Without it, the device would jump so its centre sits under the mouse, wherever on it you clicked. Positions **snap** to 1 cm along a belt and 5 cm on the floor, so lines come out tidy. With **Shift** they snap to 1 mm.

### 1.4 When a drag starts
Left-dragging already turned the camera, so a drag only starts on the **selected** thing: a press whose ray picks the selection. A press anywhere else still turns the camera, so you click once to select, then drag.

---

## Part 2: A drag is one edit

### 2.1 The problem
A drag changes the scene every frame for as long as it lasts. Without care, that means:
- **dozens of undo steps** and saves, since the scene is committed whenever no ImGui widget is being used, and a drag in the 3D view isn't an ImGui widget;
- **a new world every frame** for anything that changes the layout, like a moved pusher or conveyor. The run would restart sixty times a second.

### 2.2 Edit now, follow later
While a drag lasts, the app neither commits the scene nor lets the world follow it. It draws a **preview** instead: a world built from the scene as it is now (`boxesToDraw`), with no program, which never runs. Its parts are drawn with the running world's boxes on top. The preview is rebuilt only when the scene has changed since the last frame.

When you let go, the next frame commits the change (**one undo step**) and the world follows, once. A test drove a drag from code: halfway through, the scene's photocell was at 2.03 m and the running world's still at 1.50 m. After letting go, both were at 2.53 m, with one undo step, and undo took it back to 1.50 m.

---

## Part 3: Placing by clicking

The Line panel's device buttons used to add a device "halfway along the selected conveyor", which you then moved. Now a button **arms** a placing mode (it turns blue): the belt under the mouse lights up, a tooltip says where you are ("1.25 m along C1"), and a click puts the device there. Esc cancels.

`beltUnder(conveyors, ray)` finds the belt: for each conveyor, it checks where the ray meets its belt's level, whether that point is on the belt (within its length and width), and keeps the nearest. `addAt(scene, kind, conveyor, along)` adds the device, wired to free addresses like any new device, then moves it to the click.

---

## Part 4: Copy and paste

### 4.1 One clip for anything
**Ctrl+C** keeps a copy of the selection in a `std::variant` of all the kinds of things in a scene: a conveyor, a photocell, a pusher, and so on. **Ctrl+V** adds it again, on the belt under the mouse if there is one, otherwise next to the original. **Ctrl+D** does both at once.

### 4.2 A copy must not be wired to the same I/O
A pasted pusher with the same valve output as the original would make one PLC output drive two valves, and two "extended" sensors would fight over one input. So a paste:
- gives the copy the **next free name** of its kind (P1 → P2);
- moves **each of its addresses** to a free address of the same kind (`rewire`: same area, same size).

A pasted conveyor gets new addresses for its encoder, run output, zones, carriage and drive, and goes beside the original, on its own.

### 4.3 `std::visit` with `if constexpr`
`paste` has one branch per kind, chosen at compile time:

```cpp
std::visit([&](const auto& thing) -> Pick {
    using T = std::decay_t<decltype(thing)>;
    if constexpr (std::is_same_v<T, PlacedConveyor>) { ... }
    else if constexpr (std::is_same_v<T, PusherSpec>) {
        return device(thing, scene.pushers, Pick::Kind::Pusher,
                      &PusherSpec::valveOutput, &PusherSpec::extendedInput, &PusherSpec::retractedInput);
    }
    ...
    else { static_assert(std::is_same_v<T, LightCurtainSpec>, "a kind of clip without a paste"); ... }
}, clip);
```

Two techniques here:
- **Pointers to members:** `device` takes them as a **parameter pack** (`auto... fields`), and rewires each one with a fold expression, `(rewire(scene, list[index].*fields), ...)`. One helper serves every device, whatever its addresses are called.
- **The final `static_assert`:** if a new kind is added to the variant without a paste branch, it would fall into the last branch and fail to compile there anyway, with an error about light curtains deep inside it. The assert makes the compiler say what's actually wrong: "a kind of clip without a paste".

---

## Part 5: Testing

- **`edit/placing_test.cpp`** (6 cases):
  - the belt under a ray and where along it, and "along" off the belt;
  - a ray meeting a level, and snapping;
  - moving a device along its belt and stopping at its end, including a spawner's offset;
  - placing by click, wired to free addresses;
  - copy and paste of a pusher: new name, new addresses, 0.2 m further on, or at a given place;
  - copy and paste of a conveyor with zones: beside it, re-addressed.
- **By driving the app:** a hook fed `editInView` computed rays for a few dozen frames: a press on the selected photocell, a 1 m drag, a release. Before letting go the world hadn't followed; after, it had, with one undo step; undo restored it; paste added a copy on the next free input. (Real mouse events can't be sent to the window here, so the drag was driven from the code, in the same frames a mouse would use.)

---

## Review questions

**1.** Why does dragging a photocell move it along its conveyor instead of freely in 3D?
<details><summary>Answer</summary>

In the model a photocell is a place along a conveyor (its `along`), not a point in space. A free drag would move it somewhere that isn't on its belt, which the model has no way to say. Constraining the drag to the belt gives only positions the scene can hold.
</details>

**2.** Why does the app stop committing the scene while a drag is going on?
<details><summary>Answer</summary>

The scene changes every frame of the drag. Committing each change would make dozens of undo steps and saves, and for a layout change (a moved pusher or conveyor) a new world every frame. The drag is shown as a preview, and letting go makes it one undo step, with one new world.
</details>

**3.** You paste a copy of a pusher. Why doesn't it keep the original's valve output?
<details><summary>Answer</summary>

Then one PLC output would drive both valves, and both pushers' "extended" sensors would share one input. A copy needs its own I/O, so paste moves each address to a free one of the same kind.
</details>

**4.** What does the `static_assert` at the end of `paste`'s `if constexpr` chain protect against?
<details><summary>Answer</summary>

A new kind added to the clip's variant without a paste branch. That kind would fall into the last `else`, the light curtain's, and fail to compile there with a confusing error about light curtains. The `static_assert` makes the error say what's really missing: a paste for that kind.
</details>
