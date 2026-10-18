# 6. Design notes

The other topics explain how things work. This one is about why the code is shaped the way it is: the decisions that run through all of factory_sim, what else was possible, and what each choice costs. Several are the same ideas as in soft_plc's and ladder_editor's design notes, met again in a new setting.

---

## 1. The design and the run are different things

**Decision:** the line you build is a `Scene` (a plain value: vectors of specs), and the `World` that runs it is built from it. The panels edit the scene, never the world.

**Why:** they answer different questions and live different lives. The scene is what you save, undo and share; the world is boxes in flight and a clock, thrown away on restart. Mixing them means every feature has to ask "is this part of the design or of the run?" again: undo would have to decide whether to undo box positions, saving would write the time.

**What it costs:** a bridge between them. `followScene()` compares the scene with the one the world was made from, every frame, and either updates the world live or builds a new one (lesson 3). Deciding what can change live (`sameLayout`) is a real design question, the same one soft_plc's online change answers for variables.

---

## 2. Values, compared with `==`

**Decision:** specs and scenes are structs with `bool operator==(...) const = default;`, no pointers, no identity.

**Why:** a value can be copied (undo keeps whole scenes), compared (has anything changed since the last commit? does the world need to follow?), saved and reloaded (and checked equal after the round trip). Three features (undo, change detection, autosave) come from one property. That's the same reason ladder_editor's rungs and projects are values.

**What it costs:** copies. A scene is a few hundred bytes, compared every frame and copied once per undo step: nothing. A model with millions of objects would need structures that share unchanged parts between versions.

---

## 3. Derive instead of storing

Several things are computed when needed instead of stored:

| Derived | From | Stored, it would go stale when… |
|---|---|---|
| a following conveyor's start and heading | the end of the one before it | the one before moves or changes length |
| what to draw (`sceneBoxes`) | the world, every frame | anything moves |
| the encoder count | the belt's travel | (and it would drift: see topic 4) |
| the simulated time | the step count | (it would drift) |
| whether a world can follow live | comparing two scenes' layouts | a field is added and someone forgets to mark it |

**The rule:** store the *facts*, compute the *consequences*. A stored consequence is a second copy of information that must be kept in step with the first, and the one place someone forgets to update it is a bug that waits.

---

## 4. Keep OpenGL thin

**Decision:** everything about **what** to draw is plain C++ with no OpenGL (`render/scene.cpp`: a list of `DrawBox`es; `render/orbit_camera.cpp`: the camera's maths; picking with rays). The OpenGL code (`renderer.cpp`) only draws a list of boxes.

**Why:** OpenGL needs a window and a GPU, which the tests don't have, and its mistakes are silent (a wrong matrix draws nothing, no error). So the code that can be wrong in interesting ways (is the belt's top where the physics thinks it is? does a click on the housing select the photocell? does the ray through the centre hit the target?) is kept where it can be tested, and the untested part is as small and dumb as possible. The same list of boxes is drawn and picked, so what you see is exactly what you can click.

**What it costs:** an intermediate list rebuilt every frame (a few hundred small structs) instead of drawing straight from the world. Cheap, and it makes the renderer replaceable.

ladder_editor does the same with its rungs: the layout is computed and tested as plain data, and the ImGui code only draws it.

---

## 5. Ours and the engine's

**Decision:** Jolt does general physics; factory_sim does everything specific to a factory (topic 5).

**Why:** general physics is hard and well-solved; a specialist library is better at it than anything written here. The factory parts (VFD ramps, response times, encoder counts, exact switching times) are simple, must be exactly right, and are the reason the simulator exists. Keeping them in our code means they're tested directly, documented, and never fight the engine. The engine is reached only through the contact listener (which surface moves), the body interface (where is each box) and `step()`.

---

## 6. Indices, not IDs

**Decision:** photocells and spawners refer to their conveyor by **index** in the scene's list, and a conveyor to the one it follows the same way.

**Why:** simple, compact in the file, and fast (no lookup).

**What it costs:** every deletion must shift the indices after it, everywhere they're used (`removeConveyor`), and a stale index is a bug. Stable IDs (a number that never changes, like the boxes' `ItemId`) would avoid that, at the cost of a lookup for every reference and an ID counter to keep. With tens of objects and one careful, well-tested function that deletes, indices win. With many kinds of references, or references from outside the scene, IDs would.

The boxes, by contrast, *do* use IDs (`ItemId`): they come and go all the time, and triggers refer to them after they may have fallen off and been cleared.

---

## 7. Saving that can't lose work

Three small decisions make "saved after every change" safe:
- **write to a temporary file, then rename it** over the old one: a rename is atomic on Linux, so a crash leaves either the old file or the new one, never half of each;
- **an unreadable file is copied aside** (`.broken`) before anything overwrites it: a newer format or a hand edit gone wrong mustn't be silently replaced by a default scene;
- **missing fields get defaults** when loading, so files saved by older versions keep loading as fields are added.

---

## 8. Measure, then optimise

With three conveyors, the Debug build fell behind real time. The tempting fix was to guess (the locking in Jolt's body interface looked expensive). Instead:

1. **measure in isolation**: a small program timing one step without the window: 0.77 ms in Debug, 0.03 ms in Release;
2. **try one change, measure again**: the no-lock interface changed nothing, so it was **reverted**;
3. **find the real cause**: unoptimised third-party code; compiling Jolt with `-O2` in Debug gave 0.10 ms.

The reverted change is the important part. A change that sounds right but measurably does nothing is still a change: more code to read, a claim in a comment that isn't true. It goes.

---

## Review questions

**1.** Why doesn't undo restore the boxes' positions?
<details><summary>Answer</summary>

Undo works on the scene (the design), not the world (the run). Box positions are part of the run; the world follows the restored scene, live or rebuilt.
</details>

**2.** A following conveyor's start isn't stored. Name one bug that storing it would make possible.
<details><summary>Answer</summary>

Lengthening or moving the conveyor before it without updating the stored start: the chain would come apart, with a gap or an overlap where they should meet.
</details>

**3.** Why is `sceneBoxes` tested but `Renderer::draw` isn't?
<details><summary>Answer</summary>

`sceneBoxes` is plain geometry with no OpenGL, so tests can check every part's position and what a click picks. `draw` needs a window and GPU, and its mistakes are silent; keeping it tiny (draw a list of boxes) leaves little to go wrong there.
</details>

**4.** Photocells refer to conveyors by index, but triggers refer to boxes by ID. Why the difference?
<details><summary>Answer</summary>

Conveyors change rarely, and only through one tested function that fixes indices on deletion. Boxes appear and disappear constantly, and a trigger must still refer to its box after others have come and gone; a stable ID survives that, an index wouldn't.
</details>
