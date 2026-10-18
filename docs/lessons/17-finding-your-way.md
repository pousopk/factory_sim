# Lesson 17: Finding your way

A line with forty devices was hard to read: the Line panel listed photocells, then pushers, then lasers, each kind in its own flat list, and the only way to learn what something in the 3D view was, was to click it and read the inspector. Milestone 17 makes the line easy to read in two ways:
- the **Line panel is a tree**, arranged the way the line is;
- the **3D view answers "what is that?"** when you hover over something.

It also makes **items** (boxes and tins) selectable.

```
 [filter by name            ]
 ● ▼ C1  6.00 m
     spawner  0.20 m        every 1.00 s
   ● BQ       inspection 1.50 m   %IX0.0
   ● P1       pusher 3.50 m       %QX0.0
     Bin1     bin 3.50 m          1 items
```

---

## Part 1: The tree

### 1.1 Arranged like the line
An engineer thinks of a line conveyor by conveyor: "the photo-eye at the end of C3". So the tree has one node per conveyor, and under it:
- what it is attached to: what it follows, or what it joins and where;
- its zones and its turntable or lift, if it has them;
- every device on it, by kind, each with its place along the belt.

Each row has the same columns: a **state dot** (lit when the input or output is on, or the belt runs), the **name**, the **kind and place**, and a **detail**, usually the address. The kind column starts just past the widest name under that conveyor, so it lines up whatever the names are.

### 1.2 The filter
The box at the top keeps the rows whose name contains the text, ignoring case: "P" gives P1, P2 and the like; a conveyor's name ("C3") keeps that conveyor with everything on it. A conveyor stays in the tree while any of its rows match. With 40 devices, typing three letters beats scrolling.

### 1.3 One selection
Clicking a row sets the same `selected_` as clicking in the view, so the inspector, the 3D highlight and the tree all agree. Hovering a row shows the same tooltip as hovering the device in the view (part 2).

---

## Part 2: Hovering in the view

### 2.1 Picking every frame
Picking (lesson 3) casts a ray from the camera through the mouse and finds the first box it hits. It used to run only on a click. Now `handleMouse` runs it **every frame** and keeps the result in `hoverPick_`. That costs one ray against a few hundred boxes, a few microseconds.

It skips the ray when it would mean nothing:
- when ImGui has the mouse (over a panel), `hoverPick_` is cleared;
- while placing or dragging, the drag owns the mouse.

A click now just selects `hoverPick_`, so what lights up is exactly what a click would select.

### 2.2 The tint
`DrawBox` gets a `hovered` flag, and the renderer passes it to the shader as one more bit of the instance's flags (8, next to *unlit* 1, *selected* 2 and *round* 4). The fragment shader lightens a hovered box toward white by 22 %:

```glsl
if (hovered) color = mix(color, vec3(1.0), 0.22);
```

That's enough to see, but different from the selection's blue tint, so "under the mouse" and "selected" don't get mixed up. It's a flag on each instance, not a separate draw, so it costs nothing.

### 2.3 The tooltip
`describe(pick)` writes one short text per kind of thing: what it is, where it is, its state, and **its I/O as the PLC sees it now**:

```
C1: conveyor, 6.00 m, 0.50 m/s (set 0.50)
encoder %ID8 = 30771
```

Bits print as `= 0` or `= 1`, words in hex the way TIA's watch tables show them (`16#047F`) plus their signed value, double words in decimal. An address that doesn't parse says so. This is the question a commissioning engineer asks most often: "is that sensor's input actually on?" The answer appears under the mouse, without opening the I/O window and looking up the address.

The tooltip only shows while no mouse button is down, so it doesn't get in the way of turning the view or dragging.

---

## Part 3: Items you can select

Items aren't part of the scene: spawners make them and the physics moves them, so they aren't saved, undone or copied. But you often want to know about **one** box ("why didn't that one get pushed?") or take one off the line by hand, the way an operator clears a jam.

- `Pick::Kind::Item` uses the item's **id** as its index, not its place in a list: items come and go, and the id stays the same while the item lives.
- `keepSelectionValid` drops the selection when the item is gone (into a bin, or off the end of the line).
- The inspector shows its shape, size, position and speed, and **Remove it** (or **Del**) calls `World::removeItem`.
- Items can't be dragged, copied or placed: the editing code's switches name `Item` and do nothing.

---

## Part 4: Testing

The new code is all UI: what's drawn and when. `describe()` reads the scene and the I/O image, which are already tested. By eye, with a screenshot of the sorting demo:
- the tree with C1's devices, P1 highlighted when it's selected, the kind column aligned (the first try had a fixed column, and "spawner" ran into its place);
- the tooltip over C1 with its encoder's double word;
- the hovered belt lit a little.

Both builds, both suites, all passing.

---

## Review questions

**1.** Why does the tooltip show the I/O values, not only the address?
<details><summary>Answer</summary>

Because that's what you check during commissioning: is that sensor's input on now, is the PLC setting that valve's output. With the value under the mouse, you see it while you watch the device, without looking the address up in the I/O window.
</details>

**2.** Why does a selected item use the item's id and not its place in the list of items?
<details><summary>Answer</summary>

Items are added and removed all the time, so the list's order changes and place 3 is a different item a moment later. The id belongs to one item for its whole life, so the selection stays on that box, and is dropped when it's gone.
</details>

**3.** Why is the hover a lighter colour and not the same blue tint as the selection?
<details><summary>Answer</summary>

They mean different things: "a click would select this" and "this is selected". If they looked the same, moving the mouse over a device would look like selecting it, and you couldn't tell which one the inspector shows.
</details>
