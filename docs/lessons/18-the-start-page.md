# Lesson 18: A start page

Until now, `./factory_sim` with no file opened the scene used last, or the untitled one. For most people that was one of the demos, so the program started inside someone else's line, with nothing saying what else it could do. **File > New** wasn't empty either: it gave you a conveyor, a spawner and a photocell. Milestone 18 adds what most desktop programs open with: a **start page**.

```
 File  Edit  View  Help
            factory_sim
            Conveyors, sensors and actuators in 3D, controlled by a PLC

            [ New empty line ]      ── Start from a demo ─────────────────────
            [ Open...        ]      ┌ Sorting ───────────┐ ┌ Tins and cameras ──┐
                                    │ A conveyor, a third│ │ Tins at 1 m/s, one │
            ── Recent ───────       └────────────────────┘ └────────────────────┘
            Continue untitled       ┌ Conveyor (soft_plc)┐ ┌ Conveyor with a ...┐
            conveyor_line.json      ...
```

---

## Part 1: When it shows

### 1.1 Always, unless a file is named
- `factory_sim` shows the start page **every time**, even if a scene was open yesterday: the recent list is one click away, and starting in a known place beats starting in whatever was left.
- `factory_sim my_line.json` skips it and opens that file, as before. That's how scripts, file managers and "open with" call a program, and they expect the file, not a choice.
- **File > Close scene** goes back to it. There's nothing to ask first: every change is already saved (lesson 9).

### 1.2 A state, not a window
The start page isn't a panel over the line: until something is chosen there is **no line**. `App` has a `startPage_` flag, and `frame()` checks it first:

```cpp
if (startPage_) {
    drawMenuBar();
    drawStartPage();
    drawHelp();
    drawFileDialog();
    drawNewConfirmation();
    return;
}
```

No mouse handling in the view, no dockspace, toolbar or status bar, no simulation steps, no drawing in 3D, and no autosave. The last one matters most: saving runs when the undo history has a new step, and with no scene there must be nothing to write, or an empty scene could overwrite the untitled one.

Leaving the start page goes through `replaceScene`, the one place every new scene passes through (open, new, demo), which clears the flag. `closeScene` does the reverse: an empty scene, a fresh world (so a soft_plc program stops and frees its monitor port), and the flag set again.

The menu bar stays, so the keys people expect still work (Ctrl+N, Ctrl+O). **Edit** and **View** are greyed out, and so are **Save as** and **Close scene**: there's nothing for them to act on.

---

## Part 2: What it offers

### 2.1 New, Open, Recent
- **New empty line** is really empty now: `Scene{}`, a floor. **File > New** does the same. `defaultScene()` stays, as the fixture about thirty tests build on.
- **Open...** is the same dialog as File > Open.
- **Recent** is the same list as File > Open recent. A file deleted since it was listed is greyed out, with the reason in its tooltip.

### 2.2 Continue untitled
An untitled scene lives only in the scratch file in the config folder (lesson 9). Before this milestone, starting up reopened it; now the start page has to offer it, or the work in it would look lost.

`untitledWork(file)` (in `edit/settings.hpp`) decides: the scene in that file if it can be read **and isn't empty**, nothing otherwise. An untitled scene you created and left empty isn't worth a line on the start page. It's a plain function of a file, so it's tested without a window.

It's read once, when the start page appears (`untitledLeft_`), not every frame: parsing a JSON file sixty times a second to draw one line would be silly.

The same check guards **New** and the demos from the start page: if there's untitled work, "Replace the untitled scene?" asks first, as it does inside a scene. **Save it first...** continues the untitled scene, then opens Save as, so what gets saved is that scene and not the empty one behind the start page.

### 2.3 The demos, in one list
The demos used to be written out once, as menu items with tooltips. The start page needs the same names and descriptions, so they're now one list, `demos()`, in `welcome.cpp`:

```cpp
struct Demo {
    const char* name;
    const char* description;
    Scene (*make)();
};
```

`make` is a plain function pointer: lambdas that capture nothing convert to one. File > New from a demo and the start page both loop over the list, so a new demo shows up in both.

---

## Part 3: Drawing it

### 3.1 A window the size of the work area
`GetMainViewport()->WorkPos` and `WorkSize` are the window minus the menu bar. The start page is one ImGui window placed exactly there, without a title, borders, moving or saved settings, and with `NoBringToFrontOnFocus` so the Help window and the file dialog stay above it.

The content is no wider than 56 times the font size and is centred, so it reads well on a wide screen and still fits a small one.

### 3.2 Tiles
ImGui has no "card" widget, so a tile is built from two simple parts:
- an empty `Selectable` of the tile's size, which handles hovering and clicking (and draws the hover highlight);
- text drawn over it with the window's draw list: the name, then the description wrapped to the tile's width.

All the tiles get the height of the tallest, so the rows line up. Clicks are acted on **after** the window is drawn, since opening a scene in the middle of drawing the page would change what's being drawn.

---

## Part 4: Testing

`untitledWork` has its own test: no file, an empty scene, a scene with something in it, a damaged file. The rest is UI, checked by eye: the start page with recent scenes, Continue untitled and the eight demo tiles.

Both suites, all passing.

---

## Review questions

**1.** Why does a file named on the command line skip the start page?
<details><summary>Answer</summary>

Whoever named it has already chosen: a script, a file manager's "open with", or someone typing the path. Showing a choice they've already made would be in their way.
</details>

**2.** Why must there be no autosave on the start page?
<details><summary>Answer</summary>

Autosave writes the current scene to its file, and an untitled scene's file is the scratch file. Behind the start page the current scene is an empty one, so saving it would overwrite the untitled work that "Continue untitled" offers.
</details>

**3.** Why does "Save it first..." continue the untitled scene before opening Save as, when it's clicked from the start page?
<details><summary>Answer</summary>

Save as saves the current scene. On the start page that's the empty scene, not the untitled one in the scratch file. Continuing it first makes it the current scene, so that's what gets a name.
</details>
