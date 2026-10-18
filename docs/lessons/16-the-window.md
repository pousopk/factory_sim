# Lesson 16: A better window

After fifteen milestones of features, factory_sim had about ten floating windows at fixed places. They covered the 3D view and each other, and a "main panel" mixed the simulation's controls, the control program, the file and a help text. Milestone 16 changes nothing in the simulation. It reorganises the window:
- **docked panels** around the 3D view;
- a **toolbar** and a **status bar**;
- a proper **font and theme**.

```
 File  Edit  View  Help                                  untitled (saved in the config folder)
 [Pause] [Step 1 ms] [Step 100 ms] [Restart] [Clear items] [time scale 1.00x]  13.682 s  13 items
┌ Control | Line ─────┬──────────────────────────────────────┬ Triggers | I/O | ... ┐
│ program, sorters…   │                                      │                     │
├ Inspector ──────────┤            the 3D view               │                     │
│ the selection       │                                      │                     │
│                     ├ Operator panel | Logic analyser | Camera view ─────────────┤
│                     │ timelines, ...                                             │
└─────────────────────┴────────────────────────────────────────────────────────────┘
 running │ 1.00x real time │ 53 fps, 4 draw calls, GPU 1.3 ms │ untitled
```

---

## Part 1: Docking

### 1.1 A layout people know
Engineering tools (TIA Portal, Visual Studio, CAD programs) share one layout: the work in the middle, tool panels docked around it as tabs, which you can drag elsewhere, stack, float or close. Dear ImGui does this in its **docking branch**, the same version plus docking. The build now fetches `v1.92.6-docking` instead of `v1.92.6`, and `ImGuiConfigFlags_DockingEnable` turns it on.

### 1.2 The dockspace and its middle
`DockSpaceOverViewport` makes the whole window, between the toolbar and the status bar, a dockspace. Its **central node** is the part no panel covers. With `ImGuiDockNodeFlags_PassthruCentralNode` that middle is see-through: ImGui draws nothing there and leaves the mouse to the program. That's where the 3D view goes. Each frame the app reads the central node's rectangle (`viewMin_`, `viewSize_`) and uses it everywhere the view is concerned:
- the renderer draws into it (`glViewport` with that rectangle, its y turned upside down, since OpenGL counts from the bottom);
- the camera's aspect ratio is its shape;
- the mouse ray is worked out relative to it, so clicking, dragging and placing hit what's under the mouse;
- the device names are placed in it, and clipped to it.

### 1.3 A default layout
On first start (no saved layout) or after **View > Reset layout**, the **dock builder** (`DockBuilderSplitNode`, `DockBuilderDockWindow`, from `imgui_internal.h`) splits the dockspace:
- a column on the left for **building**: Control and Line as tabs, the Inspector under them;
- one on the right for **watching**: I/O, Triggers, PLC variables, Cameras;
- a strip under the view for **time**: the Logic analyser, Camera view and Operator panel.

Windows are placed by name, so a panel shown later (Camera view, when a scene has cameras) lands in its slot. After that, whatever you rearrange is saved in `layout.ini` with the rest of ImGui's settings (lesson 9).

---

## Part 2: Toolbar and status bar

The old main panel held three different things, and each went to where it belongs:
- **Running time** went to a **toolbar** under the menu: Run/Pause (also **Space**), the steps, Restart, Clear items, the time scale, the simulated time and the item count.
- **The control program** (which program, its sorters or triggers, soft_plc's status) got its own docked panel, **Control**.
- **How things are going** went to a **status bar** at the bottom: running or paused, the real-time factor (or *behind*, in red), frames per second, draw calls and GPU time, the file's state, and a control program's error.

Both bars are ImGui's **viewport side bars** (`BeginViewportSideBar`): windows fixed to an edge of the window that shrink the work area, so the dockspace fits between them.

The operator's buttons and lamps were in a window called "Control panel", easy to mix up with the new Control panel, so they're now the **Operator panel**: what an operator touches, as on a machine.

The **real-time factor**, simulated seconds per real second, is new in the status bar. It's smoothed over about ten frames (`0.9 · old + 0.1 · new`), so it reads steadily instead of flickering with each frame's step count.

**Help > Mouse and keys** (F1) lists every mouse action and key in one table, instead of a help text squeezed into a panel.

---

## Part 3: Font and theme

### 3.1 A font built into the program
ImGui's default font is a small bitmap font, readable but rough. Roboto comes with Dear ImGui (in `misc/fonts`, under the Apache 2.0 licence). CMake reads the file at configure time and writes its bytes into a generated C++ source:

```cmake
file(READ ${FACTORY_SIM_FONT} font_hex HEX)
string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," font_bytes "${font_hex}")
file(WRITE ${FACTORY_SIM_FONT_SOURCE} "... const unsigned char kUiFont[] = {${font_bytes}}; ...")
```

So the program carries its font, and there's no file to find at run time, wherever it's started from. ImGui loads it from memory, told not to free bytes that aren't its own (`FontDataOwnedByAtlas = false`). It's only regenerated when the font file is newer than the source.

### 3.2 Sizes that follow the screen and the font
- **HiDPI:** GLFW reports the screen's content scale (2 on a screen set to 200 %), and the theme scales the font (`FontScaleDpi`) and every spacing (`ScaleAllSizes`) by it.
- **Field widths** were a fixed 180 pixels, which cut labels in narrow panels and wasted room in wide ones. They're now `fieldWidth()`: 55 % of the panel's width, between 7 and 16 times the font size.
- **Table columns and small fields** were fixed pixel widths. They're now multiples of the font size, so a bigger font or a HiDPI screen doesn't cut "16#047F" in half.

### 3.3 One accent colour
The theme is dark blue-grey panels, slightly lighter fields, and **one** accent blue for whatever is active, selected or being dragged: the selected tab's line, slider grabs, check marks, the docking preview. One accent means the eye finds the active thing without reading, and the PLC's own colours in the scene (red bad boxes, yellow lit LEDs) stay distinct from the UI's.

---

## Part 4: Testing

There's nothing new to unit-test: no simulation changed, and the layout is ImGui's. The tests check that nothing else moved: both builds, both suites, all passing, with ImGui now on the docking branch. By eye:
- the default layout on a fresh config (screenshot above): the 3D view in the middle at its own aspect ratio, the names on the belt placed and clipped to it;
- the toolbar and status bar, and the real-time factor at 1.00x;
- the panels' tabs;
- the Roboto font.

---

## Review questions

**1.** Why does the 3D view need the central node's rectangle every frame, and what would go wrong without it?
<details><summary>Answer</summary>

The view isn't the whole window any more: panels are docked around it, and you can resize them. Drawing into the whole window would put the scene under the panels with the wrong aspect ratio, and mouse rays worked out over the whole window would pick and drag the wrong things.
</details>

**2.** Why is the font compiled into the program instead of loaded from a file?
<details><summary>Answer</summary>

So the program finds it wherever it's run from and however it's installed. A file next to the source would only be found from the build tree, and a missing font would mean a broken UI.
</details>

**3.** Why does the status bar smooth the real-time factor?
<details><summary>Answer</summary>

Each frame does a whole number of 1 ms steps, so one frame's factor jumps around (16 steps, then 17). Averaging over about ten frames shows the trend, not the jitter.
</details>
