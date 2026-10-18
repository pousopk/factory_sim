# Lesson 9: Files

Until now, factory_sim worked on one file: `factory_sim_scene.json` in the folder it was started from, or the file named on the command line. Milestone 9 gives it what any editor has:
- **File:** New, New from a demo, Open, Open recent, Save as;
- **Edit:** Undo and Redo;
- **View:** show or hide each window.

The window's title says which scene is open, and factory_sim remembers the recent scenes and where you put its windows.

```
File   Edit   View                              /home/me/lines/sorting.json
├ New                 Ctrl+N
├ New from a demo   ▸ Sorting / Conveyor (soft_plc) / Tins and cameras
├ Open...             Ctrl+O
├ Open recent       ▸ sorting.json, conveyor_line.json, ...
├ Save as...          Ctrl+Shift+S
└ Quit
```

---

## Part 1: Where a scene lives

### 1.1 Still saved after every change
Since milestone 3, every change is saved at once. There's no Save button, so a crash never loses work. That stays. What's new is *where* it's saved:
- A **named** scene (opened, or saved with Save as) is saved to its own file.
- An **untitled** scene (File > New, or a demo) is saved to a scratch file, `untitled.json` in the config folder (see 1.2), until **Save as** gives it a name.

`App::savePath()` is the one place that decides, and autosave just calls it.

The untitled scene's only copy is that scratch file, so a new scene would overwrite it. Replacing an *edited* untitled scene therefore asks first: save it, replace it, or cancel. A named scene needs no question: it's safe in its file.

### 1.2 The config folder
Linux programs keep their own files in `$XDG_CONFIG_HOME/<program>`, or `~/.config/<program>` when that variable isn't set (the XDG Base Directory convention). factory_sim keeps three files there:
- `settings.json`: the recent scenes;
- `layout.ini`: where its windows are, written by Dear ImGui itself;
- `untitled.json`: the untitled scene.

`configFolder()` works the path out. Setting `XDG_CONFIG_HOME` moves it: the runs for this lesson's screenshots pointed it at a scratch folder, so they didn't touch the real one.

### 1.3 Starting up
- `factory_sim my_line.json` opens that scene, or starts a new one there if the file doesn't exist.
- `factory_sim` alone opens the most recent scene, or the untitled one.

Opening a file starts a **fresh undo history**: undo shouldn't take you back into another file.

### 1.4 Demos open as new scenes
Before, **Load the sorting demo** replaced the scene in place. It could be undone, but autosave had already written the demo over your file. Now a demo is a new, untitled scene, so your file is never touched.

---

## Part 2: Recent scenes

`addRecent` puts a scene first in the list, once, and drops the oldest past eight. "Once" needs care: `lines/a.json`, `/home/me/lines/a.json` and `/home/me/lines/x/../a.json` are the same file. The list stores every path **absolute and normalised** (`absolute(path).lexically_normal()`), so the same file always compares equal.

Settings must never stop the program from starting:
- A missing file means the defaults.
- A damaged file also means the defaults. It's parsed with `nlohmann::json::parse(text, nullptr, false)`, the form that returns a "discarded" value instead of throwing.
- Recent scenes that no longer exist are left out when the list is read, so the menu never offers a file that's gone.

---

## Part 3: Save as, and relative paths

### 3.1 The problem
A scene that runs soft_plc stores its program's path **relative to the scene's folder**: `conveyor.st`, next to `conveyor_line.json`. Then the folder can be copied or moved as a whole, or cloned from Git, and still work. But **Save as** into another folder breaks that: from `lines/backup/`, `conveyor.st` means a file that isn't there.

`moveScene(scene, from, to)` rewrites the path so it points at the same file from the new folder: `conveyor.st` saved into `sub/` becomes `../conveyor.st`. It uses `lexically_relative`, which works the relative path out from the two paths' text alone, without touching the disk. If there's no relative path (another drive on Windows), it uses the absolute one. Absolute paths stay as they are.

### 3.2 In every undo step
Rewriting only the current scene isn't enough. Undo restores *older* snapshots of the scene, and those still hold the old relative path. Undoing a change to a belt's speed after Save as would bring back a program path that's now wrong.

So `SceneHistory::changeEverywhere(f)` applies a change to the current scene, the committed one, and every snapshot undo or redo could bring back. It isn't an edit, so it adds no undo step. With undo by snapshots (lesson 3), any fact that changes about the *world outside* the scene has to be rewritten in every snapshot.

The app also rewrites `applied_`, the scene the World was built from. Otherwise the next frame would see a different program path, take it for a new program, and restart soft_plc for nothing.

---

## Part 4: The file dialog

### 4.1 Model and view
There's no native file dialog in Dear ImGui, and pulling in a toolkit (GTK, Qt) for one dialog would be a heavy dependency. The dialog is our own, split the usual way:
- `FileBrowser` (`src/edit/file_browser.hpp`) is the model: one folder, its sub-folders and `.json` files (folders first, each alphabetical, hidden ones left out), going in and up, and the file a typed name means (`line` becomes `line.json` in this folder).
- `App::drawFileDialog` draws it as a modal popup.

The model is tested against real temporary folders.

### 4.2 Errors as values
`std::filesystem` functions come in two forms: one throws `filesystem_error`, the other takes a `std::error_code&` and sets it. A folder that can't be listed (deleted meanwhile, no permission) isn't exceptional in a file dialog, it's something to show. So `refresh()` uses the `error_code` forms and keeps the message in `error()` for the dialog to show in red.

### 4.3 Don't change a list while walking it
Two places in the dialog code would invalidate the very loop they're in:
- Clicking a folder in the list *enters* it, which replaces the list of entries.
- Clicking a recent scene *opens* it, which reorders the recent list.

Both note what was clicked and act after the loop (`enter`, `chosen`). Changing a `std::vector` while iterating over it is undefined behaviour, and it may even work by luck until the vector reallocates.

---

## Part 5: Dear ImGui details

### 5.1 The menu bar and popups
`BeginMainMenuBar` draws the bar across the top. The windows' first positions moved down to make room for it.

The dialog and the "replace the untitled scene?" question are **modal popups**: `OpenPopup` names one, `BeginPopupModal` draws it while it's open, and `CloseCurrentPopup` ends it. In immediate mode, the app keeps its own state (`dialog_`, `pendingNew_`) and asks ImGui to open the popup on every frame where that state says so.

### 5.2 A pointer that must outlive the context
`ImGui::GetIO().IniFilename` is a `const char*`. ImGui keeps the pointer, not a copy, and uses it again when the context is destroyed to write the layout. So `main` keeps the `std::string` alive in a variable that outlives the context. A pointer from a temporary like `(folder / "layout.ini").string().c_str()` would dangle at the end of that statement.

### 5.3 Windows that can be closed
`ImGui::Begin(name, &flag)` adds a close button that sets `flag` to false. The View menu's `MenuItem(name, nullptr, &flag)` toggles the same `bool`, so both stay in step with no extra code.

### 5.4 The window's title
The title comes from the app (`App::title()`), and `main` passes it to GLFW whenever it changes.

---

## Part 6: Testing

- **`edit/files_test.cpp`** (6 cases):
  - recent scenes: order, once each, at most eight, the same file written two ways;
  - settings saved and read back, with a deleted scene, a missing file and a damaged file;
  - the browser's listing (folders first, `.json` only, no hidden ones), going in and up, and a folder that doesn't exist;
  - typed names becoming files, and the ones refused;
  - `moveScene` to a sibling folder, a deeper one, and an absolute path left alone;
  - `changeEverywhere` surviving undo, without adding a step.

  A small RAII `TemporaryFolder` makes a fresh folder per test and removes it afterwards.
- **By eye:**
  - the title, menu bar and file name;
  - the Open dialog listing a folder;
  - starting without a scene reopens the last one;
  - Save as into a sub-folder rewrote `conveyor.st` to `../conveyor.st` and put the new file first in the recent list.

---

## Review questions

**1.** Why does an untitled scene still get saved after every change, and where?
<details><summary>Answer</summary>

So a crash or a quit never loses it, as for a named scene. It goes to `untitled.json` in the config folder (`~/.config/factory_sim`) until Save as gives it a name and a place.
</details>

**2.** A scene in `/work/lines` runs `programs/main.st`. You save it as `/work/backup/main_line.json`. What does its program path become, and why rewrite it at all?
<details><summary>Answer</summary>

`../lines/programs/main.st`. The path is relative to the scene's folder: left as it was, from `/work/backup` it would mean `/work/backup/programs/main.st`, which doesn't exist.
</details>

**3.** After that Save as, you undo a speed change you made before it. Why must the program path not come back as `programs/main.st`?
<details><summary>Answer</summary>

The snapshot undo restores was taken while the scene was in `/work/lines`, with the old relative path, which is wrong from `/work/backup`. That's why Save as rewrites the path in every snapshot (`changeEverywhere`), not just in the current scene.
</details>

**4.** In the dialog, clicking a folder enters it, but `enter` is called after the loop over the entries. What could go wrong if it were called inside the loop?
<details><summary>Answer</summary>

`enter` replaces the vector of entries that the loop is walking. The loop's iterators would point into freed memory: undefined behaviour, which can crash or appear to work.
</details>
