# Porting Brain Screen from 2145Z-EzTemp (PROS + LVGL) to RW-Template-Test (VexCode)

This guide walks through the steps to get your custom brain screen (auton selector, alliance/color indicators, console, etc.) working in **RW-Template-Test**, where VexCode does **not** include LVGL.

---

## What the current screen does (2145Z-EzTemp)

| Feature | Implementation |
|--------|-----------------|
| **Auton selector** | LVGL list (`lv_list`), clickable items, SD card save/load |
| **Alliance / color** | Clickable color indicators (Red / Blue / None), affects odom angle |
| **Path viewer** | Field image + robot icon, animated path from `autonPath` / `injectPath` |
| **Angle check** | LVGL message box, live heading vs target |
| **Console** | Scrollable log (structured lines + unstructured), toggle with logo tap |
| **Assets** | `pics/`: logo, matchField, skillsField, pfp2145, colorindOverlay, red/blue alliance (LVGL C arrays) |

Dependencies: **liblvgl** (PROS port), **PROS** (`pros::usd`, `pros::delay`, `pros::Task`), **EZ-Template** (chassis odom, `auton_sel`), **drive.hpp** (Coordinate, injectPath, etc.).

---

## Two ways to port

### Option A: Port LVGL to VexCode (hard)

- LVGL needs a **display driver** (flush callback to draw to the V5 screen) and an **input driver** (touch → LVGL events).
- In PROS this is done by the LVGL port (display + indev tied to the brain). That code is in the PROS kernel/liblvgl, not in your repo.
- **Steps:** Get LVGL source, add a VexCode “hal” that uses `Brain.Screen` for drawing and Vex’s `Brain.Screen.pressed()`/`released()`/`x_position()`/`y_position()` for touch, then build and link. This is a non-trivial driver port (buffer format, refresh, touch coordinates).
- **Use this if:** You want to keep the exact same UI code and assets and are willing to maintain a LVGL+VexCode port.

### Option B: Reimplement UI with VexCode Brain.Screen (recommended)

- VexCode’s **Brain.Screen** API supports:
  - Drawing: `clearScreen()`, `setPenColor()`, `setFillColor()`, `drawLine()`, `drawRectangle()`, `print()`, `printAt()`, etc.
  - Touch: `pressed(callback)`, `released(callback)`, `pressing()`, `x_position()`, `y_position()`.
- Reimplement the same **behavior** (auton list, alliance selector, console, optional path/angle views) using these APIs and manual hit-testing by (x,y). No LVGL, no pics in LVGL format; images can be skipped or replaced with simple graphics.
- **Use this if:** You want something that works in RW-Template-Test with minimal ongoing maintenance.

The rest of this walkthrough assumes **Option B**.

---

## Step-by-step (Option B: Native Brain.Screen)

### 1. Add screen types and globals (no LVGL)

In **RW-Template-Test** you don’t have `lv_obj_t*`, `lv_color32_t`, or `lv_style_t`. Define your own types and colors.

- **Create** `custom/include/screen.hpp` (or `include/screen.hpp` if you prefer project-level):
  - An **alliance/color enum** (e.g. `Alliances { BLUE, NONE, RED }`).
  - **Theme colors** as `uint32_t` hex or use Vex’s `vex::color` in the .cpp.
  - **AutonSel / AutonObj**: same idea as EzTemp — e.g. `AutonObj` with `void (*callback)()`, `std::string name`, and a color; `AutonSel` with `std::vector<AutonObj> autons`, `void (*selector_callback)()`, `std::string selector_name`, and `selector_populate()`.
  - Declare **`uiInit()`**, **`autoSelectorInit()`**, **`print(int line, const std::string& msg)`** and **`print(const std::string& msg)`** for the console.
  - Declare **`extern AutonSel auton_sel`** and any globals the rest of the code expects (e.g. `allianceColor`, `currentField` if you keep it).

You can mirror the **logic** of 2145Z-EzTemp’s `screen.hpp` without any LVGL or EZ-Template includes.

### 2. Implement screen layout and drawing in VexCode

- **Create** `custom/src/screen.cpp` (or `src/screen.cpp`):
  - **Background:** `Brain.Screen.clearScreen(theme_color)` (or equivalent with `setFillColor` + `drawRectangle` for full screen).
  - **Auton list:** Define a rectangle (x, y, width, height) for the list. In a loop, `printAt()` or `print()` auton names at fixed row positions. Store the bounding box for each row (or a single scroll “window” of visible rows) for hit-testing.
  - **Alliance / color indicator:** Draw one or two rectangles (or circles) at fixed (x,y) with colors for Red/Blue/None. Record their bounding boxes for touch.
  - **Field view (optional):** Without LVGL images, either skip the field image or draw a simple representation (e.g. rectangle for field, small rectangle for robot). Path animation would require your own drawing (e.g. lines between points) and a task/thread that updates position; you can stub this at first.
  - **Console:** Reserve a region; maintain a buffer of strings (e.g. last N lines). When `print()` or `print(line, msg)` is called, update the buffer and redraw that region with `printAt()`.
  - **Logo / “toggle console” button:** Draw a rectangle or text at a fixed area; in the touch callback, if the tap is inside that area, toggle between “auton list” view and “console” view and redraw.

No LVGL calls; only Vex `Brain.Screen.*` and your own state (selected auton index, alliance, console lines, etc.).

### 3. Touch handling (replace LVGL events)

- Use **`Brain.Screen.pressed(callback)`** and/or **`Brain.Screen.released(callback)`**.
  - In the callback, read **`Brain.Screen.x_position()`** and **`Brain.Screen.y_position()`**.
  - **Hit-test:** Check if (x,y) falls inside the auton list rows, alliance box, logo box, scroll up/down areas, etc.
  - Update state (e.g. `auton_sel.selector_callback = ...`, `auton_sel.selector_name = ...`, `allianceColor = ...`) and call your **redraw** function so the screen reflects the new selection.
- If you have “scroll list” behavior, you can use two small regions (up/down arrows) and on press there adjust a “scroll index” and redraw the list.

This replaces all `lv_obj_add_event_cb(..., SelectAuton, ...)` style logic with one or two Brain.Screen callbacks and your own hit-test and state updates.

### 4. Auton list population and default auton

- In **pre_auton** (or your init that runs once), call **`auton_sel.selector_populate({ ... })`** with the same list of autons you use in EzTemp (e.g. `doNothing`, `SAWP13`, `left43`, …), using the **RW-Template-Test** function names/pointers. Set a default, e.g. `auton_sel.selector_callback = right7Odom; auton_sel.selector_name = "Right 7 Odom";`.
- In **autonomous()**, call **`auton_sel.selector_callback();`** instead of a hardcoded switch. So the “brain screen” is the source of truth for which auton runs.

You can keep the same **auton names and order** as in 2145Z-EzTemp for consistency.

### 5. SD card save/load (optional)

- PROS uses **`pros::usd::is_installed()`** and file paths like **`/usd/selected_auton.txt`**. VexCode uses the **V5 filesystem** differently (e.g. `Brain.SDcard` or similar — check Vex C++ API for “SD” or “file”).
- **Steps:** Look up the Vex C++ API for SD card / file read-write. Implement **load_selected_auton_from_sd()** and **save_selected_auton_to_sd()** using that API and the same file name/path convention. If no SD or API is available, you can skip persistence and always start with the default auton.

### 6. Replace PROS/EZ-Template dependencies in screen code

- **Delay:** Replace `pros::delay(10)` with `vex::wait(10, vex::msec)` (or your template’s wait) in any screen task.
- **Tasks:** Replace `pros::Task pathViewer(pathViewerTask);` with **`vex::thread pathViewer(pathViewerTask);`** (or run the path viewer logic in a loop inside a thread). Same for any “angle check” task.
- **Chassis / odom:** The current screen uses **`chassis.odom_x_get()`, `chassis.odom_theta_get()`**, etc. In RW-Template-Test you have **`x_pos`, `y_pos`, `correct_angle`** (and `getInertialHeading()`). So in **screen.cpp**:
  - Replace chassis odom calls with **`x_pos`**, **`y_pos`,** and **`correct_angle`** (or `getInertialHeading()`) from **motor-control.h**.
- **Path viewer:** EZ-Template’s path comes from **`autonPath`**, **`injectPath()`**, **`Coordinate`**, etc. RW-Template-Test doesn’t have that path model. You can either:
  - Stub the path viewer (nothing drawn), or
  - Implement a minimal path representation and draw lines on the brain from (x_pos, y_pos) history or a small list of waypoints. That would be a separate, simplified implementation.

### 7. Pics / images

- LVGL uses **LV_IMG_DECLARE** and C arrays in **pics/** (e.g. `logo.c`, `matchField.c`). VexCode’s Brain.Screen does **not** use LVGL image descriptors.
- **Options:**
  - **Skip images:** Use text and colored rectangles for logo, field, and robot. Easiest.
  - **Vex images:** If the Vex C++ API supports drawing images from file (e.g. on SD) or from a buffer, you could load and draw in the screen region. This would require converting or re-exporting your assets to a format Vex supports.
  - **Simple graphics:** Draw shapes (e.g. rectangle for field, small filled rect for robot) and optional grid lines. Good compromise.

So: **do not** copy the LVGL `pics/` C arrays into RW-Template-Test expecting them to work as-is; either leave them out or replace with Vex-compatible assets or shapes.

### 8. drive.cpp / drive.hpp

- **2145Z-EzTemp** screen uses **drive.hpp** for **Coordinate**, **injectPath**, **get_velocity**, **get_time_point**, **autonPath**, and **matchState**. In RW-Template-Test you don’t have EZ-Template or that drive layer.
- For the **screen only** you need:
  - **Position:** Use **motor-control**’s **x_pos**, **y_pos**, **correct_angle** (or heading) for console and any simple “robot dot” on a field.
  - **Path viewer:** As above, either stub or implement a small, local path representation; no need to port the full EZ-Template drive/drive.hpp unless you are also porting autons that use it.
- So you **do not** need to copy **drive.cpp**/drive.hpp into RW-Template-Test just for the brain screen. You only need the **auton function pointers** and names (in AutonSel) and the **odom/position** from motor-control.

### 9. Wiring into main and pre_auton

- In **main** (or wherever you set up the program):
  - After your other init, call **`uiInit()`** (which clears and draws the initial layout) and **`autoSelectorInit()`** (which draws the auton list and sets up touch callbacks).
- In **pre_auton** (or the same init):
  - Call **`auton_sel.selector_populate(...)`** and set default **selector_callback** / **selector_name**.
  - If you implement SD load, call **load_selected_auton_from_sd()** after populate and use it to set **selector_callback** / **selector_name** if a valid name is found.
- In **autonomous()**:
  - Call **`auton_sel.selector_callback();`** so the selected auton runs.
- In **usercontrol()** (optional):
  - If you want live odom on the console, call **`print(1, "X: " + std::to_string(x_pos));`** and similarly for y and angle, in your main loop (with a small wait). That matches the EzTemp behavior.

### 10. Optional: angle check and path viewer tasks

- **Angle check:** If you keep it, run a thread that periodically reads **getInertialHeading()** (or correct_angle) and a target angle, and updates a small on-screen “angle check” area (e.g. text or a bar). No LVGL msgbox; use **Brain.Screen.printAt()** in a fixed rectangle and clear/redraw as needed.
- **Path viewer:** Either leave it out or implement a simple version: e.g. a thread that every N ms appends (x_pos, y_pos) to a short list and draws a polyline on the brain, or that draws a fixed waypoint list. No need for injectPath unless you port that separately.

### 11. Build and include paths

- Add **screen.cpp** to the build (e.g. under **custom/src/** or **src/** so the makefile’s `wildcard` picks it up).
- Ensure **screen.hpp** is in an include path (e.g. **custom/include/** or **include/**) and that **main.cpp**, **user.cpp**, and any file that calls **auton_sel** or **print()** include **screen.hpp** (or the header that declares them).
- Remove or stub any remaining references to **LVGL**, **PROS**, or **EZ-Template** in the new screen files so the project compiles with VexCode only.

---

## Summary checklist

| Step | Action |
|------|--------|
| 1 | Add **screen.hpp** with AutonSel, AutonObj, alliance enum, colors, uiInit, print, etc. (no LVGL). |
| 2 | Implement **screen.cpp** with Brain.Screen layout: list area, alliance/color boxes, console area, logo/toggle. |
| 3 | Use **Brain.Screen.pressed/released** and **x_position/y_position** for touch; hit-test and update state; redraw. |
| 4 | **selector_populate** in pre_auton; **auton_sel.selector_callback()** in autonomous. |
| 5 | (Optional) Implement SD save/load with Vex file API. |
| 6 | Replace **pros::delay** → **vex::wait**, **pros::Task** → **vex::thread**; use **x_pos, y_pos, correct_angle** instead of chassis odom. |
| 7 | Skip or replace **pics/** (no LVGL images); use text/shapes or Vex image API if available. |
| 8 | Do **not** port drive.cpp/drive.hpp for screen-only; use motor-control position for console/path. |
| 9 | Call **uiInit()** and **autoSelectorInit()** in init; wire **selector_callback()** in autonomous. |
| 10 | (Optional) Angle check and path viewer with Brain.Screen.printAt and simple drawing. |
| 11 | Add screen sources to build and fix includes so the project compiles. |

After this, your brain screen behavior (auton selection, alliance, console, optional path/angle) will run on **RW-Template-Test** using only VexCode’s Brain.Screen API, without LVGL or PROS.
