# LVGL 8.3.4–8.4 on VEX V5 (VexCode) – RW-Template-Test

This project can be built **with or without** LVGL. When the `lvgl/` directory is present and contains LVGL 8.3.4 or 8.4 source, the makefile builds the V5 driver (`v5lvgl.c`) and links LVGL so you can use the Brain screen as an LVGL display with touch.

## 1. Get the `lvgl` folder (required for LVGL build)

You must have the **`lvgl`** folder inside **`RW-Template-Test`** with the LVGL 8.4.0 (or 8.3.4) source. The repo may track it as a **git submodule**; if `lvgl` is missing or only contains `.git`, use one of the options below.

### Option A: Submodule (if this repo has the submodule registered)

From the **repository root** (the folder that contains `RW-Template-Test`):

```bash
git submodule update --init RW-Template-Test/lvgl
```

Then switch the submodule to LVGL 8.4.0:

```bash
cd RW-Template-Test/lvgl
git checkout v8.4.0
cd ../..
```

After this, `RW-Template-Test/lvgl/` will contain the full LVGL 8.4.0 source (`lvgl.h`, `src/`, etc.).

### Option B: Clone LVGL 8.4.0 manually (use if submodule is not set up or `lvgl` is empty)

From **`RW-Template-Test`** (this project folder). If `lvgl` already exists but is empty (only `.git`), remove it first, then:

```bash
git clone --depth 1 --branch v8.4.0 https://github.com/lvgl/lvgl.git lvgl
```

If you prefer 8.3.4, use `--branch v8.3.4`.

### Option C: Download ZIP

1. Go to [https://github.com/lvgl/lvgl/releases](https://github.com/lvgl/lvgl/releases).
2. Download the source code zip for **v8.4.0** (or v8.3.4).
3. Extract it so that **`RW-Template-Test`** contains a folder named **`lvgl`** with:
   - `lvgl/lvgl.h` or `lvgl/src/lvgl.h`
   - `lvgl/src/core/`, `lvgl/src/draw/`, `lvgl/src/widgets/`, etc.

If the zip extracts to `lvgl-8.4.0/`, rename it to `lvgl` or move its contents into a folder named `lvgl`.

## 2. Configuration

- **`lv_conf.h`** – Already in the project root. It is used automatically when LVGL is built (include path includes `.`). You can edit it to enable/disable widgets, fonts, or memory size (see [LVGL porting guide](https://docs.lvgl.io/8.4/porting/index.html)).
- **`include/v5lvgl.h`**, **`src/v5lvgl.c`** – V5 display and touch driver for LVGL 8.3+ (flush + touch read + `lv_timer_handler` task).

## 3. V5 SDK requirement (v5.h and display/task API)

The driver uses **V5 kernel/internal API** that may not be in every VexCode SDK:

- **`v5.h`** – VEX V5 C API header.
- **`vexDisplayCopyRect`** – Copy a rectangle from a pixel buffer to the Brain display (see [Display SDK notes](https://internals.vexide.dev/sdk/display)).
- **`vexTouchDataGet`**, **`V5_TouchStatus`**, **`kTouchEventPress`**, **`kTouchEventRelease`**, **`lastXpos`**, **`lastYpos`** – Touch input.
- **`vexTaskAdd`**, **`vexTaskSleep`** – Task creation and delay for the LVGL timer loop.

When you build from **VexCode IDE**, the SDK path (`T=` or `VEX_SDK_PATH`) usually includes these. If the build fails with “undefined reference to vexDisplayCopyRect” or “v5.h not found”:

- Confirm you are using a VexCode/VEX SDK that ships the full V5 C API (including display and tasks).
- The [jpearman/vexcode-lvgllib8_X](https://github.com/jpearman/vexcode-lvgllib8_X) project was built against a VEX toolchain that provides these; you may need the same or a compatible SDK/toolchain.

## 4. Build

With `lvgl/` in place (and `lvgl/src/lvgl.h` present):

```bash
make
```

Or build from VexCode IDE as usual; the same makefile is used and will include LVGL and `v5lvgl.c` when it sees `lvgl/src/lvgl.h`.

Without the `lvgl/` directory, the project builds as before (no LVGL, no `v5lvgl.c`).

## 5. Use LVGL in your code

1. **Initialize once** (e.g. in `runPreAutonomous()` or before `competition_initialize`):

   ```c
   #include "v5lvgl.h"

   void runPreAutonomous() {
     vexcodeInit();
     v5_lv_init();   // Start LVGL + display + touch + timer task
     // ... then create your UI or call an example ...
   }
   ```

2. **Example screen**: When the project is built with LVGL (`lvgl/` present), `runPreAutonomous()` already calls `v5_lv_init()` and then `lvgl_example_screen()`, which shows a label **"LVGL 8.4 Ready"** and a **"Click me"** button. Tapping the button changes the label to **"Button pressed!"** You can replace or remove this in `custom/src/user.cpp` (inside `#ifdef USE_LVGL`).

3. **Create your own UI** after `v5_lv_init()`: use standard LVGL 8.x APIs (`lv_obj_t*`, `lv_btn_create`, `lv_label_set_text`, `lv_obj_add_event_cb`, etc.). The default screen is 480×240; replace it with `lv_scr_load(your_screen)` if you want your own root object.

4. **Do not** use `Brain.Screen.*` for the same area LVGL is drawing to; the driver owns the display buffer.

## 6. Version notes

- **LVGL 8.3.4–8.4**: The driver uses `lv_timer_handler()` (not `lv_task_handler()`), `lv_disp_draw_buf_t`, and the standard flush/indev callbacks. It is compatible with 8.3.4 and 8.4.x.
- **jpearman/vexcode-lvgllib8_X**: That repo uses an older LVGL 8.3.7 and a different makefile (builds a static lib). This setup builds LVGL and the driver directly into your project and uses LVGL 8.4 (or 8.3.4) with the same driver idea (flush + touch + timer task).

## 7. Troubleshooting

| Problem | What to check |
|--------|----------------|
| `lvgl.h: No such file` | Ensure `lvgl/` is present and contains `lvgl/src/lvgl.h` (correct clone or extract). |
| `lv_conf.h: No such file` | Makefile adds `.` to include path when LVGL is used; `lv_conf.h` must be in the project root. |
| `undefined reference to vexDisplayCopyRect` (or `vexTaskAdd`) | SDK does not expose V5 display/task API; use a VexCode/SDK that provides `v5.h` and these symbols (see §3). |
| Build very slow on Windows | Known with some Clang/IDE setups; try building from a WSL or macOS shell, or reduce LVGL features in `lv_conf.h`. |
