# LVGL porting checklist (jpearman/vexcode-lvgllib8_X → RW-LVGL)

Based on [jpearman/vexcode-lvgllib8_X](https://github.com/jpearman/vexcode-lvgllib8_X). **LVGL 8.4 is fully ported** for the VEX V5 Brain.

---

## Port status: complete

| LVGL 8.4 requirement | Done |
|----------------------|------|
| `lv_init()` | Yes, in `v5_lv_init()` |
| Display driver (flush_cb) | Yes, `disp_flush` → `vexDisplayCopyRect` |
| Draw buffer + `lv_disp_drv_register` | Yes, full-screen double buffer |
| Input device (read_cb) | Yes, `touch_read` → `vexTouchDataGet` |
| Tick source | Yes, `lv_tick_inc()` in task |
| Timer handler | Yes, `lv_timer_handler()` in task |
| `lv_conf.h` (root, enabled) | Yes, 32-bit, memory, DPI, fonts, theme |
| Build (makefile + lvgl.mk) | Yes, official `LVGL_DIR` / include lvgl.mk |
| Init called from user code | Yes, `v5_lv_init()` in `runPreAutonomous()` |

No further porting steps are required. What remains is build environment and using the API.

---

## What you already did

- [x] **`lvgl/`** – Copied the LVGL folder (e.g. from jpearman or from [lvgl/lvgl](https://github.com/lvgl/lvgl) v8.4.0).
- [x] **`include/v5lvgl.h`** – From jpearman; declares `v5_lv_init()` and includes LVGL.
- [x] **`include/vex.h`** – From jpearman; pulls in `v5.h`, `v5_vcs.h`, and `v5lvgl.h`.

---

## What was added for you

1. **`src/v5lvgl.c`**  
   Display and touch driver for the V5 Brain. Uses:
   - `vexDisplayCopyRect` for flush
   - `vexTouchDataGet` for touch
   - `vexTaskAdd` / `vexTaskSleep` for the LVGL task  
   Updated for **LVGL 8.4**: `lv_timer_handler()` instead of `lv_task_handler()`, and `LV_INDEV_STATE_PRESSED` / `LV_INDEV_STATE_RELEASED`.

2. **`lv_conf.h`** (project root)  
   Copied from `lvgl/lv_conf_template.h`, enabled (`#if 1`), and **`LV_COLOR_DEPTH`** set to **32** for the V5 display.

3. **`include/v5lvgl.h`**  
   Include changed from `#include "../lvgl/lvgl.h"` to `#include "lvgl/lvgl.h"` so that with `-I.` the compiler finds `lvgl/lvgl.h` from the project root.

4. **Makefile**  
   - When `lvgl/lvgl.h` or `lvgl/src/lvgl.h` exists: builds `src/v5lvgl.c` and all `lvgl/src/**/*.c`, adds include paths `.` and `lvgl`, and defines `USE_LVGL`.
   - Object paths keep the directory structure so LVGL object files don’t clash.

5. **`custom/src/user.cpp`**  
   In `runPreAutonomous()`, added:
   ```cpp
   #ifdef USE_LVGL
     v5_lv_init();
   #endif
   ```
   so LVGL starts on the Brain when the project is built with LVGL.

---

## What you still need to do (environment / usage)

1. **Build**  
   Build from the VexCode IDE (or `make` in `RW-LVGL`). The makefile will compile LVGL and `v5lvgl.c` only if the `lvgl/` folder is present.

2. **V5 SDK**  
   The driver expects the V5 C API from your toolchain:
   - `v5.h`
   - `vexDisplayCopyRect`, `vexTouchDataGet`, `vexTaskAdd`, `vexTaskSleep`
   - Types: `V5_TouchStatus`, `kTouchEventPress`, `kTouchEventRelease`, `lastXpos`, `lastYpos`  

   If you get “undefined reference to vexDisplayCopyRect” (or similar), your SDK may not expose these. Use a VexCode/toolchain that provides this API (as in jpearman’s setup).

3. **Use LVGL in your code**  
   After `v5_lv_init()`, use normal LVGL 8.x APIs: create objects on the default screen (480×240), or create your own screen and call `lv_scr_load()`. Don’t use `Brain.Screen.*` for the same area LVGL is drawing to.

---

## Summary

| Item                         | Status   |
|-----------------------------|----------|
| `lvgl/` folder              | You had  |
| `include/v5lvgl.h`          | You had (include path fixed) |
| `include/vex.h`             | You had  |
| `src/v5lvgl.c`              | **Added** (LVGL 8.4 compatible) |
| `lv_conf.h` at project root | **Added** (from template, enabled, 32-bit) |
| Makefile (LVGL + paths)     | **Updated** |
| Call `v5_lv_init()` in pre-auton | **Added** in `user.cpp` |

Once the build succeeds and the V5 API is available, the Brain screen will run LVGL and you can build your UI after `v5_lv_init()`.
