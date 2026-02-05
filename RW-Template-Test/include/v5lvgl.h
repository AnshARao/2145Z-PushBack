/**
 * v5lvgl.h - LVGL 8.x display and input driver for VEX V5 Brain (VexCode)
 *
 * Based on jpearman/vexcode-lvgllib8_X. Adapted for LVGL 8.3.4–8.4 and
 * current VexCode SDK. Requires V5 SDK with v5.h (vexDisplayCopyRect,
 * vexTouchDataGet, vexTaskAdd, vexTaskSleep).
 *
 * Usage: Call v5_lv_init() once from pre_auton or initialize before
 * creating any LVGL UI. The driver starts an internal task that runs
 * lv_timer_handler() and lv_tick_inc().
 */

#ifndef V5_LVGL_H_
#define V5_LVGL_H_

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize LVGL and register V5 Brain display + touch. Call once at startup. */
void v5_lv_init(void);

#ifdef __cplusplus
}
#endif

#endif /* V5_LVGL_H_ */
