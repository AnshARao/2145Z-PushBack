/**
 * v5lvgl.c - LVGL 8.3+ display and input driver for VEX V5 Brain (VexCode)
 *
 * Based on jpearman/vexcode-lvgllib8_X. Updated for LVGL 8.3/8.4:
 * - lv_timer_handler() instead of lv_task_handler()
 * - Same display/indev API (lv_disp_drv_t, flush_cb, touch read_cb).
 *
 * Requires V5 SDK providing: v5.h, vexDisplayCopyRect, vexTouchDataGet,
 * vexTaskAdd, vexTaskSleep, V5_TouchStatus, kTouchEventPress, kTouchEventRelease.
 */

#include "v5.h"
#include "v5lvgl.h"

/* V5 Brain user drawing area (below menu bar) */
#define V5_HOR_RES_MAX 480
#define V5_VER_RES_MAX 240

/*----------------------------------------------------------------------------
 * Flush: copy LVGL draw buffer to the Brain display
 *----------------------------------------------------------------------------*/
static void disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
	vexDisplayCopyRect(area->x1, area->y1, area->x2, area->y2, (uint32_t *)color_p, area->x2 - area->x1 + 1);
	lv_disp_flush_ready(disp_drv);
}

/*----------------------------------------------------------------------------
 * Touch: read V5 touch and feed to LVGL
 *----------------------------------------------------------------------------*/
static void touch_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
	V5_TouchStatus status;
	vexTouchDataGet(&status);

	if (status.lastEvent == kTouchEventPress)
		data->state = LV_INDEV_STATE_PRESSED;
	else if (status.lastEvent == kTouchEventRelease)
		data->state = LV_INDEV_STATE_RELEASED;
	else
		data->state = LV_INDEV_STATE_RELEASED; /* no more data */

	data->point.x = (int32_t)status.lastXpos;
	data->point.y = (int32_t)status.lastYpos;
}

/*----------------------------------------------------------------------------
 * LVGL timer task: tick + timer handler (replaces lv_task_handler in 8.3+)
 *----------------------------------------------------------------------------*/
#define V5_LVGL_MS 5

static int lvgl_task(void)
{
	while (1) {
		lv_tick_inc(V5_LVGL_MS);
		lv_timer_handler();
		vexTaskSleep(V5_LVGL_MS);
	}
	return 0;
}

/*----------------------------------------------------------------------------
 * Initialize LVGL for V5 (VexCode)
 *----------------------------------------------------------------------------*/
void v5_lv_init(void)
{
	lv_init();

	/* Draw buffer (double buffer for better performance) */
	static lv_disp_draw_buf_t disp_buf;
	static lv_color_t buf1[V5_HOR_RES_MAX * V5_VER_RES_MAX];
	static lv_color_t buf2[V5_HOR_RES_MAX * V5_VER_RES_MAX];
	lv_disp_draw_buf_init(&disp_buf, buf1, buf2, V5_HOR_RES_MAX * V5_VER_RES_MAX);

	/* Display driver */
	static lv_disp_drv_t disp_drv;
	lv_disp_drv_init(&disp_drv);
	disp_drv.hor_res = V5_HOR_RES_MAX;
	disp_drv.ver_res = V5_VER_RES_MAX;
	disp_drv.flush_cb = disp_flush;
	disp_drv.draw_buf = &disp_buf;
	lv_disp_drv_register(&disp_drv);

	/* Touch input */
	static lv_indev_drv_t indev_drv;
	lv_indev_drv_init(&indev_drv);
	indev_drv.type = LV_INDEV_TYPE_POINTER;
	indev_drv.read_cb = touch_read;
	lv_indev_drv_register(&indev_drv);

	/* Default screen (optional; your code can replace it) */
	lv_obj_t *scr = lv_obj_create(NULL);
	lv_obj_set_size(scr, V5_HOR_RES_MAX, V5_VER_RES_MAX);
	lv_scr_load(scr);

	/* Run LVGL timer in a V5 task */
	vexTaskAdd(lvgl_task, V5_LVGL_MS, "LVGL");
}
