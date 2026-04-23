# LVGL 8.3 vs 9.2 — Doc & API Reference

Quick reference for porting from [LVGL 8.3](https://docs.lvgl.io/8.3/index.html) to [LVGL 9.2](https://docs.lvgl.io/9.2/index.html) (e.g. screen.cpp from 2145Z-EzTemp).

---

## Documentation entry points

| Topic | LVGL 8.3 | LVGL 9.2 |
|-------|----------|----------|
| **Home** | [docs.lvgl.io/8.3](https://docs.lvgl.io/8.3/index.html) | [docs.lvgl.io/9.2](https://docs.lvgl.io/9.2/index.html) |
| **Objects / screens** | Overview → [Objects](https://docs.lvgl.io/8.3/overview/object.html) (screens, layers, `lv_scr_act()`, `lv_scr_load()`) | Overview → [Objects](https://docs.lvgl.io/9.2/overview/objects/index.html) → [Screens](https://docs.lvgl.io/9.2/overview/objects/screens.html) |
| **Images** | Overview → [Images](https://docs.lvgl.io/8.3/overview/image.html) | Overview → [Images](https://docs.lvgl.io/9.2/overview/images/index.html); Widgets → [Image (lv_image)](https://docs.lvgl.io/9.2/widgets/image/lv_image.html) |
| **Display** | Overview → [Displays](https://docs.lvgl.io/8.3/overview/display.html) | Overview → [Displays](https://docs.lvgl.io/9.2/overview/displays/index.html) |
| **Styles** | Overview → [Style properties](https://docs.lvgl.io/8.3/overview/style-props.html) | Overview → [Style properties](https://docs.lvgl.io/9.2/overview/styles/properties.html) |
| **Events** | Overview → [Events](https://docs.lvgl.io/8.3/overview/event.html) | Overview → [Events](https://docs.lvgl.io/9.2/overview/events/index.html) |
| **Widgets** | [Widgets](https://docs.lvgl.io/8.3/widgets/index.html) (core + extra) | [Widgets](https://docs.lvgl.io/9.2/widgets/index.html) (same idea, some renames) |

---

## Naming and API changes (8.3 → 9.2)

### Display / screen
- **8.3:** `lv_scr_act()`, `lv_scr_load(scr)`, `lv_scr_load_anim(...)`, `lv_disp_*`
- **9.2:** `lv_screen_active()`, `lv_screen_load(scr)`, `lv_screen_load_anim(...)`, `lv_display_*`  
  So: **scr** → **screen**, **disp** → **display**.

### Image widget
- **8.3:** `lv_img_*`, `lv_img_dsc_t`, `LV_IMG_DECLARE`, `LV_IMG_CF_*`
- **9.2:** `lv_image_*`, `lv_image_dsc_t`, `LV_IMAGE_DECLARE`, `lv_color_format_t` (e.g. `LV_COLOR_FORMAT_ARGB8888`)  
  See `docs/LVGL9_IMAGE_IMPORT.md` in this project.

### Object / style
- **8.3:** `lv_obj_clear_flag` / `lv_obj_add_flag`, `lv_obj_set_style_transform_angle`, `lv_obj_set_style_img_*`, `lv_obj_move_foreground`
- **9.2:** `lv_obj_remove_flag` / `lv_obj_add_flag`, `lv_obj_set_style_transform_rotation`, `lv_obj_set_style_image_*`, `lv_obj_move_to_index` (or move_foreground if still present in v9 API map)  
  Check `include/liblvgl/lv_api_map_v8.h` for compatibility macros if enabled.

### Events
- **8.3:** `lv_obj_add_event_cb`, `lv_event_get_user_data`, etc.
- **9.2:** Often `lv_obj_add_event_cb` still used; event codes and payload types may differ—check [Events](https://docs.lvgl.io/9.2/overview/events/index.html).

### List
- **8.3:** `lv_list_add_btn`, `lv_list_create`
- **9.2:** `lv_list_add_button`, `lv_list_create` (and possibly other renames; see List widget docs).

---

## Where to look in each version

- **8.3:** [Overview](https://docs.lvgl.io/8.3/overview/index.html) (objects, images, display, styles, events), [Widgets](https://docs.lvgl.io/8.3/widgets/index.html) (core = btn, label, img, slider, etc.; extra = list, msgbox, etc.).
- **9.2:** Same sections under [Overview](https://docs.lvgl.io/9.2/overview/index.html) and [Widgets](https://docs.lvgl.io/9.2/widgets/index.html); 9.2 adds more examples and some new widgets (e.g. Scale). Use the 9.2 sidebar or search to find the 9.2 equivalent of any 8.3 page.

Use this file together with `LVGL9_IMAGE_IMPORT.md` when bringing 2145Z-EzTemp screen logic into this LVGL 9.2 project.
