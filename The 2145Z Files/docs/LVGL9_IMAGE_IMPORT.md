# Importing Images in LVGL 9 (this project)

This project uses **LVGL 9.2.0**. Image handling is different from LVGL 8 (e.g. 2145Z-EzTemp).

## Quick answer: “True color with alpha” in LVGL 9

- **LVGL 8:** `LV_IMG_CF_TRUE_COLOR_ALPHA`
- **LVGL 9:** Use one of:
  - **`LV_COLOR_FORMAT_NATIVE_WITH_ALPHA`** – best choice; matches your `LV_COLOR_DEPTH` (e.g. ARGB8888 at 32-bit, RGB565A8 at 16-bit).
  - Or explicitly **`LV_COLOR_FORMAT_ARGB8888`** (32-bit color + alpha) or **`LV_COLOR_FORMAT_RGB565A8`** (16-bit color + separate 8-bit alpha).

The descriptor type is **`lv_image_dsc_t`** (replaces `lv_img_dsc_t`), and the header includes a **magic** value **`LV_IMAGE_HEADER_MAGIC`** (0x19) plus **`lv_color_format_t`** in `header.cf`.

## Using the LVGL Image Converter (LVGL 9)

**Available color formats in the v9 converter:** RGB565, RGB565A8, RGB888, XRGB8888, **ARGB8888**. For the old "true color with alpha" behavior, use **ARGB8888** (or **RGB565A8** to save memory).

1. Open **[LVGL Image Converter](https://lvgl.io/tools/imageconverter)**.
2. **Select “LVGL v9”** (not v8).
3. Upload your image(s).
4. **Color format:** For “true color with alpha” choose **ARGB8888**. (The v9 dropdown only lists: RGB565, RGB565A8, RGB888, XRGB8888, ARGB8888.) For smaller images with alpha use **RGB565A8**. The generated C will use the correct `lv_image_dsc_t` and `lv_color_format_t` (e.g. `LV_COLOR_FORMAT_ARGB8888` or `LV_COLOR_FORMAT_NATIVE_WITH_ALPHA`).
5. Convert and download the `.c` file.
6. Add the `.c` to your project and in your code:
   - Declare: **`LV_IMAGE_DECLARE(my_image_name);`**
   - Set source: **`lv_image_set_src(img_obj, &my_image_name);`**

This project does **not** include the LVGL 8 API map, so use the v9 names: `lv_image_*`, `LV_IMAGE_DECLARE`, `lv_obj_set_style_image_*`, etc.

## Manual C array (e.g. migrating from LVGL 8)

If you have an old v8 C array (e.g. `LV_IMG_CF_TRUE_COLOR_ALPHA`) or need to hand-write a descriptor:

1. **Descriptor layout (LVGL 9)**  
   The generated or manual descriptor must match **`lv_image_dsc_t`**:

   - **`header.magic`** = **`LV_IMAGE_HEADER_MAGIC`** (0x19)
   - **`header.cf`** = a **`lv_color_format_t`** value (e.g. `LV_COLOR_FORMAT_ARGB8888` or `LV_COLOR_FORMAT_NATIVE_WITH_ALPHA`)
   - **`header.flags`** = 0 (or `LV_IMAGE_FLAGS_*` if needed)
   - **`header.w`** / **`header.h`** = width / height in pixels
   - **`header.stride`** = bytes per row (e.g. `w * 4` for ARGB8888)
   - **`data_size`** = size of the pixel array in bytes
   - **`data`** = pointer to your `const uint8_t` pixel data

2. **Pixel layout**  
   Must match the chosen format, e.g.:
   - **ARGB8888:** 4 bytes per pixel (A, R, G, B).
   - **RGB565A8:** RGB565 (2 bytes) per pixel plus a separate alpha byte per pixel (color array then alpha array).

3. **Declaration and usage**  
   - **`LV_IMAGE_DECLARE(my_image_name);`**
   - **`lv_image_set_src(obj, &my_image_name);`**

## API mapping (LVGL 8 → 9, for screen replication)

| LVGL 8 (2145Z-EzTemp)     | LVGL 9 (this project)        |
|----------------------------|------------------------------|
| `lv_img_create()`          | `lv_image_create()`          |
| `lv_img_set_src()`         | `lv_image_set_src()`         |
| `lv_img_set_angle()`       | `lv_image_set_rotation()` (angle in 0.1°, e.g. 900 = 90°) |
| `LV_IMG_DECLARE()`         | `LV_IMAGE_DECLARE()`         |
| `lv_img_dsc_t`             | `lv_image_dsc_t`             |
| `LV_IMG_CF_TRUE_COLOR_ALPHA` | `LV_COLOR_FORMAT_NATIVE_WITH_ALPHA` or `LV_COLOR_FORMAT_ARGB8888` |
| `lv_obj_set_style_img_recolor()`   | `lv_obj_set_style_image_recolor()`   |
| `lv_obj_set_style_img_recolor_opa()` | `lv_obj_set_style_image_recolor_opa()` |

Using the v9 converter and the table above, you can replicate the screen logic from 2145Z-EzTemp (including logo, overlays, field images) in this LVGL 9 project.
