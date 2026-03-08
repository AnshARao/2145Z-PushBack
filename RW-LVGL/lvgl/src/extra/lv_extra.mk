# Portable recursive .c list (no Unix find; works on Windows)
LVGL_EXTRA_BASE := $(LVGL_DIR)/$(LVGL_DIR_NAME)/src/extra
CSRCS += $(patsubst $(LVGL_DIR)/$(LVGL_DIR_NAME)/%,$(LVGL_DIR_NAME)/%,$(wildcard $(LVGL_EXTRA_BASE)/*.c $(LVGL_EXTRA_BASE)/*/*.c $(LVGL_EXTRA_BASE)/*/*/*.c))

DEPPATH += --dep-path $(LVGL_DIR)/$(LVGL_DIR_NAME)/src/extra
CFLAGS += "-I$(LVGL_DIR)/$(LVGL_DIR_NAME)/src/extra"
