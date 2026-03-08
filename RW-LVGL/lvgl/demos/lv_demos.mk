# Portable recursive .c list (no Unix find; works on Windows)
LVGL_DEMOS_BASE := $(LVGL_DIR)/$(LVGL_DIR_NAME)/demos
CSRCS += $(patsubst $(LVGL_DIR)/$(LVGL_DIR_NAME)/%,$(LVGL_DIR_NAME)/%,$(wildcard $(LVGL_DEMOS_BASE)/*.c $(LVGL_DEMOS_BASE)/*/*.c $(LVGL_DEMOS_BASE)/*/*/*.c $(LVGL_DEMOS_BASE)/*/*/*/*.c))

DEPPATH += --dep-path $(LVGL_DIR)/$(LVGL_DIR_NAME)/demos
CFLAGS += "-I$(LVGL_DIR)/$(LVGL_DIR_NAME)/demos"
