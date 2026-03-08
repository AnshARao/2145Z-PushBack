# Portable recursive .c list (no Unix find; works on Windows)
LVGL_EXAMPLES_BASE := $(LVGL_DIR)/$(LVGL_DIR_NAME)/examples
CSRCS += $(patsubst $(LVGL_DIR)/$(LVGL_DIR_NAME)/%,$(LVGL_DIR_NAME)/%,$(wildcard $(LVGL_EXAMPLES_BASE)/*.c $(LVGL_EXAMPLES_BASE)/*/*.c $(LVGL_EXAMPLES_BASE)/*/*/*.c $(LVGL_EXAMPLES_BASE)/*/*/*/*.c))

DEPPATH += --dep-path $(LVGL_DIR)/$(LVGL_DIR_NAME)/examples
CFLAGS += "-I$(LVGL_DIR)/$(LVGL_DIR_NAME)/examples"
