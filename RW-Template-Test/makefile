# VEXcode makefile 2019_03_26_01

# show compiler output
VERBOSE = 0

# include toolchain options
include vex/mkenv.mk

# location of the project source cpp and c files
SRC_C  = $(wildcard src/*.cpp)
SRC_C += $(wildcard src/*.c)
SRC_C += $(wildcard src/*/*.cpp)
SRC_C += $(wildcard src/*/*.c)
SRC_C += $(wildcard custom/src/*.cpp)
SRC_C += $(wildcard custom/src/*.c)

# LVGL 8.3.4–8.4: add driver and LVGL sources when lvgl/ is present (see LVGL_README.md)
# Support both v8.4 (lvgl/lvgl.h) and older (lvgl/src/lvgl.h) layout
ifneq ($(wildcard lvgl/lvgl.h),)
  USE_LVGL = 1
endif
ifneq ($(wildcard lvgl/src/lvgl.h),)
  USE_LVGL = 1
endif
ifneq ($(USE_LVGL),)
  SRC_C += src/v5lvgl.c
  SRC_C += $(wildcard lvgl/src/*.c)
  SRC_C += $(wildcard lvgl/src/*/*.c)
  SRC_C += $(wildcard lvgl/src/*/*/*.c)
  SRC_C += $(wildcard lvgl/src/*/*/*/*.c)
endif

# Object files (preserve path so lvgl/*.o do not clash)
OBJ = $(patsubst %.cpp,$(BUILD)/%.o,$(filter %.cpp,$(SRC_C)))
OBJ += $(patsubst %.c,$(BUILD)/%.o,$(filter %.c,$(SRC_C)))

# location of include files that c and cpp files depend on
SRC_H  = $(wildcard include/*.h)
ifneq ($(USE_LVGL),)
  SRC_H += lv_conf.h
endif

# additional dependancies
SRC_A  = makefile

# project header file locations (add . and lvgl when using LVGL; lvgl for lvgl.h, . for lv_conf.h)
INC_F  = include
ifneq ($(USE_LVGL),)
  INC_F += . lvgl
  CXX_FLAGS += -DUSE_LVGL
  CFLAGS += -DUSE_LVGL
endif

# build targets
all: $(BUILD)/$(PROJECT).bin

# include build rules
include vex/mkrules.mk
