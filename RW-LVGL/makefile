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

# LVGL: official integration (https://docs.lvgl.io/8.4/porting/project.html)
ifneq ($(wildcard lvgl/lvgl.h),)
  USE_LVGL = 1
endif
ifneq ($(wildcard lvgl/src/lvgl.h),)
  USE_LVGL = 1
endif
ifneq ($(USE_LVGL),)
  LVGL_DIR_NAME ?= lvgl
  LVGL_DIR      ?= $(CURDIR)
  CSRCS =
  include $(LVGL_DIR)/$(LVGL_DIR_NAME)/lvgl.mk
  SRC_C += src/v5lvgl.c
endif

# Object files (project sources + LVGL CSRCS when present)
OBJ = $(patsubst %.cpp,$(BUILD)/%.o,$(filter %.cpp,$(SRC_C)))
OBJ += $(patsubst %.c,$(BUILD)/%.o,$(filter %.c,$(SRC_C)))
ifneq ($(USE_LVGL),)
  OBJ += $(patsubst %.c,$(BUILD)/%.o,$(CSRCS))
endif

# location of include files that c and cpp files depend on
SRC_H  = $(wildcard include/*.h)
ifneq ($(USE_LVGL),)
  SRC_H += lv_conf.h
endif

# additional dependancies
SRC_A  = makefile

# project header file locations (add . and lvgl for LVGL: lv_conf.h and lvgl.h)
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
