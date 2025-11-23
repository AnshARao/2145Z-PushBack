#pragma once

#include "liblvgl/misc/lv_color.h"
#include "screen.hpp"
#include <functional>
#include <string>

const lv_color_t theme_color = lv_color_hex(0xffade7);
const lv_color_t theme_accent = lv_color_hex(0xffffff);
const lv_color_t red = lv_color_hex(0xff3643);
const lv_color_t orange = lv_color_hex(0xffa500);
const lv_color_t yellow = lv_color_hex(0xffd700);
const lv_color_t blue = lv_color_hex(0x01b1f0);
const lv_color_t green = lv_color_hex(0x22d428);
const lv_color_t cyan = lv_color_hex(0x00ffff);
const lv_color_t purple = lv_color_hex(0x800080);
const lv_color_t violet = lv_color_hex(0xb672f2);
const lv_color_t pink = lv_color_hex(0xffade7);
const lv_color_t gray = lv_color_hex(0x575757);
const lv_color_t black = lv_color_hex(0x000000);
const lv_color_t white = lv_color_hex(0xffffff);

enum Fields {MATCH = 0, SKILLS = 1};
inline int curField = MATCH;

void angleCheckTask();
void pathViewerTask();

class AutonObj {
    public:
        std::function<void()> callback = nullptr;
        std::string name = "no name";
        lv_color_t color = pink;
};