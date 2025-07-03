#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "autons.hpp"

const lv_color32_t theme_color = lv_color_hex(0xffade7);
const lv_color32_t theme_accent = lv_color_hex(0xffffff);
const lv_color32_t red = lv_color_hex(0xff3643);
const lv_color32_t orange = lv_color_hex(0xffa500);
const lv_color32_t yellow = lv_color_hex(0xffd700);
const lv_color32_t blue = lv_color_hex(0x01b1f0);
const lv_color32_t green = lv_color_hex(0x22d428);
const lv_color32_t cyan = lv_color_hex(0x00ffff);
const lv_color32_t purple = lv_color_hex(0x800080);
const lv_color32_t violet = lv_color_hex(0xb672f2);
const lv_color32_t pink = lv_color_hex(0xffade7);
const lv_color32_t gray = lv_color_hex(0x575757);
const lv_color32_t black = lv_color_hex(0x000000);
const lv_color32_t white = lv_color_hex(0xffffff);

enum Fields {MATCH = 1, SKILLS = 2};
inline int currentField = Fields::MATCH;

// Auton selector
void angleCheckTask();
void pathViewerTask();

class AutonObj {
    public:
        function<void()> callback = doNothing;
        string name = "no name";
        lv_color32_t color = pink;
};

class AutonSel {
    public:
        vector<AutonObj> autons = {};
        function<void()> selector_callback = doNothing;
        void selector_populate(vector<AutonObj> auton_list);
};

extern string controllerInput;
extern AutonSel auton_sel;

// Main UI
extern lv_obj_t* colorInd;
extern lv_obj_t* autoSelector;
extern lv_obj_t* autonTable;
extern lv_obj_t* autonField;
extern lv_obj_t* autonRobot;
extern lv_obj_t* autonUp;
extern lv_obj_t* autonDown;
extern lv_obj_t* logoImg;
extern lv_obj_t* console_container;
extern lv_obj_t* console_label;
extern lv_obj_t* angleViewer;
extern lv_obj_t* angleText;
extern lv_obj_t* allianceInd;
extern lv_obj_t* allianceOverlay;


void uiInit();

// Auton Selector UI
void autoSelectorInit();

void print(const std::string& msg);