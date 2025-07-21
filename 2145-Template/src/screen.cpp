#include <cstddef>
#include <string>
#include "autons.hpp"
#include "controls.hpp"
#include "drive.hpp"
#include "liblvgl/core/lv_event.h"
#include "liblvgl/core/lv_obj.h"
#include "liblvgl/core/lv_obj_pos.h"
#include "liblvgl/core/lv_obj_scroll.h"
#include "liblvgl/core/lv_obj_style.h"
#include "liblvgl/font/lv_font.h"
#include "liblvgl/font/lv_symbol_def.h"
#include "liblvgl/hal/lv_hal_disp.h"
#include "liblvgl/misc/lv_anim.h"
#include "liblvgl/misc/lv_area.h"
#include "liblvgl/misc/lv_style.h"
#include "liblvgl/widgets/lv_img.h"
#include "liblvgl/widgets/lv_label.h"
#include "main.h"  // IWYU pragma: keep
#include "subsystems.hpp"
#include "screen.hpp"

/**
 * @file screen.cpp
 * @brief This file contains the screen function for the brain
 * @details This includes the autonomous selector, ui setup, and auto-shower
 * @author Ansh Rao - 2145Z, with a BIG inspiration from Jordan - 21S
 */
 
// // // // // // Tasks & Non-UI // // // // // //

//
// Object creation
//

lv_obj_t* autoSelector = lv_obj_create(NULL);
lv_obj_t* logoImg = lv_img_create(autoSelector);
lv_obj_t* colorInd = lv_obj_create(autoSelector);
lv_obj_t* colorOverlay = lv_img_create(autoSelector);
lv_obj_t* allianceInd = lv_obj_create(autoSelector);
lv_obj_t* allianceOverlay = lv_img_create(autoSelector);

lv_obj_t* autonTable = lv_list_create(autoSelector);
lv_obj_t* autonUp = lv_label_create(autoSelector);
lv_obj_t* autonDown = lv_label_create(autoSelector);
lv_obj_t* autonField = lv_img_create(autoSelector);
lv_obj_t* autonRobot = lv_img_create(autonField);
lv_obj_t* angleViewer;
lv_obj_t* angleText;

lv_obj_t* console_container = lv_obj_create(autoSelector);
lv_obj_t* console_label = lv_label_create(console_container);

lv_style_t pushback;

LV_IMG_DECLARE(red_alliance);
LV_IMG_DECLARE(blue_alliance);
LV_IMG_DECLARE(colorindOverlay);
LV_IMG_DECLARE(pfp2145);
LV_IMG_DECLARE(matchField);
LV_IMG_DECLARE(skillsField);
LV_IMG_DECLARE(logo);

// // // // // // Tasks & Non-UI // // // // // //

//
// Auton Selector
//

string controllerInput = "";
bool aligning = false;
bool playing = true;
bool showConsole = false;
static const char* allianceColorNames[] = {"Blue", "None", "Red"};

AutonSel auton_sel;

void AutonSel::selector_populate(vector<AutonObj> auton_list) { autons.insert(autons.end(), auton_list.begin(), auton_list.end()); }

void angleCheckTask() {
	while(true) {
		if(aligning) {
			auto target = autonPath.size() > 0 ? autonPath[0].t : 0;
			auto current = fmod(chassis.odom_theta_get(), 360);
			lv_label_set_text(angleText,
							  (util::to_string_with_precision(current, 2) + " °" + "\ntarget: " + util::to_string_with_precision(target, 2)).c_str());
			if(target + 0.15 >= current && target - 0.15 <= current)
				lv_obj_set_style_bg_color(angleViewer, green, LV_PART_MAIN);
			else
				lv_obj_set_style_bg_color(angleViewer, red, LV_PART_MAIN);
		}
		pros::delay(10);
	}
}

int pathIter = 0;
vector<Coordinate> pathDisplay;

void resetViewer(bool full) {
	if(full) {
		auto preference = matchState;
		matchState = MatchStates::DISABLED;
		autonPath = {};
		auton_sel.selector_callback();
		pathDisplay = injectPath(autonPath, 1);
		matchState = preference;
		lv_img_set_src(autonField, &(currentField == Fields::MATCH ? matchField : skillsField));
	}
	pathIter = 0;
}

void pathViewerTask() {
	while(true) {
        if (allianceColor == BLUE) {
            pathDisplay[pathIter].x = -pathDisplay[pathIter].x;
            pathDisplay[pathIter].y = -pathDisplay[pathIter].y;
            pathDisplay[pathIter].t = 360 - pathDisplay[pathIter].t;
            if (pathDisplay[pathIter].t == 0) {
                pathDisplay[pathIter].t = 180;
            } else if (pathDisplay[pathIter].t == 180) {
                pathDisplay[pathIter].t = 0;
            }
        }
		if(pathIter < pathDisplay.size() && pathDisplay.size() > 1 && playing) {
			lv_obj_clear_flag(autonRobot, LV_OBJ_FLAG_HIDDEN);
			lv_obj_set_pos(autonRobot, (1.5 * pathDisplay[pathIter].x) + 97, 95 - (1.5 * pathDisplay[pathIter].y));
			if(pathIter < pathDisplay.size() - 1) {
				lv_img_set_angle(autonRobot, 10 * (pathDisplay[pathIter].t));
				if(pathDisplay[pathIter].left == KEY)
					pros::delay(pathDisplay[pathIter].right);
				else {
					double velocity = get_velocity(pathDisplay[pathIter].left) + get_velocity(pathDisplay[pathIter].right) / 2;
					if(velocity == 0) velocity = get_velocity(pathDisplay[pathIter].left);
					pros::delay(1000 * abs(get_time_point(1, velocity)));
				}
			}
			if(pathIter == 1) pros::delay(500);
			pathIter++;
		} else if(pathIter >= pathDisplay.size()) {
			pros::delay(1000);
			resetViewer(false);
		}
		pros::delay(10);
	}
}

// // // // // // UI // // // // // //

void colorSet(Alliances color, lv_obj_t* object) {
	// Set on screen elements to the corresponding color
	lv_color32_t color_use = theme_accent;
	if(color == Alliances::RED) {
		color_use = red;
	}	else if(color == Alliances::BLUE) {
		color_use = blue;
	}
	lv_obj_set_style_bg_color(object, color_use, LV_PART_MAIN);
}

//
// Main UI
//

void uiInit() {
    // pushback style setup
    lv_style_init(&pushback);
    lv_style_set_bg_color(&pushback, theme_color);
    lv_style_set_outline_color(&pushback, theme_accent);
    lv_style_set_text_color(&pushback, white);
    lv_style_set_opa(&pushback, 255);
    lv_style_set_bg_opa(&pushback, 255);
    lv_style_set_outline_width(&pushback, 5);
    lv_style_set_border_width(&pushback, 0);
    lv_style_set_text_font(&pushback, &lv_font_montserrat_16);
    lv_style_set_radius(&pushback, 0);

    // autoSelector setup
    lv_obj_add_style(autoSelector, &pushback, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(autoSelector, 255, LV_PART_MAIN);
    lv_obj_set_style_outline_opa(autoSelector, 0, LV_PART_MAIN);
    lv_obj_set_style_transform_angle(autoSelector, 0, LV_PART_MAIN);

    // colorInd setup
    lv_obj_clear_flag(colorInd, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_style(colorInd, &pushback, LV_PART_MAIN);
    lv_obj_set_style_outline_width(colorInd, 0, LV_PART_MAIN);
    lv_obj_set_size(colorInd, 51, 51);
    lv_obj_set_pos(colorInd, 185, 120);
    lv_obj_add_flag(colorInd, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_move_foreground(colorInd);

    // colorOverlay setup
    lv_img_set_src(colorOverlay, &colorindOverlay);
    lv_obj_set_size(colorOverlay, 51, 51);
    lv_obj_set_pos(colorOverlay, 185, 120);
    lv_obj_set_style_outline_width(colorOverlay, 0, LV_PART_MAIN);
    lv_obj_set_style_img_recolor_opa(colorOverlay, 255, LV_PART_MAIN);
    lv_obj_set_style_img_recolor(colorOverlay, theme_color, LV_PART_MAIN);
    lv_obj_add_flag(colorOverlay, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_move_foreground(colorOverlay);

    // allianceInd setup
    lv_obj_clear_flag(allianceInd, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_style(allianceInd, &pushback, LV_PART_MAIN);
    lv_obj_set_style_outline_width(allianceInd, 0, LV_PART_MAIN);
    lv_obj_set_size(allianceInd, 51, 51);
    lv_obj_set_pos(allianceInd, 185, 65);
    lv_obj_add_flag(allianceInd, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_move_foreground(allianceInd);

    // allianceOverlay setup
    lv_img_set_src(allianceOverlay, &colorindOverlay);
    lv_obj_set_size(allianceOverlay, 51, 51);
    lv_obj_set_pos(allianceOverlay, 185, 65);
    lv_obj_set_style_outline_width(allianceOverlay, 0, LV_PART_MAIN);
    lv_obj_set_style_img_recolor_opa(allianceOverlay, 255, LV_PART_MAIN);
    lv_obj_set_style_img_recolor(allianceOverlay, theme_color, LV_PART_MAIN);
    lv_obj_add_flag(allianceOverlay, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_move_foreground(allianceOverlay);

    // Set color for allianceInd
    colorSet(allianceColor, allianceInd);

    // Initialize screens
    autoSelectorInit();

    // Load main screen
    lv_scr_load(autoSelector);
}
//
// Auton Selector UI
//

static void selectAuton(lv_event_t* e) {
	AutonObj* getAuton = (AutonObj*)lv_event_get_user_data(e);
	lv_obj_t* target = lv_event_get_target(e);
	for(int i = 0; i < lv_obj_get_child_cnt(autonTable); i++) {
		lv_obj_t* auton = lv_obj_get_child(autonTable, i);
		lv_obj_clear_state(auton, LV_STATE_CHECKED);
	}
	lv_obj_add_state(target, LV_STATE_CHECKED);
	auton_sel.selector_callback = (*getAuton).callback;
	// Set currentField based on selected auton
	if ((*getAuton).callback.target<void(*)()>() && *(*getAuton).callback.target<void(*)()>() == skills) {
		currentField = Fields::SKILLS;
	} else {
		currentField = Fields::MATCH;
	}
	resetViewer(true);
	print(1, "Auton: " + getAuton->name);
}

static void autonUpEvent(lv_event_t* e) {
	lv_obj_scroll_by_bounded(autonTable, 0, lv_obj_get_height(autonTable), LV_ANIM_ON); 
	lv_obj_scroll_by_bounded(console_container, 0, lv_obj_get_height(console_container), LV_ANIM_ON); 
}

static void autonDownEvent(lv_event_t* e) {
	lv_obj_scroll_by_bounded(autonTable, 0, -lv_obj_get_height(autonTable), LV_ANIM_ON); 
	lv_obj_scroll_by_bounded(console_container, 0, -lv_obj_get_height(console_container), LV_ANIM_ON);}

static void angleCheckCloseEvent(lv_event_t* e) { aligning = false; }

lv_event_cb_t AngleCheckCloseEvent = angleCheckCloseEvent;

static void angleCheckEvent(lv_event_t* e) {
	angleViewer = lv_msgbox_create(NULL, "check alignment", "0°", NULL, true);
	angleText = lv_msgbox_get_text(angleViewer);
	aligning = true;

	lv_obj_add_event_cb(lv_msgbox_get_close_btn(angleViewer), AngleCheckCloseEvent, LV_EVENT_PRESSED, NULL);
	lv_obj_add_style(lv_msgbox_get_close_btn(angleViewer), &pushback, LV_PART_MAIN);
	lv_obj_add_style(angleViewer, &pushback, LV_PART_MAIN);
	lv_obj_set_style_text_font(angleViewer, &lv_font_montserrat_30, LV_PART_MAIN);
	lv_obj_set_style_text_font(lv_msgbox_get_title(angleViewer), &lv_font_montserrat_14, LV_PART_MAIN);
	lv_obj_set_style_text_font(lv_msgbox_get_close_btn(angleViewer), &lv_font_montserrat_24, LV_PART_MAIN);
	lv_obj_set_width(angleViewer, 300);
	lv_obj_align(angleViewer, LV_ALIGN_CENTER, 0, 0);
}

static void pauseEvent(lv_event_t* e) {
	auto event = lv_event_get_code(e);
	if(event == LV_EVENT_PRESSING) playing = false;
	if(event == LV_EVENT_CLICKED) playing = true;
}

static void colorEvent(lv_event_t* e) {
	allianceColor = (Alliances)(((int)allianceColor + 1) % 3);
	colorSet(allianceColor, allianceInd);
    if (allianceColor == RED) chassis.drive_angle_set(chassis.odom_theta_get() + 90);
    if (allianceColor == BLUE) chassis.drive_angle_set(chassis.odom_theta_get() + 180);
    if (allianceColor == NONE) chassis.drive_angle_set(chassis.odom_theta_get() - 270);
	resetViewer(true);
    print(2, std::string("Alliance: ") + allianceColorNames[(int)allianceColor]);
}

static void refreshConsoleEvent(lv_event_t* e) {
    unstructured_log.clear();
}

void refresh_console_label() {
    std::string output;

    // Add structured lines
    for (int i = 0; i < STRUCTURED_LINES; i++) {
        output += structured_log[i] + "\n";
    }

    // Add unstructured lines
    for (const std::string& line : unstructured_log) {
        output += line + "\n";
    }

    lv_label_set_text(console_label, output.c_str());

    // Auto-scroll to bottom
    lv_obj_scroll_by_bounded(console_container, 0, -lv_obj_get_height(console_container), LV_ANIM_ON);
}

void print(int line, const std::string& msg) {
    if (line < 0 || line >= STRUCTURED_LINES) return;
    structured_log[line] = msg;
    refresh_console_label();
}

void print(const std::string& msg) {
    unstructured_log.push_back(msg);
    refresh_console_label();
}

lv_event_cb_t SelectAuton = selectAuton;
lv_event_cb_t AutonUpEvent = autonUpEvent;
lv_event_cb_t AutonDownEvent = autonDownEvent;
lv_event_cb_t AngleCheckEvent = angleCheckEvent;
lv_event_cb_t PauseEvent = pauseEvent;
lv_event_cb_t ColorEvent = colorEvent;

static void toggleConsoleEvent(lv_event_t* e) {
    showConsole = lv_obj_has_flag(autonTable, LV_OBJ_FLAG_HIDDEN);
    if (showConsole) {
        lv_obj_clear_flag(autonTable, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(console_container, LV_OBJ_FLAG_HIDDEN);
		print("Auton Selector Selected");

    } else {
        lv_obj_add_flag(autonTable, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(console_container, LV_OBJ_FLAG_HIDDEN);
		print("Console Selected");
    }
}

void autoSelectorInit() {
    // logoImg setup
    lv_obj_set_size(logoImg, 51, 51);
    lv_obj_set_pos(logoImg, 183, 5);
    lv_obj_add_flag(logoImg, LV_OBJ_FLAG_CLICKABLE);
    lv_img_set_src(logoImg, &logo);
    lv_img_set_angle(logoImg, 0);
    lv_obj_add_event_cb(logoImg, toggleConsoleEvent, LV_EVENT_CLICKED, NULL);

    // autonTable setup
    lv_obj_set_size(autonTable, 160, 216);
    lv_obj_align(autonTable, LV_ALIGN_CENTER, -150, 0);
    lv_obj_add_style(autonTable, &pushback, LV_PART_MAIN);
    lv_obj_add_style(autonTable, &pushback, LV_PART_ITEMS);
    lv_obj_set_style_transform_angle(autonTable, 0, LV_PART_MAIN);
    lv_obj_set_style_outline_width(autonTable, 1, LV_PART_ITEMS);
    lv_obj_set_style_pad_hor(autonTable, 0, LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(autonTable, LV_SCROLLBAR_MODE_OFF);
    lv_obj_add_flag(autonTable, LV_OBJ_FLAG_CLICKABLE);
    // Set up list
    for(int i = 0; i < auton_sel.autons.size(); i++) {
        lv_obj_t* new_auto = lv_list_add_btn(autonTable, NULL, (" " + auton_sel.autons[i].name).c_str());
        lv_obj_add_style(new_auto, &pushback, LV_PART_MAIN);
        lv_obj_set_style_text_font(new_auto, &pros_font_dejavu_mono_18, LV_PART_MAIN);
        lv_obj_set_style_bg_color(new_auto, auton_sel.autons[i].color, LV_PART_MAIN);
        lv_obj_set_style_outline_width(new_auto, 1, LV_PART_MAIN);
        lv_obj_set_style_outline_width(new_auto, 4, LV_STATE_CHECKED);
        lv_obj_set_style_outline_width(new_auto, 6, LV_STATE_PRESSED);
        lv_obj_set_style_bg_opa(new_auto, 220, LV_STATE_CHECKED);
        lv_obj_set_style_bg_opa(new_auto, 180, LV_STATE_PRESSED);
        lv_obj_set_style_pad_hor(new_auto, 0, LV_PART_MAIN);
        lv_obj_add_event_cb(new_auto, SelectAuton, LV_EVENT_CLICKED, &auton_sel.autons[i]);
    }

    // console_container and console_label setup
    lv_obj_set_size(console_container, 160, 216);
    lv_obj_align(console_container, LV_ALIGN_CENTER, -150, 0);
    lv_obj_add_flag(console_container, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_style(console_container, &pushback, LV_PART_MAIN);
    lv_obj_set_style_outline_width(console_container, 5, LV_PART_MAIN);
    lv_obj_set_style_bg_color(console_container, black, LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(console_container, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_width(console_label, 200);
    lv_label_set_long_mode(console_label, LV_LABEL_LONG_WRAP);
	lv_obj_set_scrollbar_mode(console_label, LV_SCROLLBAR_MODE_ON);
    lv_obj_set_style_text_font(console_label, &lv_font_montserrat_10, LV_PART_MAIN);

    // autonField setup
    lv_obj_set_size(autonField, 216, 216);
    lv_obj_align(autonField, LV_ALIGN_CENTER, 120, 0);
    lv_obj_add_style(autonField, &pushback, LV_PART_MAIN);
    lv_img_set_src(autonField, &matchField);
    // lv_img_set_angle(autonField, 900);
    lv_obj_add_flag(autonField, LV_OBJ_FLAG_CLICKABLE);
	lv_obj_add_event_cb(autonField, AngleCheckEvent, LV_EVENT_SHORT_CLICKED, NULL);
	lv_obj_add_event_cb(autonField, PauseEvent, LV_EVENT_CLICKED, NULL);
	lv_obj_add_event_cb(autonField, PauseEvent, LV_EVENT_PRESSING, NULL);
    // autonRobot setup
    lv_obj_add_style(autonRobot, &pushback, LV_PART_MAIN);
    lv_img_set_src(autonRobot, &pfp2145);
	lv_obj_set_style_bg_opa(autonRobot, 0, LV_PART_MAIN);
	lv_obj_set_style_outline_width(autonRobot, 0, LV_PART_MAIN);
    lv_obj_add_flag(autonRobot, LV_OBJ_FLAG_HIDDEN);

    // autonUp setup
    lv_obj_align(autonUp, LV_ALIGN_CENTER, -45, 80);
    lv_obj_add_style(autonUp, &pushback, LV_PART_MAIN);
    lv_label_set_text(autonUp, LV_SYMBOL_UP "\n" LV_SYMBOL_UP "\n" LV_SYMBOL_UP);
    lv_obj_add_flag(autonUp, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_text_opa(autonUp, 128, LV_STATE_PRESSED);
    lv_obj_set_style_text_font(autonUp, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_line_space(autonUp, -12, LV_PART_MAIN);
    lv_obj_set_style_outline_width(autonUp, 0, LV_PART_MAIN);
    lv_obj_add_event_cb(autonUp, AutonUpEvent, LV_EVENT_CLICKED, NULL);

    // autonDown setup
    lv_obj_align(autonDown, LV_ALIGN_CENTER, -15, 80);
    lv_obj_add_style(autonDown, &pushback, LV_PART_MAIN);
    lv_label_set_text(autonDown, LV_SYMBOL_DOWN "\n" LV_SYMBOL_DOWN "\n" LV_SYMBOL_DOWN);
    lv_obj_add_flag(autonDown, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_text_opa(autonDown, 128, LV_STATE_PRESSED);
    lv_obj_set_style_text_font(autonDown, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_line_space(autonDown, -12, LV_PART_MAIN);
    lv_obj_set_style_outline_width(autonDown, 0, LV_PART_MAIN);
    lv_obj_add_event_cb(autonDown, AutonDownEvent, LV_EVENT_CLICKED, NULL);

    // allianceInd setup
    lv_obj_set_size(allianceInd, 51, 51);
    lv_obj_set_pos(allianceInd, 185, 65);
    lv_obj_add_style(allianceInd, &pushback, LV_PART_MAIN);
    lv_obj_set_style_outline_width(allianceInd, 0, LV_PART_MAIN);
    lv_obj_add_flag(allianceInd, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(allianceInd, colorEvent, LV_EVENT_CLICKED, NULL);

    // allianceOverlay setup
    lv_obj_set_size(allianceOverlay, 51, 51);
    lv_obj_set_pos(allianceOverlay, 185, 65);
    lv_img_set_src(allianceOverlay, &colorindOverlay);
    lv_obj_set_style_outline_width(allianceOverlay, 0, LV_PART_MAIN);
    lv_obj_set_style_img_recolor_opa(allianceOverlay, 255, LV_PART_MAIN);
    lv_obj_set_style_img_recolor(allianceOverlay, theme_color, LV_PART_MAIN);
    lv_obj_add_flag(allianceOverlay, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(allianceOverlay, colorEvent, LV_EVENT_CLICKED, NULL);

    // colorInd setup
    lv_obj_set_size(colorInd, 51, 51);
    lv_obj_set_pos(colorInd, 185, 120);
    lv_obj_add_style(colorInd, &pushback, LV_PART_MAIN);
    lv_obj_set_style_outline_width(colorInd, 0, LV_PART_MAIN);
    lv_obj_add_flag(colorInd, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(colorInd, refreshConsoleEvent, LV_EVENT_CLICKED, NULL);

    // colorOverlay setup
    lv_obj_set_size(colorOverlay, 51, 51);
    lv_obj_set_pos(colorOverlay, 185, 120);
    lv_img_set_src(colorOverlay, &colorindOverlay);
    lv_obj_set_style_outline_width(colorOverlay, 0, LV_PART_MAIN);
    lv_obj_set_style_img_recolor_opa(colorOverlay, 255, LV_PART_MAIN);
    lv_obj_set_style_img_recolor(colorOverlay, theme_color, LV_PART_MAIN);
    lv_obj_add_flag(colorOverlay, LV_OBJ_FLAG_CLICKABLE);

    // Move foreground for overlays and indicators
    lv_obj_move_foreground(colorInd);
    lv_obj_move_foreground(allianceInd);
    lv_obj_move_foreground(colorOverlay);
    lv_obj_move_foreground(allianceOverlay);

    // Set color for allianceInd
    colorSet(allianceColor, allianceInd);
}