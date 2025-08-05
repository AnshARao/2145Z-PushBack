#pragma once

#include "EZ-Template/api.hpp"  // IWYU pragma: keep
#include "api.h"    // IWYU pragma: keep

// declaring global variables
inline bool doColorSort = true;
inline int sortTime = 0;
enum MatchStates {DISABLED = 0, AUTO_PID = 1, AUTO_ODOM = 2, DRIVER = 3};
inline MatchStates matchState = DISABLED;
enum Alliances {BLUE = 0, NONE = 1, RED = 2};
inline Alliances allianceColor = Alliances::RED; // for now
inline bool override = false;
enum HopperControl {TOP = 0, BOTTOM = 1};
inline HopperControl currentHopper = TOP;

inline bool ctrlLock = false;

inline int vltg_front = 0;
inline int vltg_top   = 0;
inline int vltg_back  = 0;

enum Rollers {STOP = 0, INTAKE = 1, HOPPER_BOTTOM = 2, HOPPER_TOP = 3, OUTTAKE = 4, SCORE_BOT = 5, SCORE_MID = 6, SCORE_TOP = 7};
inline Rollers curRoller = STOP;

inline bool statePto =     false;
inline bool stateLoader =  false;
inline bool stateBlocker_top = false;
inline bool stateBlocker_bot = true;
inline bool stateHopper =  false;
inline bool stateHood =    false;
inline bool statePuncherMid = false;
inline bool statepuncherTop = false;

void set_roller_front(int vltg);
void set_roller_top(int vltg);
void set_roller_back(int vltg);
void set_rollers(int front, int top, int back);
void set_rollers(int main, int top);
void set_rollers(int vltg);
void set_rollers(Rollers situation);

void set_pto(bool state);
void set_loader(bool state);
void set_puncher_mid(bool state);
void set_puncher_top(bool state);

void control_rollers();
void control_pto();
void control_loader();
void control_punchers();

Alliances get_color_top();
Alliances get_color_mid();
Alliances get_color_bot();

bool rightBlockDetected(Alliances blockColor);
bool wrongBlockDetected(Alliances blockColor);

void colorSortLoop();
void load_until(int blocks);
void score_until(Rollers situation, int blocks);

void roller_t();
void colorSort_t();
void misc_t();

inline pros::Task RollerTask(roller_t);
inline pros::Task MiscTask(misc_t);
inline pros::Task ColorSortTask(colorSort_t);
