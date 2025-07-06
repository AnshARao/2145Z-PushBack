#pragma once
#include "liblvgl/lvgl.h"

// ** @file controls.hpp
// ** @brief This file contains the function headers for the robot's controls.
// ** @details This includes the driver and autonomous controls, as well as the tasks connecting both
// ** @author Ansh Rao - 2145Z

// declaring global variables
enum MatchStates {DISABLED = 0, AUTO_PID = 1, AUTO_ODOM = 2, DRIVER = 3};
inline MatchStates matchState = DISABLED;
enum Alliances {BLUE = 0, NONE = 1, RED = 2};
inline Alliances allianceColor = Alliances::NONE;

inline bool ctrlLock = false;

// declaring global functions
void default_constants();
void initAll();

// declaring roller variables
inline int roller_front_vltg = 0;
inline int roller_top_vltg = 0;
inline int roller_back_vltg = 0;

// declaring roller functions
void set_rollers(int vltg);
void set_rollers(int frontAndBackVltg, int topVltg);
void set_rollers(int frontVltg, int topVltg, int backVltg);
void control_roller();
void set_roller_front(int vltg);
void set_roller_top(int vltg);
void set_roller_back(int vltg);
void set_rollers_intake();
void set_rollers_outtake();
void set_rollers_score_middle();
void set_rollers_score_top();
void roller_t();

//declaring color sort variables
inline bool doColorSort = true;
inline int colorSortTime = 0;
inline const int MAXCOLORSORTTIME = 1000;  // 1 second
inline bool doIntakeUntilTop = false;
inline int intakeUntilTime = 0;  // Time to wait until the top block is detected
inline const int MAXINTAKEUNTILTIME = 1000;  // 1 second

// declaring color sort functions
Alliances getTopBlockColor();
Alliances getBottomBlockColor();
bool rightBlockDetected(Alliances blockColor);
bool wrongBlockDetected(Alliances blockColor);
void colorSortLoop();
void colorSort_t();
void colorSet(Alliances color, lv_obj_t* object);

// declaring misc variables
inline bool loaderState = false;
inline bool hoodState = false;
inline bool ptoState = false;
inline bool trapdoorState = false;

// declaring misc functions

void setPto(bool state);
void setPtoRpm(bool state);
void control_pto();

void setTrapdoor(bool state);
void control_trapdoor();

void setLoader(bool state);
void control_loader();

void setHood(bool state);
void control_hood();

void misc_t();


enum WaitTypes {WAIT_NONE = 0, WAIT_NORMAL = 1, WAIT_QUICK = 2, WAIT_CHAIN = 3};

