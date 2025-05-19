#pragma once

// ** @file controls.hpp
// ** @brief This file contains the function headers for the robot's controls.
// ** @details This includes the driver and autonomous controls, as well as the tasks connecting both
// ** @author Ansh Rao - 2145Z

// declaring global variables
inline bool isAuto = false;
inline bool isRed = true;
inline bool isPto = false;

// declaring global functions
void default_constants();\
void initAll();

// declaring intake variables
inline int intake_vltg = 0;

// declaring intake functions
void set_intake(int vltg);
void control_intake();
void intake_t();

// declaring rollers variables
inline int rollers_vltg = 0;

// declaring rollers functions
void set_rollers(int vltg);
void control_rollers();
void rollers_t();

