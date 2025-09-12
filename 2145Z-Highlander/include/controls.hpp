#pragma once

#include "EZ-Template/api.hpp"  // IWYU pragma: keep
#include "api.h"    // IWYU pragma: keep

// declaring global variables
enum MatchStates {DISABLED = 0, AUTO = 1, DRIVER = 2};
inline MatchStates matchState = DISABLED;
enum Alliances {BLUE = 0, NONE = 1, RED = 2};
inline Alliances allianceColor = Alliances::NONE; // for now
enum RollerStates {INTAKE = 0, OUTTAKE = 1, SCORE_TOP = 2, SCORE_MID = 3, STOP = 4};

void set_motor(pros::Motor& motor, int vltg);

void set_rollers(int vltg1, int vltg2);
void set_rollers(int vltg);
void set_rollers(RollerStates state);

void control_rollers();

void set_piston(ez::Piston& piston, bool state);
void control_piston(ez::Piston& piston, pros::controller_digital_e_t button);
