#pragma once
#include "pros/motors.hpp"
#include "pros/optical.hpp"
extern pros::Motor intake;
extern pros::Optical optical;

extern bool isRed;
extern bool wrongRing;

void intakeSpin(int speed);
void intakeControl();
void colorSortLoop();
void sortOutRing();
void colorSort_t();