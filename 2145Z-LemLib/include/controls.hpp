#pragma once
#include "api.h"
#include "subsystems.hpp"

void set_drive_coast();
void set_drive_hold();
double input_scaling(double input, double curve);
void set_tank(int left, int right);
void tank_drive(double scale = 7.0, pros::Controller controller = master);