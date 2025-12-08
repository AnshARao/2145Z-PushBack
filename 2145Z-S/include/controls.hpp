#pragma once

#include "api.h"    // IWYU pragma: keep
#include "pros/adi.hpp"
#include "subsystems.hpp"


void intake_t();

void set_motor(pros::Motor& motor, int vltg);

void set_rollers(int vltg1, int vltg2);
void set_rollers(int vltg);
void set_rollers(RollerStates state);

void control_rollers();

void set_piston(pros::adi::DigitalOut& piston, bool state);
void control_piston_toggle(pros::adi::DigitalOut& piston, pros::controller_digital_e_t button);
void control_piston_hold(pros::adi::DigitalOut& piston, pros::controller_digital_e_t button);

void set_tank(double left, double right);
void tank_drive(double curve, pros::Controller& controller);