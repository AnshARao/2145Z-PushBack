#pragma once

#include "api.h"    // IWYU pragma: keep
#include "pros/adi.hpp"
#include "subsystems.hpp"

inline bool state_loader = false;
inline int vltg_1 = 0;
inline int vltg_2 = 0;
inline int vltg_3 = 0;

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

void set_loader(bool state);
void control_loader();
void set_wing(bool state);
void control_wing();
void set_hood(bool state);