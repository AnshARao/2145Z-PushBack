#pragma once

#include "pros/adi.hpp"
#include "pros/motors.hpp"   // add this line

extern pros::Motor intake1;
extern pros::Motor intake2;
extern pros::adi::DigitalOut hood;
extern pros::adi::DigitalOut stopper;
extern pros::adi::DigitalOut intakeLift;

void intake();
void outtake();
void score();
void scoreMiddle();
void intakeControl();