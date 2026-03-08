#include "main.h"
#include "pros/rtos.hpp"
#include "subsystems/chassis.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/misc.hpp"

void doNothing() {}

void skills() {
    
}

void right7() {
    chassis.setPose(0, 0, 0);
    intake();
    chassis.moveToPoint(8, 23, 2000);
    chassis.waitUntilDone();
    setLoader(true);
    chassis.moveToPoint(40, -3, 2000);
    setLoader(true);
    chassis.turnToHeading(180, 500);
    chassis.moveToPoint(40, -14, 2000);
    chassis.waitUntilDone();
    chassis.moveToPoint(40, 20, 2000, {.forwards = false});
    score();
    pros::delay(2000);
    chassis.moveToPoint(40, 1, 1000);
    chassis.turnToHeading(200, 500);
    chassis.moveToPose(50, 24, 180, 3000, {.forwards = false});
    chassis.setBrakeMode(MOTOR_BRAKE_HOLD);
}