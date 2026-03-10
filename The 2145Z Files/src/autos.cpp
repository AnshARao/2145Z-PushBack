#include "main.h"
#include "pros/rtos.hpp"
#include "subsystems/chassis.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/misc.hpp"

void doNothing() {
    set_position(0, 0, 0);
    set_drive(0.00001, 10000.0);
}

void skills() {
    
}

void test1() {
    set_position(0, 0, 0);

}

void sawp() {
    set_position(-48, -12, 0);
    resetLeft();
    resetBack();
    intake();
    set_drive(4.0, 2000);
    set_point(-48.0, -48.0, 2000, {.forwards = false});
    setLoader(true);
    set_turn(270.0, 2000);
    set_point(-60.0, -48.0, 2000);
    wait(300);
    set_drive(-30.0, 2000);
    score();
    wait(500);
    resetFront();
    resetLeft();
    setLoader(false);
    set_drive(4.0, 1000);
    set_turn(30.0, 2000);
    resetBack();
    intake();
    set_point(-24.0, -24.0, 2000, {.forwards = true, .maxSpeed = 90});
    resetLeft();
    set_point(-24.0, 24.0, 2000, {.forwards = true, .maxSpeed = 90});
    setLoader(true);
    resetLeft();
    set_turn(-45.0, 2000);
    set_drive(24.0, 2000);
    set_pose(-48.0, 48.0, 270.0, 2000);
    set_drive(-30.0, 2000);
    score();
}

void right7() {

}