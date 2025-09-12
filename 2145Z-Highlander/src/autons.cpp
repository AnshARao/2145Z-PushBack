#include "controls.hpp"
#include "drive.hpp"
#include "main.h"
#include "pros/motors.h"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////


///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  set_position(0, 0, 0);
  //set_drive(24.0, 80);
  set_mtp({0, 24}, DRIVE_SPEED);
  wait();

  //set_drive(-12.0, 80);
  set_mtp({0, 12}, DRIVE_SPEED, rev);
  wait();

  //set_drive(-12.0, 80);
  set_mtp({0, 0}, DRIVE_SPEED, rev);
  wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at
  set_position(0, 0, 0);

  set_turn(90);
  wait();

  set_turn(45);
  wait();

  set_turn(0);
  wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  set_position(0, 0, 0);
  
  set_drive(24.0);
  wait(CHAIN);

  set_turn(45);
  wait(CHAIN);

  set_turn(-45);
  wait(CHAIN);

  set_turn(0);
  wait(CHAIN);

  set_drive(-24.0);
  wait(CHAIN);
}

void leftAWP() {
  set_position(-50, 18, 0);
  set_piston(piston_loader, true);
  //move_point({-50, 48}, DRIVE_SPEED);
  set_drive(30.0, 70);
  wait();

  set_turn(270);
  wait(CHAIN);
  set_drive(14.0, 127);
  wait();
  
  set_rollers(INTAKE);
  wait(500, false);

  set_drive(-32.0, DRIVE_SPEED, false, false);
  wait();
  set_rollers(SCORE_TOP);
}

void rightAWP() {
  set_position(-50, -18, 0);

  set_drive(27, DRIVE_SPEED, true);
  set_piston(piston_loader,true);
  set_rollers(INTAKE);
  wait(QUICK);
  set_turn({-58, -45}, fwd, TURN_SPEED);
  wait(QUICK);
  set_drive(9, 127);
  wait(WAIT);
  wait(215);
}

void elimsLeft() {
  set_position(-47, 16, 90);

  set_drive(4.0, DRIVE_SPEED);
  wait(CHAIN);

  set_mtp({-13, 23}, 40, fwd, true);
  set_rollers(INTAKE);
  wait(1000);
  set_piston(piston_loader, true);
  wait();

  set_boom({-5, 43, 5}, DRIVE_SPEED);
  wait(250);
  set_piston(piston_loader, false);
  wait();

  set_piston(piston_loader, true);
  wait(200);

  set_drive(-14.0, DRIVE_SPEED);
  wait(CHAIN);
  //set_piston(piston_loader, false);

  set_mtp({-44, 47}, DRIVE_SPEED);
  wait(CHAIN);
  

  set_turn({-25, 47}, rev, TURN_SPEED);
  wait(CHAIN);

  set_turn(270, TURN_SPEED, ez::shortest);
  wait();

  set_drive(-16, DRIVE_SPEED, false, false);
  wait();

  set_rollers(SCORE_TOP);
  wait(2000);
  set_rollers(INTAKE);

  set_mtp({-60, 47}, 120);
  wait();

  // set_drive(127);
  // wait(750);

  wait(500);
  set_drive(-30.0);
  wait();
  set_piston(piston_loader, false);
  set_rollers(SCORE_TOP);
  wait(500);
}

void elimsRight() {
  set_position(-47, -16, 90);

  set_drive(4.0, DRIVE_SPEED);
  wait(CHAIN);

  set_mtp({-13, -23}, 40, fwd, true);
  set_rollers(INTAKE);
  wait(1000);
  set_piston(piston_loader, true);
  wait();

  set_mtp({-6, -42}, DRIVE_SPEED);
  wait(250);
  set_piston(piston_loader, false);
  wait();

  set_piston(piston_loader, true);
  wait(200);

  set_drive(-13.0, DRIVE_SPEED);
  wait(CHAIN);
  //set_piston(piston_loader, false);

  set_mtp({-44, -47}, DRIVE_SPEED);
  wait(CHAIN);
  

  set_turn({-25, -47}, rev, TURN_SPEED);
  wait(CHAIN);

  set_drive(-16, DRIVE_SPEED, false, false);
  wait();

  set_rollers(SCORE_TOP);
  wait(2000);
  set_rollers(INTAKE);

  set_mtp({-60, -47}, 127);
  wait();

  // set_drive(127);
  // wait(750);

  wait(500);
  set_drive(-30.0);
  wait();
  set_piston(piston_loader, false);
  set_rollers(SCORE_TOP);
  wait(500);

}

void doNothing() {

}

void test() {
  
}

void skills() {
  
}
