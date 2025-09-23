#include <cmath>
#include "EZ-Template/util.hpp"
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

void leftQual() {
  set_position(-47, 16, 90);

  set_drive(4.0, 125);
  wait(CHAIN);

  set_mtp({-13, 23.5}, 75, fwd, true);
  set_rollers(INTAKE);
  wait(650);
  set_piston(piston_loader, true);
  wait();

  set_mtp({-6, 42}, DRIVE_SPEED);
  wait(325);
  set_piston(piston_loader, false);
  wait();

  set_rollers(-4000);
  set_piston(piston_loader, true);
  wait(200);

  set_mtp({-24, 24}, DRIVE_SPEED, rev);
  wait();

  //5303591943

  set_boom({-12.5, 15.5, 135}, DRIVE_SPEED, rev);
  wait();
  set_piston(piston_scorer, false);
  wait();

  set_drive(-6.0);
  wait();
  set_rollers(SCORE_MID);
  wait(2000);
  set_drive(7.0);
  set_rollers(INTAKE);
  wait(CHAIN);

  set_boom({-60, 46, 270}, DRIVE_SPEED, fwd, true);
  wait(400);
  set_piston(piston_loader, true);
  wait();
  wait(500);
  set_mtp({-26, 46}, DRIVE_SPEED, rev);
  wait();
  set_rollers(SCORE_TOP);
  wait(3000);
  set_drive(20);

  set_piston(piston_loader, false);  
}

void leftAWP() {
  set_position(-47, 16, 90);

  set_drive(4.0, DRIVE_SPEED);
  wait(CHAIN);

  set_mtp({-20, 23}, 40, fwd, true);
  set_rollers(INTAKE);
  wait(1000);
  set_piston(piston_loader, true);
  wait();

  set_boom({-12.5, 15.5, 135}, DRIVE_SPEED, rev);
  set_rollers(1000);
  set_piston(piston_loader, false);
  wait(1000);
  set_piston(piston_scorer, false);
  wait();

  set_drive(-4.0);
  wait();
  set_rollers(SCORE_MID);
  wait(1250);
  set_rollers(INTAKE);
  set_drive(4.0);
  wait(CHAIN);

  set_boom({-60, 46, 270}, DRIVE_SPEED);
  wait(400);
  set_piston(piston_loader, true);
  wait();
  wait(500);
  set_mtp({-26, 46}, DRIVE_SPEED, rev);
  wait();
  set_rollers(SCORE_TOP);
  wait(3000);
  set_drive(20);

  set_piston(piston_loader, false);
}

void rightAWP() {
  set_position(-47, -16, 90);

  set_drive(4.0, DRIVE_SPEED);
  wait(CHAIN);

  set_mtp({-20, -23}, 40, fwd, true);
  set_rollers(INTAKE);
  wait(1000);
  set_piston(piston_loader, true);
  wait();

  set_boom({-10.5, -13.5, 45}, DRIVE_SPEED);
  wait(100);
  set_piston(piston_loader, false);
  wait();
  set_rollers(OUTTAKE);
  wait(1250);
  set_rollers(INTAKE);
  set_drive(-3.0);
  wait(CHAIN);

  set_boom({-60, -46, 270}, DRIVE_SPEED);
  wait(400);
  set_piston(piston_loader, true);
  wait();
  wait(500);
  set_mtp({-26, -46}, DRIVE_SPEED, rev);
  wait();
  set_rollers(SCORE_TOP);
  wait(3000);
  set_drive(20);

  set_piston(piston_loader, false);
}

void left9() {
  set_position(-47, 16, 90);

  set_drive(4.0, 125);
  wait(CHAIN);

  set_mtp({-13, 23.5}, 75, fwd, true);
  set_rollers(INTAKE);
  wait(650);
  set_piston(piston_loader, true);
  wait();

  set_mtp({-6, 42}, DRIVE_SPEED);
  wait(325);
  set_piston(piston_loader, false);
  wait();

  set_piston(piston_loader, true);
  wait(200);

  set_drive(-15.0, DRIVE_SPEED);
  wait(QUICK);
  //set_piston(piston_loader, false);

  set_mtp({-44, 47}, 75, fwd, true);
  wait(CHAIN);
  
  set_turn({-25, 47}, rev, TURN_SPEED);
  wait(CHAIN);

  //set_drive(-16, DRIVE_SPEED, false, false);
  set_boom({-26, 47, 270}, 125, rev);
  wait();
  set_rollers(SCORE_TOP);
  wait(2000);
  set_rollers(INTAKE);

  set_boom({-60.5, 47, 270}, 120);
  wait(400);
  set_piston(piston_loader, true);
  wait();
  wait(500);
  set_mtp({-26, 47}, 125, rev);
  wait();
  set_rollers(SCORE_TOP);
  wait(2500);
  set_drive(4.0);
  wait();
  set_drive(-127);
  set_piston(piston_loader, false);
}

void right9() {
set_position(-47, -16, 90);

  set_drive(4.0, 125);
  wait(CHAIN);

  set_mtp({-13, -23.5}, 75, fwd, true);
  set_rollers(INTAKE);
  wait(650);
  set_piston(piston_loader, true);
  wait();

  set_mtp({-6, -42}, DRIVE_SPEED);
  wait(325);
  set_piston(piston_loader, false);
  wait();

  set_piston(piston_loader, true);
  wait(200);

  set_drive(-15.0, DRIVE_SPEED);
  wait(QUICK);
  //set_piston(piston_loader, false);

  set_mtp({-44, -47}, 75, fwd, true);
  wait(CHAIN);
  
  set_turn({-25, -47}, rev, TURN_SPEED);
  wait(CHAIN);

  //set_drive(-16, DRIVE_SPEED, false, false);
  set_boom({-26, -47, 90}, 125, rev);
  wait();
  set_rollers(SCORE_TOP);
  wait(2000);
  set_rollers(INTAKE);

  set_boom({-60.5, -47, 270}, 120);
  wait(400);
  set_piston(piston_loader, true);
  wait();
  wait(500);
  set_mtp({-26, -47}, 125, rev);
  wait();
  set_rollers(SCORE_TOP);
  wait(2500);
    set_drive(4.0);
  wait();
  set_drive(-127);
  set_piston(piston_loader, false);
}

void doNothing() {

}

void test() {
  
}

void skills() {
  set_position(-50, -17, 180);

  set_piston(piston_loader, true);
  set_mtp({-50, -47.5}, 50);
  set_rollers(INTAKE);
  wait();

  set_turn({-60, -48}, fwd, 100);
  wait();

  //set_mtp({-60.5, -47.5}, 125);
  set_drive(11.0, 125);
  wait();
  wait(2000);

  set_mtp({-26, -47.5}, DRIVE_SPEED, rev);
  wait();
  set_rollers(SCORE_TOP);
  wait(5000);

  set_drive(10.0);
  wait(CHAIN);

  set_boom({-63, -22, 355},  DRIVE_SPEED);
  set_piston(piston_loader, false);
  wait(QUICK);

  set_drive(127);
  wait(1250);
  set_drive(-20);
  wait(100);
  wait();
}

void skillsLong() {
    set_position(-50, -17, 180);

  set_mtp({-50, -47}, 50);
  set_piston(piston_loader, true);
  set_rollers(INTAKE);
  wait();

  set_turn({-60, -47}, fwd, TURN_SPEED);
  wait();

  set_boom({-60.5, -47, 270}, 125);
  //set_drive(11.0);
  wait();
  wait(1500);

  set_mtp({-26, -47}, DRIVE_SPEED, rev);
  wait();
  set_rollers(SCORE_TOP);
  wait(5000);

  set_drive(14.0);
  wait();

  

  set_boom({-40, -30, 90}, DRIVE_SPEED);
  set_piston(piston_loader, false);
  wait(QUICK);

  set_boom({50, -30, 135}, DRIVE_SPEED);
  wait();

  set_boom({60.5, -47, 90}, DRIVE_SPEED);
  set_piston(piston_loader, true);
  wait();
  wait(1000);

  set_mtp({26, -47}, DRIVE_SPEED, rev);
  wait();
  set_rollers(SCORE_TOP);
  wait(5000);

  set_piston(piston_loader, false);

  set_drive(10.0);
  wait(CHAIN);

  set_boom({60, -20, 0}, DRIVE_SPEED);
  wait(CHAIN);
  set_drive(20.0);
  wait(); 
}
