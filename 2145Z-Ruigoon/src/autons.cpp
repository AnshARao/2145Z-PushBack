#include "autons.hpp"
#include <cmath>
#include "EZ-Template/util.hpp"
#include "controls.hpp"
#include "drive.hpp"
#include "main.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

void botb1() {
  chassis.pid_turn_set(15, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(14.0, DRIVE_SPEED);
  set_rollers(INTAKE);
  chassis.pid_wait();
  set_piston(piston_loader, true);

  chassis.pid_turn_set(-50, TURN_SPEED);
  pros::delay(100);
  set_piston(piston_loader, false);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(8.5, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  set_rollers(-80);
  chassis.pid_drive_set(-1, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(1, 127);
  chassis.pid_wait_quick_chain();
}

void botb2() {
  chassis.pid_turn_set(15, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(14.0, DRIVE_SPEED);
  set_rollers(INTAKE);
  chassis.pid_wait_quick_chain();
  set_piston(piston_loader, true);

  chassis.pid_turn_set(-50, TURN_SPEED);
  pros::delay(150);
  set_piston(piston_loader, false);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(8.5, 127);
  set_rollers(-85);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-1, 127);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-26, 127);
  set_rollers(INTAKE);
  chassis.pid_wait();
  set_piston(piston_loader, true);
  chassis.pid_turn_set(180, 127);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(12, DRIVE_SPEED);
  chassis.pid_wait_quick();
  pros::delay(300);
  chassis.pid_drive_set(-27, 127);
  chassis.pid_wait_quick_chain();
  set_rollers(SCORE);
  pros::delay(750);
  chassis.pid_turn_set(90, TURN_SPEED);
  set_piston(piston_loader, false);
  chassis.pid_wait();
  chassis.pid_drive_set(14, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(175, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-12, 127);
}


void testing() {
  set_position(0,0,0);

  set_mtp({-12, 36}, DRIVE_SPEED);
  wait();

  set_mtp({24, 24}, DRIVE_SPEED);
  wait();

  set_boom({0,0, 0}, DRIVE_SPEED);
  wait();
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  set_position(0, 0, 0);
  set_mtp({0, 24}, DRIVE_SPEED);
  wait();

  set_mtp({0, 12}, DRIVE_SPEED, rev);
  wait();

  set_mtp({0, 0}, DRIVE_SPEED, rev);
  wait();

  // set_drive(24.0);
  // wait();

  // set_drive(-12.0);
  // wait();

  // set_drive(-12.0);
  // wait();
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

void SAWP13() {

  int loadSpeed = 75;
  int unjamTime = 75;

  set_position(-45, -12.5, 180);

  set_drive(33.5, 127);
  wait(CHAIN);
  set_piston(piston_loader, true);
  set_turn(270);
  wait(QUICK);
  set_drive(14.0, loadSpeed);
  set_rollers(INTAKE);
  wait(CHAIN);

  // set_boom({-58, -48, 270}, DRIVE_SPEED);
  // set_rollers(INTAKE);
  // wait();

  set_drive(-30.0, 127);
  wait(350);
  set_rollers(OUTTAKE);
  wait(unjamTime);
  set_rollers(SCORE);
  wait();

  wait(900);
  set_piston(piston_loader, false);

  set_turn(10, 127);
  wait();
  set_drive(15.0, 127);
  wait(400);
  set_piston(piston_loader, true);
  set_rollers(INTAKE);
  wait(CHAIN);

  set_turn(-1, DRIVE_SPEED);
  wait(CHAIN);
  set_drive(44.0, 127);
  set_piston(piston_loader, false);
  wait(1050);
  set_piston(piston_loader, true);
  wait(CHAIN);

  set_boom({-19.25, 5.5, 314.5}, 127, rev);
  set_rollers(STOP);
  set_piston(piston_scorer, true);
  wait(CHAIN);

  set_rollers(-2000);
  set_drive(-1.5, 127);
  set_rollers(SCORE);
  wait(CHAIN);
  wait(800);

  set_drive(51.5, 127);
  set_rollers(INTAKE);
  set_piston(piston_loader, false);
  wait(CHAIN);
  set_piston(piston_loader, true);
  set_turn(269);
  wait(CHAIN);
  set_drive(15.0, loadSpeed);
  wait(CHAIN);
  set_drive(-29.0, 127);
  wait(350);
  set_rollers(OUTTAKE);
  wait(unjamTime);
  set_rollers(SCORE);
  wait(CHAIN);
  set_piston(piston_loader, false);

}

void SAWP13Old() {
  set_position(-48, -13, 180);

  set_piston(piston_loader, true);
  set_mtp({-50, -46}, 80, fwd, true);
  //set_drive(28.0, DRIVE_SPEED, true);
  set_rollers(INTAKE);
  wait(QUICK);

  set_turn({-59, -48}, fwd, 100);
  wait();

  set_mtp({-59, -48}, 100);
  wait();
  wait(650);

  set_mtp({-28, -48}, DRIVE_SPEED, rev);
  wait();
  set_piston(piston_loader, false);
  set_rollers(SCORE);
  wait(1750);
  set_rollers(INTAKE);

  set_turn(7, 127);
  wait();
  set_drive(14.0, 127);
  wait(200);
  set_piston(piston_loader, true);
  wait(CHAIN);
  set_mtp({-28, 12}, 127);
  wait(200);
  set_piston(piston_loader, false);
  wait(CHAIN);
  set_drive(14.0);
  wait(QUICK);
  set_piston(piston_loader, true);
  set_rollers(STOP);

  set_boom({-15, 9, get_theta(currentPoint, {-15, 9}, ez::drive_directions::rev)}, 127, rev);
  set_piston(piston_scorer, true);
  wait();
  set_rollers(SCORE);
  wait(1500);

  set_mtp({-50, 42}, 127);
  wait(100);
  set_rollers(INTAKE);
  wait(CHAIN);
  set_turn({-64, 42}, fwd, DRIVE_SPEED);
  wait(QUICK);
  set_drive(15.0);
  wait();
  wait(600);
  set_mtp({-32, 44}, DRIVE_SPEED, rev);
  wait(QUICK);
  set_rollers(SCORE);
  wait(2000);

}

void SAWP15() {

}


void doNothing() {

}

void test() {
  
}


void SAWP() {
  set_position(-50, -17, 180);

  set_piston(piston_loader, true);
  set_mtp({-50, -46}, 80, fwd, true);
  //set_drive(28.0, DRIVE_SPEED, true);
  set_rollers(INTAKE);
  wait(QUICK);

  set_mtp({-26, -47.5}, 70, rev);
  wait();
  set_rollers(SCORE);
  wait(2500);
  set_rollers(INTAKE);

  set_piston(piston_loader, false);
  set_swing(ez::LEFT_SWING, 20, 100, 10, cw);
  wait(CHAIN);

  set_mtp({-22, -24}, 70);
  wait(QUICK);
  set_piston(piston_loader, true);
  
  set_mtp({-22, 24}, 70, fwd, true);
  wait(250);
  set_piston(piston_loader, false);
  wait(QUICK);
  set_rollers(OUTTAKE);
  set_piston(piston_loader, true);

  set_boom({-14, 16, 315}, DRIVE_SPEED, rev);
  wait();
  set_piston(piston_scorer, true);

  set_drive(-6.0);
  wait();
  set_rollers(SCORE);
  wait(2000);
  set_rollers(OUTTAKE);
  set_drive(7.0);
  wait();
  set_rollers(INTAKE);

  set_boom({-54, 46, 270}, DRIVE_SPEED);
  wait();

  set_turn(270, TURN_SPEED);
  wait();

  set_drive(127);
  wait(150);
  wait();
  wait(750);

  set_drive(-38.0);
  wait();
  set_rollers(SCORE);
  set_piston(piston_loader, false);

}

void sixThreeLeft() {
  set_position(-47, 16, 90);
  set_piston(piston_loader, false);
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
  wait(CHAIN);
  set_rollers(SCORE);
  wait(2000);
  set_rollers(INTAKE);

  set_boom({-60.5, 47, 270}, 127);
  set_piston(piston_loader, true);
  wait();
  set_drive(-1.0);
  wait();
  set_drive(1.5, 127);
  wait();
  set_mtp({-26, 47}, 127, rev);
  wait();
}

void sixThreeRight() {
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

  set_mtp({-44, -48}, DRIVE_SPEED, fwd, true);
  wait(CHAIN);
  
  set_turn({-25, -48}, rev, TURN_SPEED);
  wait(CHAIN);

  //set_drive(-16, DRIVE_SPEED, false, false);
  set_boom({-26, -48, 90}, DRIVE_SPEED, rev);
  wait();
  set_rollers(OUTTAKE);
  wait(100);
  set_rollers(SCORE);
  wait(2000);
  set_rollers(INTAKE);

  set_boom({-60.15, -48, 270}, 125);
  set_piston(piston_loader, true);
  wait(CHAIN);
  // set_drive(-0.25);
  // wait(CHAIN);
  set_mtp({-26, -48}, 125, rev);
  wait(CHAIN);
  set_piston(piston_loader, false);

  set_turn(0, 127);
  wait(QUICK);
  set_drive(14.0, 127);
  wait(QUICK);
  set_turn(50);
  wait(QUICK);
  set_drive(16.0, 127);
  wait(QUICK);
  set_rollers(-12000);
  wait(100);
  set_rollers(INTAKE);
  wait(100);
  set_rollers(-9750);
  wait(1750);
  set_drive(1.0);
  wait();
}

void fourFiveLeft() {
  // sets on brain sreen. Lowk needed
  //(x, y, angle)
  set_position(-47, 16, 90);

  //(distance in inches *MAKE SURE ITS A DECIMAL*, speed)
  set_drive(4.0, 125); //add decimal for distance
  wait(CHAIN); //wait (QUICK or CHAIN) (chain is starting next movement before current movement is finished)

  set_mtp({-13, 23.5}, 75, fwd, true); //Set move to point (Moves to cordinate, speed , fwd or rev, only true when the movement is bigger (long distance))
  //slew makes it slow down at the end of movement
  set_rollers(INTAKE); //whenever i want intake to move put this in
  //Ex - set_rollers(Outtake); - makes rollers outtake
  //Ex - set_rollers(Stop); - makes rollers stop
  //Ex - set_rollers(Intake); - makes rollers intake
  //Ex - set_rollers(SCORE); - makes rollers score at mid speed
  //Ex - set_rollers(SCORE); - makes rollers score at top speed
  wait(650);
  set_piston(piston_loader, true); //sets piston to true (out) (lilwill)
  wait(); //waits until movement is done

  set_mtp({-5, 41}, DRIVE_SPEED); //moves to point at drive speed
  set_piston(piston_loader, false);
  wait(); //waits until movement is done
  set_piston(piston_loader, true);
  set_mtp({-20, 20}, DRIVE_SPEED, rev);
  wait(200);
  wait();

  set_boom({-9.5, 10, 315}, DRIVE_SPEED, rev);
  set_rollers(STOP);
  set_piston(piston_scorer, true);
  wait(CHAIN);
  set_drive(-1.0);
  wait(CHAIN);
  set_rollers(OUTTAKE);
  wait(100);
  set_rollers(SCORE); 
  wait(700);
  set_boom({-50, 40, 269}, DRIVE_SPEED);
  set_rollers(OUTTAKE);
  wait(150);
  set_rollers(INTAKE);
  wait();
  set_drive(16.0, 60);
  wait(CHAIN);
  set_rollers(6000, -12000);
  set_turn(270);
  set_drive(-30.0);
  wait(CHAIN);
  set_rollers(SCORE);
  wait(75);
  set_rollers(OUTTAKE);
  wait(100);
  set_rollers(SCORE);
  wait(75);
  set_rollers(OUTTAKE);
  wait(100);
  set_rollers(SCORE);
  wait(1250);
  set_piston(piston_loader, false);
  set_turn(180, 127);
  wait();
  set_drive(5.0, 127);
  wait();
  set_turn(257, 90);
  wait();
  set_drive(-14.0, 70, false, false);
  wait(CHAIN);
  set_turn(270);
  wait(CHAIN);
}

void fourFiveRight() {
  // sets on brain sreen. Lowk needed
  //(x, y, angle)
  set_position(-47, -16, 90);

  //(distance in inches *MAKE SURE ITS A DECIMAL*, speed)
  set_drive(4.0, 125); //add decimal for distance
  wait(CHAIN); //wait (QUICK or CHAIN) (chain is starting next movement before current movement is finished)

  set_mtp({-13, -23.5}, 125, fwd, true); //Set move to point (Moves to cordinate, speed , fwd or rev, only true when the movement is bigger (long distance))
  //slew makes it slow down at the end of movement
  set_rollers(INTAKE); //whenever i want intake to move put this in
  //Ex - set_rollers(Outtake); - makes rollers outtake
  //Ex - set_rollers(Stop); - makes rollers stop
  //Ex - set_rollers(Intake); - makes rollers intake
  //Ex - set_rollers(SCORE); - makes rollers score at mid speed
  //Ex - set_rollers(SCORE); - makes rollers score at top speed
  wait(500);
  set_piston(piston_loader, true); //sets piston to true (out) (lilwill)
  wait(); //waits until movement is done

  set_mtp({-5, -41}, DRIVE_SPEED); //moves to point at drive speed
  set_piston(piston_loader, false);
  wait(QUICK); //waits until movement is done
  set_piston(piston_loader, true);
  set_mtp({-20, -20}, DRIVE_SPEED, rev);
  wait(200);
  wait();

  set_boom({-11.75, -12, 46}, DRIVE_SPEED, fwd);
  set_piston(piston_loader, false);
  set_rollers(STOP);
  wait(CHAIN);
  set_drive(1.0);
  set_rollers(INTAKE);
  wait(CHAIN);
  set_rollers(OUTTAKE); 
  wait(1250);
  set_drive(-12.0);
  wait(CHAIN);
  set_piston(piston_loader, true);
  set_boom({-50, -42, 270}, DRIVE_SPEED);
  set_rollers(INTAKE);
  wait();
  set_drive(10.15);
  wait(QUICK);
  
  set_drive(-30.0);
  wait(CHAIN);
  set_rollers(SCORE);
  wait(1750);
  set_piston(piston_loader, false);
  set_turn(180, 127);
  wait();
  set_drive(5.0, 127);
  wait();
  set_turn(260, 127);
  wait();
  set_drive(-18.0, 75);
  wait();
}

void fourFiveRightOld() {
  // sets on brain sreen. Lowk needed
  //(x, y, angle)
  set_position(-47, -16, 90);

  //(distance in inches *MAKE SURE ITS A DECIMAL*, speed)
  set_drive(4.0, 127); //add decimal for distance
  wait(CHAIN); //wait (QUICK or CHAIN) (chain is starting next movement before current movement is finished)

  set_mtp({-18, -23.5}, 90, fwd, true); //Set move to point (Moves to cordinate, speed , fwd or rev, only true when the movement is bigger (long distance))
  //slew makes it slow down at the end of movement
  set_rollers(INTAKE); //whenever i want intake to move put this in

  wait(650);//wait in miliseconds
  set_piston(piston_loader, true); //sets piston to true (out) (lilwill)
  wait(); //waits until movement is done

  set_turn({-9,-7}, fwd, TURN_SPEED);
  wait();

  set_drive(12.0, 127);
  set_piston(piston_loader, false);
  wait();
  set_rollers(OUTTAKE);
  for (int i = 0; i < 3; i++) {
    wait(100);
    set_rollers(INTAKE);
    wait(100);
    set_rollers(OUTTAKE);
    wait(500);
  }
  
  set_drive(-3.0, 127);
  wait(CHAIN);

  set_mtp({-5, -40}, DRIVE_SPEED); //moves to point at drive speed
  set_rollers(INTAKE);
  wait(); //waits until movement is done

  set_piston(piston_loader, true);
  wait(200);
  set_drive(-14.0, 127);
  wait(CHAIN);

  set_mtp({-48, -46}, 127);
  wait(CHAIN);
  set_turn(270);
  wait();
  set_drive(12.0);
  wait(CHAIN);

  set_drive(-0.5);
  wait(CHAIN);


  set_drive(-30.0, 127);
  wait();
  set_rollers(SCORE);
  set_piston(piston_loader, false);
  wait(2000);
  set_turn(180, 127);
  wait();
  set_drive(3.0, 127);
  wait();
  set_turn(260, 127);
  wait();
  set_drive(-18.0);
  wait();
}

void left7() {
  set_position(-47, 16, 90);
  set_piston(piston_loader, false);
  set_drive(4.0, 125);
  wait(CHAIN);

  set_mtp({-13, 23.5}, 75, fwd, true);
  set_rollers(INTAKE);
  wait(650);
  set_piston(piston_loader, true);
  wait();

  set_drive(-15.0, DRIVE_SPEED);
  wait(QUICK);
  //set_piston(piston_loader, false);

  set_mtp({-44, 47}, 75, fwd, true);
  wait(CHAIN);
  
  set_turn(270);
  wait(CHAIN);

  //set_drive(-16, DRIVE_SPEED, false, false);
  // set_boom({-26, 47, 270}, 125, rev);
  // wait(CHAIN);
  // set_rollers(SCORE);
  // wait(2000);
  // set_rollers(INTAKE);

  // set_boom({-60.5, 47, 270}, 127);
  set_piston(piston_loader, true);
  set_drive(22.0);
  wait();
  // set_drive(-1.0);
  // wait();
  // set_drive(1.5, 127);
  // wait();
  set_drive(-30.0, 127);
  wait(CHAIN);
  set_rollers(SCORE);
  wait(250);
  set_rollers(OUTTAKE);
  wait(100);
  wait(1000);

  set_turn(180);
  wait();
  set_drive(3.0);
  wait();
  set_turn(260);
  wait();
  set_drive(-20.0);
  wait();
  
  if (matchState != AUTO) set_piston(piston_loader, false);
}


void left9() {
  set_position(-47, 16, 90);
  set_piston(piston_loader, false);
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
  wait(CHAIN);
  set_rollers(SCORE);
  wait(2000);
  set_rollers(INTAKE);

  set_boom({-60.5, 47, 270}, 127);
  set_piston(piston_loader, true);
  wait();
  set_drive(-1.0);
  wait();
  set_drive(1.5, 127);
  wait();
  set_mtp({-26, 47}, 127, rev);
  wait();
  set_rollers(SCORE);
  wait(1500);

  set_turn(180);
  wait();
  set_drive(3.0);
  wait();
  set_turn(260);
  wait();
  set_drive(-18.0);
  wait();
  
  if (matchState != AUTO) set_piston(piston_loader, false);
}

void right7() {
  set_position(-47, -16, 90);

  set_drive(4.0, 127);
  wait(CHAIN);

  set_mtp({-13, -23.5}, 75, fwd, true);
  set_rollers(INTAKE);
  wait(500);
  set_piston(piston_loader, true);
  wait(QUICK);

  set_drive(-15.0, 127);
  wait(CHAIN);
  //set_piston(piston_loader, false);

  set_mtp({-44, -47}, DRIVE_SPEED, fwd, true);
  wait();
  
  // set_turn({-25, -47}, rev, TURN_SPEED);
  // wait(CHAIN);

  // //set_drive(-16, DRIVE_SPEED, false, false);
  // set_boom({-26, -47, 90}, 125, rev);
  // wait();
  // set_rollers(SCORE);
  // wait(2000);
  set_rollers(INTAKE);
  set_turn(270);
  wait(CHAIN);

  // set_boom({-60.5, -47, 270}, 125);
  set_drive(20.0, 60);
  set_piston(piston_loader, true);
  wait();
  // set_drive(1.0, 127);
  // wait(QUICK);
  // set_mtp({-26, -47}, 127, rev);
  set_drive(-29.0, 127);
  wait(400);
  set_rollers(OUTTAKE);
  wait(100);
  set_rollers(SCORE);
  wait(CHAIN);
  wait(1700);
  set_piston(piston_loader, false);

  set_turn(180);
  wait();
  set_drive(5.0, 127);
  wait();
  set_turn(260);
  wait();
  set_drive(-18.0);
  wait();
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
}

void right9() {
  set_position(-47, -16, 90);

  set_drive(4.0, 125);
  wait(CHAIN);

  set_mtp({-13, -23.5}, 75, fwd, true);
  set_rollers(INTAKE);
  wait(500);
  set_piston(piston_loader, true);
  wait();

  set_mtp({-6, -42}, DRIVE_SPEED);
  wait(325);
  set_piston(piston_loader, false);
  wait();

  set_piston(piston_loader, true);
  wait(200);

  set_drive(-17.0, DRIVE_SPEED);
  wait(QUICK);
  //set_piston(piston_loader, false);

  set_mtp({-44, -47}, 75, fwd, true);
  wait(CHAIN);
  
  set_turn({-25, -48}, rev, TURN_SPEED);
  wait(CHAIN);

  //set_drive(-16, DRIVE_SPEED, false, false);
  set_boom({-26, -48, 90}, 125, rev);
  wait();
  set_turn(90);
  wait(CHAIN);
  set_rollers(SCORE);
  wait(2000);
  set_rollers(INTAKE);

  set_boom({-60.5, -48, 270}, 125);
  set_piston(piston_loader, true);
  wait();
  set_drive(1.5, 127);
  wait(QUICK);
  set_mtp({-26, -48}, 125, rev);
  wait();
  set_rollers(OUTTAKE);
  wait(100);
  set_rollers(SCORE);
  wait(2000);
  set_piston(piston_loader, false);
  //set_piston(piston_scorer, true);

  // set_turn(180);
  // wait();
  // set_drive(5.0);
  // wait();
  // set_turn(260);
  // wait();
  // set_drive(-18.0);
  // wait();
}

void skillsOld() {
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
  set_rollers(SCORE);
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
  set_rollers(SCORE);
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
  set_rollers(SCORE);
  wait(5000);

  set_piston(piston_loader, false);

  set_drive(10.0);
  wait(CHAIN);

  set_boom({60, -20, 0}, DRIVE_SPEED);
  wait(CHAIN);
  set_drive(20.0);
  wait(); 
}


void skills() {

  int loadSpeed = 50;
  int unjamTime = 100;

  set_position(-48, -12.5, 180);

  set_drive(34.0, DRIVE_SPEED, true);
  wait();
  set_piston(piston_loader, true);
  set_turn(270);
  wait();
  set_drive(17.25, loadSpeed);
  set_rollers(INTAKE);
  wait();
  set_drive(-1.0);
  wait();
  set_drive(1.5,127);
  wait();
  wait(1500);

  set_drive(-12.0);
  wait(QUICK);
  set_turn(135);
  wait();

  set_drive(20.0);
  set_piston(piston_loader, false);
  wait();
  set_turn(90);
  wait();
  set_drive(70.0, DRIVE_SPEED, true);
  wait();
  set_turn(0);
  wait();
  set_drive(13.0);
  wait();
  set_turn(90);
  wait();
  set_drive(-9.0);
  wait();

  set_rollers(OUTTAKE);
  wait(unjamTime);
  set_rollers(SCORE);
  set_piston(piston_loader, true);
  wait(2750);

  set_rollers(INTAKE);
  set_drive(31.0, loadSpeed);
  wait();  
  set_drive(-1.0);
  wait();
  set_drive(1.5, 127);
  wait();
  wait(1500);
  set_drive(-31.0);
  wait();
  set_rollers(OUTTAKE);
  wait(unjamTime);
  set_rollers(SCORE);
  wait(2750);

  set_drive(12.0);
  wait();
  set_turn(0);
  wait();
  set_drive(98.0, DRIVE_SPEED, true);
  wait();
  set_piston(piston_loader, true);
  set_turn(90);
  set_rollers(INTAKE);
  wait();

  set_drive(20.0, loadSpeed);
  wait();  
  set_drive(-1.0);
  wait();
  set_drive(1.5, 127);
  wait();
  wait(1500);
  set_drive(-12.0);
  wait();

  set_turn(315);
  wait();
  set_drive(18.0, DRIVE_SPEED, true);
  set_piston(piston_loader, false);
  wait();
  set_turn(270);
  wait();
  set_drive(70.0, DRIVE_SPEED, true);
  wait();
  set_turn(180);
  wait();
  set_drive(13.0);
  wait();
  set_turn(270);
  wait();
  set_drive(-9.0);
  wait();

  set_rollers(OUTTAKE);
  wait(unjamTime);
  set_rollers(SCORE);
  set_piston(piston_loader, true);
  wait(2750);

  set_rollers(INTAKE);
  set_drive(31.0, loadSpeed);
  wait();
  set_drive(-1.0);
  wait(QUICK);
  set_drive(1.5, 127);
  wait(QUICK);
  wait(1500);
  set_drive(-31.0);
  wait();
  set_rollers(OUTTAKE);
  wait(unjamTime);
  set_rollers(SCORE);
  wait(2750);

  set_swing(ez::RIGHT_SWING, 179, DRIVE_SPEED, 60);
  set_piston(piston_loader, false);
  wait();
  set_drive(24.0, 127);
  wait();
  set_piston(piston_loader, true);
  wait(200);
  set_piston(piston_loader, false);
  set_rollers(OUTTAKE);
  set_drive(12.0, 127);
  wait();
  // set_drive(-6.0);
  // wait();
}

void skillsEnd() {
  set_position(0,0, 270);
  set_drive(18.0, 80);
  set_piston(piston_loader, false);
  wait();
  set_turn(210);
  wait();
  set_drive(32.0, 90);
  wait();
  set_turn(190, 90);
  wait();
  set_drive(8.0,127);
  wait();

  set_piston(piston_loader, true);
  wait(200);
  set_piston(piston_loader, false);
  set_rollers(OUTTAKE);
  set_drive(28.0, 127);
  wait();
}