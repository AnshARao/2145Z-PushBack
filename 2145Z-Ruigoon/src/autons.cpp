#include "autons.hpp"
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

void SAWP13() {

  int loadSpeed = 60;
  int unjamTime = 100;

  set_position(-48, -12.5, 180);

  set_drive(33.5, DRIVE_SPEED, true);
  wait();
  set_piston(piston_loader, true);
  set_turn(270);
  wait();
  set_drive(18.0, loadSpeed);
  set_rollers(INTAKE);
  wait();
  set_drive(-1.0);
  wait();
  set_drive(1.5,127);
  wait();
  wait(300);

  set_drive(-30.0);
  wait();

  set_rollers(OUTTAKE);
  wait(unjamTime);
  set_rollers(SCORE);
  wait(1600);
  set_piston(piston_loader, false);

  set_turn(7, 127);
  wait();
  set_drive(14.0, 127);
  wait(200);
  set_piston(piston_loader, true);
  wait(CHAIN);

  set_mtp({-36, 12}, 127);
  wait(200);
  set_piston(piston_loader, false);
  wait(CHAIN);
  set_drive(12.0, 127);
  wait();


  set_piston(piston_loader, true);
  set_piston(piston_scorer, true);
  set_rollers(STOP);

  set_turn(314);
  wait();
  set_drive(-20.0, 127);
  wait();

  set_rollers(SCORE);



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
  wait(450);
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
  set_rollers(SCORE);
  wait(2000);
  set_rollers(INTAKE);

  set_boom({-60.5, -47, 270}, 125);
  wait(400);
  set_piston(piston_loader, true);
  wait();
  wait(500);
  set_mtp({-26, -47}, 125, rev);
  wait();
  set_rollers(SCORE);
  wait(1500);
  set_piston(piston_loader, false);

  set_turn(180);
  wait();
  set_drive(3.0);
  wait();
  set_turn(260);
  wait();
  set_drive(-18.0);
  wait();

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

  set_turn({-61, -46}, fwd, 100);
  wait();

  //set_mtp({-60.5, -47.5}, 125);
  set_drive(11.0, 125);
  wait();
  wait(750);

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

  wait(650);//wait in miliseconds
  set_piston(piston_loader, true); //sets piston to true (out) (lilwill)
  wait(); //waits until movement is done

  set_mtp({-5, 42}, DRIVE_SPEED); //moves to point at drive speed
  wait(325); //waits 325 miliseconds
  set_piston(piston_loader, false);//sets piston to false (in) (lilwill)
  wait(); //waits until movement is done

  set_piston(piston_loader, true);
  wait(200);

  set_mtp({-18, 24}, DRIVE_SPEED, rev);
  wait();
  set_rollers(STOP);
  set_piston(piston_scorer, true); //sets piston to false (in) (scorer) (false is middle goal) (True is long goal)

  //5303591943

  set_boom({-6, 10, get_theta(currentPoint, {-6, 10}, rev)}, DRIVE_SPEED, rev); //sets boom to point (x, y, angle), speed, fwd or rev 
  //it is like a move to point but with angle
  wait();

  set_rollers(SCORE); 
  wait(1250);
  set_drive(7.0);
  set_rollers(INTAKE);
  wait(CHAIN);

  set_boom({-54, 39.5, 270}, DRIVE_SPEED, fwd, true);
  wait();

  set_drive(7, 127);
  wait(650);
  
  set_drive(-30.0, 127);
  wait();
  set_rollers(SCORE);
  set_piston(piston_loader, false);  
  wait(2000);

  set_turn(180);
  wait();
  set_drive(3.0);
  wait();
  set_turn(260);
  wait();
  set_drive(-18.0);
  wait();
}

void fourFiveRight() {
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
  wait();

  set_drive(-1.0);
  wait();
  set_drive(1.5,127);
  wait();

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
  set_piston(piston_scorer, true);
  wait();

  set_drive(-4.0);
  wait();
  set_rollers(SCORE);
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
  set_rollers(SCORE);
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
  set_rollers(SCORE);
  wait(3000);
  set_drive(20);

  set_piston(piston_loader, false);
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

  int loadSpeed = 60;
  int unjamTime = 100;

  set_position(-48, -12.5, 180);

  set_drive(34.0, DRIVE_SPEED, true);
  wait();
  set_piston(piston_loader, true);
  set_turn(270);
  wait();
  set_drive(18.0, loadSpeed);
  set_rollers(INTAKE);
  wait();
  set_drive(-1.0);
  wait();
  set_drive(1.5,127);
  wait();
  wait(2000);

  set_drive(-12.0);
  wait();
  set_turn(135);
  wait();

  set_drive(18.0);
  set_piston(piston_loader, false);
  wait();
  set_turn(90);
  wait();
  set_drive(68.0, DRIVE_SPEED, true);
  wait();
  set_turn(0);
  wait();
  set_drive(12.5);
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
  wait(2000);
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
  set_drive(96.0, DRIVE_SPEED, true);
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
  wait(2000);
  set_drive(-12.0);
  wait();

  set_turn(315);
  wait();
  set_drive(18.0, DRIVE_SPEED, true);
  set_piston(piston_loader, false);
  wait();
  set_turn(270);
  wait();
  set_drive(68.0, DRIVE_SPEED, true);
  wait();
  set_turn(180);
  wait();
  set_drive(12.0);
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
  wait();
  set_drive(1.5, 127);
  wait();
  wait(2000);
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
  set_drive(28.0, 127);
  wait();
  
}