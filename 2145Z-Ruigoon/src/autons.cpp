#include "autons.hpp"
#include <cmath>
#include <string>
#include "EZ-Template/util.hpp"
#include "controls.hpp"
#include "drive.hpp"
#include "main.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "screen.hpp"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////


void testing() {
  set_position(0,0,0);

  set_mtp({-12, 36}, DRIVE_SPEED);
  wait();

  set_mtp({24, 24}, DRIVE_SPEED);
  wait();

  set_boom({0,0, 0}, DRIVE_SPEED);
  wait();
}

void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);

  print("vert: " + std::to_string(f_offset) + " , hori: " + std::to_string(r_offset));
}

void SAWP() {

  int loadSpeed = 70;
  int unjamTime = 100;

  set_position(-45, -12.5, 180);

  set_drive(26, 127);
  wait(CHAIN);
  set_piston(piston_loader, true);
  set_turn(270);
  wait(QUICK);
  set_drive(14.0, loadSpeed);
  set_rollers(INTAKE);
  wait();
  wait(150);

  set_drive(-31.0, 127);
  wait(750);
  set_rollers(SCORE);
  wait();
  set_piston(piston_loader, false);
  wait(900);

  set_turn(10, 127);
  wait();
  set_drive(15.0, 127);
  wait(400);
  set_piston(piston_loader, true);
  set_rollers(INTAKE);
  wait(CHAIN);

  set_turn(357, DRIVE_SPEED);
  wait(CHAIN);
  set_drive(42.0, 127);
  set_piston(piston_loader, false);
  wait(750);
  set_piston(piston_loader, true);
  wait();

  set_turn(315);
  set_rollers(STOP);
  wait();
  set_drive(-14.25, DRIVE_SPEED, false, false);
  wait();

  set_rollers(-OUTTAKE);
  wait(100);
  set_rollers(SCORE_MID);
  wait(CHAIN);
  wait(800);

  set_drive(37.0, 127);
  set_rollers(INTAKE);
  set_piston(piston_loader, false);
  wait(CHAIN);
  set_piston(piston_loader, true);
  set_turn(270);
  wait(CHAIN);
  set_drive(24.0, loadSpeed);
  wait();
  wait(200);
  set_drive(-36.0, 127);
  wait(750);
  set_rollers(SCORE);
  wait(CHAIN);
  set_piston(piston_loader, false);
}


void doNothing() {

}

void sixThreeLeft() {
  set_position(-47, 16, 90);

  set_drive(4.0, 125);
  wait(CHAIN);

  set_mtp({-13, 23.5}, 75, fwd, true);
  set_rollers(INTAKE);
  wait(650);
  set_piston(piston_loader, true);
  wait();

  set_mtp({-6, 43}, DRIVE_SPEED);
  wait(125);
  set_piston(piston_loader, false);
  wait();

  set_drive(-15.0, DRIVE_SPEED);
  wait(QUICK);
  //set_piston(piston_loader, false);

  set_mtp({-44, 47.25}, DRIVE_SPEED, fwd, true);
  set_piston(piston_loader, true);
  wait(CHAIN);
  
  set_turn({-25, 48}, rev, TURN_SPEED);
  wait(CHAIN);

  set_boom({-25, 48, 90}, DRIVE_SPEED, rev);
  wait();
  set_rollers(OUTTAKE);
  wait(150);
  set_rollers(SCORE);
  wait(1750);
  set_rollers(INTAKE);

  set_boom({-60, 48, 270}, 125);
  set_piston(piston_loader, true);
  wait(CHAIN);
  // set_drive(-0.25);
  // wait(CHAIN);
  set_drive(-4.0);
  wait(CHAIN);
  set_piston(piston_loader, false);

  set_turn({0,-8}, rev, TURN_SPEED);
  wait(QUICK);
  set_drive(-50.0, 127);
  wait(QUICK);
  set_piston(piston_scorer, true);
  set_rollers(12000, -12000, -12000);
  wait(150);
  set_rollers(SCORE_MID);
  wait(1750);
  set_drive(22.0);
  wait(CHAIN);
  set_turn(270);
  wait(CHAIN);
  set_drive(-16.0);
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

  set_mtp({-6, -43}, DRIVE_SPEED);
  wait(125);
  set_piston(piston_loader, false);
  wait();


  set_drive(-15.0, DRIVE_SPEED);
  wait(QUICK);
  //set_piston(piston_loader, false);

  set_mtp({-44, -47.25}, DRIVE_SPEED, fwd, true);
  set_piston(piston_loader, true);
  wait(CHAIN);
  
  set_turn({-25, -48}, rev, TURN_SPEED);
  wait(CHAIN);

  set_boom({-25, -48, 90}, DRIVE_SPEED, rev);
  wait();
  set_rollers(OUTTAKE);
  wait(150);
  set_rollers(SCORE);
  wait(1750);
  set_rollers(INTAKE);

  set_boom({-60, -48, 270}, 125);
  set_piston(piston_loader, true);
  wait(CHAIN);
  // set_drive(-0.25);
  // wait(CHAIN);
  set_drive(-4.0);
  wait(CHAIN);
  set_piston(piston_loader, false);

  set_turn({0,14}, fwd, TURN_SPEED);
  wait(QUICK);
  set_drive(46.0, 127);
  wait(QUICK);
  set_rollers(-12000);
  wait(100);
  set_rollers(INTAKE);
  wait(100);
  set_rollers(-9750);
  wait(1750);
  set_drive(1.0);
  wait();
  set_drive(-20.0);
  wait(CHAIN);
  set_turn(90);
  wait(CHAIN);
  set_drive(12.0);
  wait();
}

void fourFive() {
  set_position(-47, -16, 90);

  set_drive(4.0, 125);
  wait(CHAIN);

  set_mtp({-13, -23.5}, 75, fwd, true);
  set_rollers(INTAKE);
  wait(650);
  set_piston(piston_loader, true);
  wait();

  set_mtp({-6, -43}, DRIVE_SPEED);
  wait(125);
  set_piston(piston_loader, false);
  wait();


  set_drive(-24.0, DRIVE_SPEED);
  wait(QUICK);
  set_turn(45);
  wait(QUICK);
  set_drive(6.0);
  set_rollers(OUTTAKE);
  wait();
  wait(500);

  set_drive(-47.0);
  set_piston(piston_loader, true);
  set_rollers(INTAKE);
  wait();
  set_turn(270);
  wait();
  set_drive(12.0);
  wait();
  wait(250);
  set_drive(-6.0);
  wait();

  set_turn(22);
  wait();

  set_drive(76.0);
  set_piston(piston_loader, false);
  wait();

  set_turn(315);
  wait();
  set_drive(-15.25);
  wait();
  set_rollers(12000, 12000, -9000);
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

void skills() {

  int loadSpeed = 50;
  int unjamTime = 125;

  set_position(-48, -12.5, 180);

  set_drive(32.0, DRIVE_SPEED, true);
  set_piston(piston_wing, true);
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
  set_turn(45);
  wait();

  set_drive(20.0);
  wait();
  set_turn(90);
  wait();
  set_drive(70.0, DRIVE_SPEED, true);
  wait();
  set_turn(180);
  wait();
  set_drive(13.0);
  wait();
  set_turn(90);
  wait();
  set_drive(-14.0);
  wait();

  set_rollers(OUTTAKE);
  wait(unjamTime);
  set_rollers(SCORE);
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
  wait(1500);
  set_drive(-12.0);
  wait();

  set_turn(225);
  wait();
  set_drive(18.0, DRIVE_SPEED, true);
  set_piston(piston_loader, false);
  wait();
  set_turn(270);
  wait();
  set_drive(70.0, DRIVE_SPEED, true);
  wait();
  set_turn(0);
  wait();
  set_drive(13.0);
  wait();
  set_turn(270);
  wait();
  set_drive(-12.0);
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