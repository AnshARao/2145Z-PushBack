#include "EZ-Template/util.hpp"
#include "controls.hpp"
#include "drive.hpp"
#include "main.h"  // IWYU pragma: keep
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "screen.hpp"
#include "subsystems.hpp"

/**
 * @file main.cpp
 * @brief This file contains the autonomous functions for the robot.
 * @details This includes both competition and example/testing functions.* 
 * @author Ansh Rao - 2145Z
 */

#pragma region example_autos

//
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
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
}

#pragma endregion

void doNothing() {
  // This is a placeholder function that does nothing
  // It can be used to test the robot's movement without doing anything
  setPosition(120, 120, 0);
  driveSet(0.001, 1);
}

void move_forward() {
    setPosition(0,0,0);
    turnSet(359,1);
}

void move_forward1() {
  setPosition(72,20,0);
  driveSet(24,  127);
}

void move_forward2() {
  setPosition(144,20,0);
  driveSet(24, 127);
}

void left9() {


}

void right9() {
    setPosition(80, 20, 45);

}

void sawpLeft() {
  setPosition(-55, 18, 0);
  driveSet(30, 127);
  turnSet(270, TURN_SPEED);
  driveSet(6, DRIVE_SPEED);
  setLoader(true);
  // set_rollers_intake();
  // // add color sensing and waiting logic here
  // delayMillis(1000); //delay will suffice for now
  // driveSet(-6, DRIVE_SPEED);
  // setLoader(false);
  // allianceColor == Alliances::RED ? turnSet(90, TURN_SPEED) : turnSet(90-180, TURN_SPEED);
  // driveSet(26, DRIVE_SPEED);
  // set_rollers_score_top();
  // // add score until and color sensing logic here
  // delayMillis(1000); // delay will suffice for now
  // driveSet(-20, DRIVE_SPEED);
  // allianceColor == RED ? turnSet(135, TURN_SPEED) : turnSet(135-180, TURN_SPEED);
  // set_rollers_intake();
  // driveSet(40, DRIVE_SPEED);
  // set_rollers_score_middle();
  // // add score until and color sensing logic here
  // delayMillis(1000); // delay will suffice for now
  // driveSet(-12, DRIVE_SPEED);
  // allianceColor == RED ? turnSet(0, TURN_SPEED) : turnSet(0-180, TURN_SPEED);

}

void sawpRight() {
  setPosition(82, 20, 45);
  driveSet(60, DRIVE_SPEED);
}

void skills() {
  allianceColor = RED;
  setPosition(-62.5,-18,0); // start
  driveSet(42, DRIVE_SPEED); // clear out zone
  pidWait(CHAIN);
  turnSet(90,TURN_SPEED); // turn to 4 stack
  pidWait(CHAIN);
  // intake
  driveSet(36, DRIVE_SPEED); // drive to 4 stack
  pidWait(CHAIN);
  turnSet(135, TURN_SPEED); // turn to center goal
  pidWait(CHAIN);
  driveSet(16, DRIVE_SPEED);  // drive to center goal
  pidWait(CHAIN);
  // score center goal
  delayMillis(500); // *TEMP*

  moveToPoint({-48, 48, 135}, ez::rev, DRIVE_SPEED);
  pidWait(CHAIN);

  turnSet(270, TURN_SPEED);
  pidWait(CHAIN);
  driveSet(10, DRIVE_SPEED);
  pidWait(CHAIN);
  setLoader(true);
  pidWait(WAIT);
  delayMillis(1000);
  driveSet(-10,DRIVE_SPEED);
  pidWait(CHAIN);
  turnSet(90, TURN_SPEED, ez::ccw);
  pidWait(CHAIN);
  driveSet(14, DRIVE_SPEED);
  pidWait(CHAIN);
  // score + logic
  delayMillis(1000); /*TEMP*/
  moveToPoint({-44, 48}, rev, DRIVE_SPEED);
  pidWait(CHAIN);
  turnSet(180, TURN_SPEED);
  pidWait(CHAIN);
  driveSet(98, DRIVE_SPEED);
  pidWait(CHAIN);
  turnSet(270, TURN_SPEED);
  pidWait(CHAIN);
  driveSet(14, DRIVE_SPEED);
  pidWait(CHAIN);
  setLoader(true);
  pidWait(WAIT);
  delayMillis(1000);
  driveSet(-14,DRIVE_SPEED);
  pidWait(CHAIN);
  turnSet(90, TURN_SPEED, ez::cw);
  pidWait(CHAIN);
  driveSet(10, DRIVE_SPEED);
  pidWait(CHAIN);
  delayMillis(1000); /*TEMP*/
  driveSet(-12, DRIVE_SPEED);
  pidWait(CHAIN);
  turnSet(0, TURN_SPEED);
  pidWait(CHAIN);
  driveSet(14, DRIVE_SPEED);
  pidWait(CHAIN);
  turnSet(90, TURN_SPEED);
  pidWait(CHAIN);
  driveSet(70, DRIVE_SPEED);
  pidWait(CHAIN);
  //turnSet(135, TURN_SPEED);
  moveToPoint({48, -51}, fwd, DRIVE_SPEED);
  pidWait(CHAIN);
  turnSet(90, TURN_SPEED);
  pidWait(CHAIN);
  driveSet(8, DRIVE_SPEED);
  pidWait(CHAIN);
  delayMillis(1000);  
  driveSet(-8, DRIVE_SPEED);
  pidWait(CHAIN);
  // turnSet(30, TURN_SPEED);
  // pidWait(CHAIN);
  // driveSet(76, DRIVE_SPEED);
  // pidWait(CHAIN);


  swingSet(ez::RIGHT_SWING, 0, SWING_SPEED, 60, ccw);
  pidWait(CHAIN);

  driveSet(30, DRIVE_SPEED);
  pidWait(CHAIN);


  // chassis.drive_set(-60, -60);
  // delayMillis(250);
  // setPosition(62.5, 18, 0);
  // driveSet(8, DRIVE_SPEED);
  // pidWait(CHAIN);
  turnSet(270, TURN_SPEED);
  pidWait(CHAIN);
  moveToPoint({24, 24}, fwd, DRIVE_SPEED);
  pidWait(CHAIN);
  turnSet(225, TURN_SPEED);
  pidWait(CHAIN);
  driveSet(14, DRIVE_SPEED);
  pidWait(CHAIN);
  delayMillis(1000);
  driveSet(-4, DRIVE_SPEED);
  pidWait(CHAIN);
  moveToPoint({62, 0}, fwd, DRIVE_SPEED);
  pidWait(CHAIN);

}
