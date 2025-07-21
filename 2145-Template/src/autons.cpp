#include "autons.hpp"
#include "EZ-Template/util.hpp"
#include "drive.hpp"
#include "main.h"
#include "pros/motors.h"
#include "subsystems.hpp"

void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

void example_on_functions() {
  set_position(60,24,90); // sets robot postion at (60,24) with an angle of 90

  wait(WAIT); // waits for a normal pid wait
  wait(QUICK); // waits for a quick pid wait
  wait(CHAIN); // waits for a motion chained pid wait
  wait(500);  // waits for 500 milliseconds
  wait_until(24); //waits until the robot has driven 24 inches or until 24 degrees
  wait_until({12,12}); // waits until the robot has gone through coordinate (12,12)

  move_point({24, 48, 60}, ez::fwd, 80); // moves to (24,48) facing 60 degrees going forwards at 80/127 speed
  move_point({48, 24}, ez::rev, 127); //moves to (48,24) facing 0 degrees going backwards at max speed

  set_drive(24, DRIVE_SPEED); // drives 24 inches at drive speed
  set_drive(24, DRIVE_SPEED, true); // same as above but with slew, only use if you know what you're doing

  set_turn(90, TURN_SPEED);   // turns to 90 degrees at turn speed
  set_turn(90, TURN_SPEED, ez::ccw);  // turns to 90 degrees counterclock at turn speed
  set_turn({24,24}, fwd, TURN_SPEED); // turns forward to (24,24) at turn speed
  set_turn({24,24}, rev, 127, ccw); // turns backwards to (24,24) at max speed in the opposite direction
  set_turn_relative(90, TURN_SPEED); // turns 90 degrees from previous position
  set_turn_relative(90, TURN_SPEED, ccw); // turns 90 degrees from the previous position counter-clockwise (left)

  set_swing(ez::LEFT_SWING, 45, SWING_SPEED); //swing left to 45 degrees at swing speed
  set_swing(ez::RIGHT_SWING, 60, 127, 30);  //swing right to 30 degrees at max speed, with an opposite speed of 30
  set_swing(ez::LEFT_SWING, 90, SWING_SPEED, ez::cw); //swing left to 90 degrees at swing speed clockwise
  set_swing(ez::RIGHT_SWING, 75, SWING_SPEED, 100, ez::ccw);  // swing right to 75 degrees at swing speed with an opposite speed of 100/127 going counter-clockwise

}

void doNothing() {
  set_position(0,0,0);
  set_drive(24, DRIVE_SPEED);
}