#pragma once

/**
* @file subsystems.hpp
* @brief This file contains the definitions and constructors for the robot's subsystems.
* @details This includes the motors, smartwire devices, and three wire port devices.
* @author Ansh Rao - 2145Z
*/

#include <cstdint>  // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/optical.hpp"
#include "api.h"    // IWYU pragma: keep
#include "EZ-Template/api.hpp"  // IWYU pragma: keep
#pragma region constants

// Defining drive motor ports
#define PORT_LF 0
#define PORT_LM 0
#define PORT_LB 0
#define PORT_RF 0
#define PORT_RM 0
#define PORT_RB 0

// Defining subsystem motor ports
#define PORT_ROLLER         0

// Defining smartwire device ports
#define PORT_IMU            0
#define PORT_ODOM_HORIZ     0
#define PORT_ODOM_VERT      0
#define PORT_OPTICAL        0

// Defining three wire ports
#define PORT_PISTON         0

// Defining robot constants
#define DRIVE_DIAMETER      3.25
#define TRACK_WIDTH         2
#define DRIVE_RPM           450
#define ODOM_DIAMETER       2.0
#define OFFSET_VERT         0
#define OFFSET_HORI         0
#define DRIVE_SPEED         110
#define TURN_SPEED          90
#define SWING_SPEED         110
#define DRIVE_CURVE         0.0

// Defining controller buttons
#define BUTTON_INTAKE       pros::E_CONTROLLER_DIGITAL_R1
#define BUTTON_OUTTAKE      pros::E_CONTROLLER_DIGITAL_R2
#define BUTTON_SCORE_TOP    pros::E_CONTROLLER_DIGITAL_L1
#define BUTTON_SCORE_MID    pros::E_CONTROLLER_DIGITAL_L2


#pragma endregion

#pragma region constructors 

// Controller constructors
inline pros::Controller controlla (pros::E_CONTROLLER_MASTER);  // Master controller
inline pros::Controller controlla2(pros::E_CONTROLLER_PARTNER);  // Partner controller

// Motor constructors
inline pros::Motor motor_LF     (PORT_LF, pros::v5::MotorGears::blue);
inline pros::Motor motor_LM     (PORT_LM, pros::v5::MotorGears::blue);
inline pros::Motor motor_LB     (PORT_LB, pros::v5::MotorGears::blue);
inline pros::Motor motor_RF     (PORT_RF, pros::v5::MotorGears::blue);
inline pros::Motor motor_RM     (PORT_RM, pros::v5::MotorGears::blue);
inline pros::Motor motor_RB     (PORT_RB, pros::v5::MotorGears::blue);


// Drive motorgroup constructors
inline pros::MotorGroup motorgroup_L({PORT_LF, PORT_LM, PORT_LB,});
inline pros::MotorGroup motorgroup_R({PORT_RF, PORT_RM, PORT_RB,});

// Subsystem motor constructors
inline pros::Motor motor_roller_front   (PORT_ROLLER, pros::v5::MotorGears::blue);


// smartwire device constructors
inline pros::Imu      imu               (PORT_IMU);
inline pros::Rotation odom_horiz        (PORT_ODOM_HORIZ);
inline pros::Rotation odom_vert         (PORT_ODOM_VERT);
inline pros::Optical  optical           (PORT_OPTICAL);

//three wire port constructors

// Chassis constructor
inline ez::Drive chassis(
    {PORT_LF, PORT_LM, PORT_LB},    // Left Chassis Ports
    {PORT_RF, PORT_RM, PORT_RB},  // Right Chassis Ports

    PORT_IMU,   // IMU Port
    DRIVE_DIAMETER, // Wheel Diameter
    DRIVE_RPM); // Wheel RPM
inline ez::tracking_wheel horiz_tracker(PORT_ODOM_HORIZ, ODOM_DIAMETER, OFFSET_HORI);  // This tracking wheel is perpendicular to the drive wheels
inline ez::tracking_wheel vert_tracker(PORT_ODOM_VERT, ODOM_DIAMETER, OFFSET_VERT);   // This tracking wheel is parallel to the drive wheels

#pragma endregion

inline void default_constants() {
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

    chassis.pid_drive_chain_constant_set(1_in);
    chassis.pid_turn_chain_constant_set(1_deg);
}