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
#define PORT_LF -15
#define PORT_LM -11
#define PORT_LB 13
#define PORT_RF 14
#define PORT_RM 16
#define PORT_RB -17

// Defining subsystem motor ports
#define PORT_ROLLER_FRONT -6
#define PORT_ROLLER_TOP   7
#define PORT_ROLLER_BACK  2

// Defining smartwire device ports
#define PORT_IMU            20
#define PORT_ODOM_HORIZ     1
#define PORT_ODOM_VERT      3
#define PORT_OPTICAL_TOP    5
#define PORT_OPTICAL_MID    5
#define PORT_OPTICAL_BOT    19

// Defining three wire ports
#define PORT_PTO            'G'
#define PORT_BLOCKER_TOP    'F'
#define PORT_BLOCKER_BOT    'A'
#define PORT_HOPPER         'H'
#define PORT_LOADER         'C'
#define PORT_HOOD           'B'
#define PORT_PUNCHER_MID    'D'
#define PORT_PUNCHER_TOP    'D'

// Defining robot constants
#define DRIVE_DIAMETER      2.75
#define TRACK_WIDTH         12.5
#define DRIVE_RPM           450
#define ODOM_DIAMETER       2.125
#define OFFSET_VERT         -6.25
#define OFFSET_HORI         3.25
#define DRIVE_SPEED         110
#define TURN_SPEED          90
#define SWING_SPEED         110
#define DRIVE_CURVE         0.0

// Defining controller buttons
#define BUTTON_INTAKE       pros::E_CONTROLLER_DIGITAL_R1
#define BUTTON_OUTTAKE      pros::E_CONTROLLER_DIGITAL_R2
#define BUTTON_SCORE_TOP    pros::E_CONTROLLER_DIGITAL_L1
#define BUTTON_SCORE_MID    pros::E_CONTROLLER_DIGITAL_L2
#define BUTTON_LOADER       pros::E_CONTROLLER_DIGITAL_B
#define BUTTON_PTO          pros::E_CONTROLLER_DIGITAL_RIGHT
#define BUTTON_OVERRIDE     pros::E_CONTROLLER_DIGITAL_LEFT
#define BUTTON_HOPPER       pros::E_CONTROLLER_DIGITAL_Y
#define BUTTON_PUNCHER_TOP pros::E_CONTROLLER_DIGITAL_DOWN
#define BUTTON_PUNCHER_MID pros::E_CONTROLLER_DIGITAL_A

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
inline pros::Motor motor_roller_front   (PORT_ROLLER_FRONT, pros::v5::MotorGears::green);
inline pros::Motor motor_roller_top     (PORT_ROLLER_TOP, pros::v5::MotorGears::green);
inline pros::Motor motor_roller_back    (PORT_ROLLER_BACK, pros::v5::MotorGears::blue);

// smartwire device constructors
inline pros::Imu      imu               (PORT_IMU);
//inline pros::Link     vexnet            (PORT_VEXNET, "2145Z_link", pros::E_LINK_TX);
inline pros::Rotation odom_horiz        (PORT_ODOM_HORIZ);
inline pros::Rotation odom_vert         (PORT_ODOM_VERT);
inline pros::Optical  optical_top       (PORT_OPTICAL_TOP);
inline pros::Optical  optical_mid       (PORT_OPTICAL_MID);
inline pros::Optical  optical_bot       (PORT_OPTICAL_BOT);

//three wire port constructors
inline pros::adi::DigitalOut piston_pto       ( PORT_PTO);  // This is the PTO for the drive
inline pros::adi::DigitalOut piston_blocker_top   (PORT_BLOCKER_TOP);
inline pros::adi::DigitalOut piston_blocker_bot   (PORT_BLOCKER_BOT);
inline pros::adi::DigitalOut piston_hopper    (PORT_HOPPER);  // This is the hood for the intake
inline pros::adi::DigitalOut piston_loader    (PORT_LOADER);  // This is the loader for the intake
inline pros::adi::DigitalOut piston_hood      (PORT_HOOD);
inline pros::adi::DigitalOut piston_puncher_mid(PORT_PUNCHER_MID);
inline pros::adi::DigitalOut piston_puncher_top(PORT_PUNCHER_TOP);
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