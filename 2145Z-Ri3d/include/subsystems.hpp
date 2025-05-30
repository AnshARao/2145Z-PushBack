#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

// ** @file subsystems.hpp
// ** @brief This file contains the definitions and constructors for the robot's subsystems.
// ** @details This includes the motors, smartwire devices, and three wire port devices.
// ** @author Ansh Rao - 2145Z

#pragma region constants
// Defining drive motor ports
#define PORT_LF 1
#define PORT_LM 2
#define PORT_LB 3
#define PORT_RF 4
#define PORT_RM 5
#define PORT_RB 6

// Defining subsystem motor ports
#define PORT_PREROLLER 7
#define PORT_ROLLER1 8
#define PORT_ROLLER2 9

// Defining smartwire device ports
#define PORT_IMU 10
#define PORT_VEXNET 11
#define PORT_ODOM_HORIZ 12
#define PORT_ODOM_VERT 13
#define PORT_OPTICAL 14

// Defining three wire ports

// Defining robot constants
#define DRIVE_DIAMETER 3.25
#define DRIVE_RPM 450
#define ODOM_DIAMETER 2.125
#define OFFSET_VERT 0
#define OFFSET_HORI 0
#define DRIVE_SPEED 110
#define TURN_SPEED 90
#define SWING_SPEED 110

// Defining controller buttons
#define BUTTON_INTAKE  pros::E_CONTROLLER_DIGITAL_L1
#define BUTTON_OUTTAKE pros::E_CONTROLLER_DIGITAL_L2
#define BUTTON_ROLLERS pros::E_CONTROLLER_DIGITAL_R1
#define BUTTON_OUTROLLERS    pros::E_CONTROLLER_DIGITAL_R2

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
inline pros::MotorGroup motorgroup_L({PORT_LF, PORT_LM, PORT_LB, /*PORT_LPTO*/});
inline pros::MotorGroup motorgroup_R({PORT_RF, PORT_RM, PORT_RB, /*PORT_RPTO*/});

// Subsystem motor constructors
inline pros::Motor motor_intake     (PORT_PREROLLER, pros::v5::MotorGears::blue);
inline pros::Motor motor_roller1    (PORT_ROLLER1, pros::v5::MotorGears::green);
inline pros::Motor motor_roller2    (PORT_ROLLER2, pros::v5::MotorGears::green);

// smartwire device constructors
inline pros::Imu      imu        (PORT_IMU);
inline pros::Link     vexnet     (PORT_VEXNET, "2145Z_link", pros::E_LINK_TX);  // This is the VEXnet radio
inline pros::Rotation odom_horiz (PORT_ODOM_HORIZ);  // This is the horizontal tracking wheel
inline pros::Rotation odom_vert  (PORT_ODOM_VERT);   // This is the vertical tracking wheel
inline pros::Distance opticalSort(PORT_OPTICAL);  // This is the distance sensor

// three wire port constructors


// Chassis constructor
inline ez::Drive chassis(
    {PORT_LF, PORT_LM, PORT_LB, /*PORT_LTPO*/},    // Left Chassis Ports
    {-PORT_RF, PORT_RM, PORT_RB, /*PORT_RPTO*/},  // Right Chassis Ports

    PORT_IMU,                                                          // IMU Port
    DRIVE_DIAMETER,                                              // Wheel Diameter
    DRIVE_RPM);                                                           // Wheel RPM
// Uncomment the trackers you're using here!
// - `8` and `9` are smart ports (making these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
//inline ez::tracking_wheel horiz_tracker(PORT_ODOM_HORIZ, ODOM_DIAMETER, OFFSET_HORI);  // This tracking wheel is perpendicular to the drive wheels
//inline ez::tracking_wheel vert_tracker(PORT_ODOM_VERT, ODOM_DIAMETER, OFFSET_VERT);   // This tracking wheel is parallel to the drive wheels

#pragma endregion