#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "pros/adi.hpp"
#include "pros/link.h"
#include "pros/link.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "subsystems.hpp"

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
// Defining pto drive motor ports
/**
#define PORT_LPTO 7
#define PORT_RPTO 8
*/

// Defining subsystem motor ports
#define PORT_INTAKE 7
#define PORT_ROLLERS 8  //placeholder for future subsystem

// Defining smartwire device ports
#define PORT_IMU 9
#define PORT_VEXNET 10
#define PORT_ODOM_HORIZ 11
#define PORT_ODOM_VERT 12
#define PORT_DISTANCE 13

// Defining three wire ports
#define PORT_PTO 'A'

// Defining robot constants
#define DRIVE_DIAMETER 3.25
#define DRIVE_RPM 600
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
//inline pros::Motor motor_LPTO   (PORT_LPTO, pros::v5::MotorGears::blue);
//inline pros::Motor motor_RPTO   (PORT_RPTO, pros::v5::MotorGears::blue);
    
// Drive motorgroup constructors
inline pros::MotorGroup motorgroup_L({PORT_LF, PORT_LM, PORT_LB, /*PORT_LPTO*/});
inline pros::MotorGroup motorgroup_R({PORT_RF, PORT_RM, PORT_RB, /*PORT_RPTO*/});

// Subsystem motor constructors
inline pros::Motor motor_intake     (PORT_INTAKE, pros::v5::MotorGears::blue);
inline pros::Motor motor_rollers    (PORT_ROLLERS, pros::v5::MotorGears::blue);

// smartwire device constructors
inline pros::Imu      imu       (PORT_IMU);
inline pros::Link     vexnet    (PORT_VEXNET, "2145Z_link", pros::E_LINK_TX);  // This is the VEXnet radio
inline pros::Rotation odom_horiz(PORT_ODOM_HORIZ);  // This is the horizontal tracking wheel
inline pros::Rotation odom_vert (PORT_ODOM_VERT);   // This is the vertical tracking wheel
inline pros::Distance distance_1(PORT_DISTANCE);  // This is the distance sensor

// three wire port constructors
inline pros::adi::DigitalOut piston_pto(PORT_PTO);

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
 inline ez::tracking_wheel horiz_tracker(PORT_ODOM_HORIZ, ODOM_DIAMETER, OFFSET_HORI);  // This tracking wheel is perpendicular to the drive wheels
 inline ez::tracking_wheel vert_tracker(PORT_ODOM_VERT, ODOM_DIAMETER, OFFSET_VERT);   // This tracking wheel is parallel to the drive wheels

#pragma endregion