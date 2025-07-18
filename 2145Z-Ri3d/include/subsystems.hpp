#pragma once

#include <algorithm>
#include <type_traits>
#include "EZ-Template/api.hpp"
#include "EZ-Template/piston.hpp"
#include "api.h"

// ** @file subsystems.hpp
// ** @brief This file contains the definitions and constructors for the robot's subsystems.
// ** @details This includes the motors, smartwire devices, and three wire port devices.
// ** @author Ansh Rao - 2145Z

#pragma region constants
// Defining drive motor ports
#define PORT_LF -18
#define PORT_LM -19
#define PORT_LB 20
#define PORT_RF -6
#define PORT_RM 7
#define PORT_RB -9

// Defining subsystem motor ports
#define PORT_ROLLER_FRONT 1
#define PORT_ROLLER_TOP   -2
#define PORT_ROLLER_BACK  13

// Defining smartwire device ports
#define PORT_IMU            3
#define PORT_VEXNET         11
#define PORT_ODOM_HORIZ     5
#define PORT_ODOM_VERT      13
#define PORT_OPTICAL_BOTTOM 14
#define PORT_OPTICAL_TOP    15
#define PORT_EXPANDER       4

// Defining three wire ports
#define PORT_PTO        3
#define PORT_TRAPDOOR   1
#define PORT_HOOD       2
#define PORT_LOADER     4

// Defining robot constants
#define DRIVE_DIAMETER 2.75
#define TRACK_WIDTH 13.5
#define DRIVE_RPM 450
#define ODOM_DIAMETER 2.125
#define OFFSET_VERT 0
#define OFFSET_HORI 0
#define DRIVE_SPEED 110
#define TURN_SPEED 90
#define SWING_SPEED 110
#define DRIVE_CURVE_THROTTLE 0.0
#define DRIVE_CURVE_TURN     0.0

// Defining controller buttons
#define BUTTON_ROLLER        pros::E_CONTROLLER_DIGITAL_L1
#define BUTTON_OUTROLLER     pros::E_CONTROLLER_DIGITAL_L2
#define BUTTON_ROLLER_CENTER pros::E_CONTROLLER_DIGITAL_R1
#define BUTTON_ROLLER_PEZ    pros::E_CONTROLLER_DIGITAL_R2
#define BUTTON_PTO           pros::E_CONTROLLER_DIGITAL_A
#define BUTTON_TRAPDOOR      pros::E_CONTROLLER_DIGITAL_B
#define BUTTON_HOOD          pros::E_CONTROLLER_DIGITAL_X
#define BUTTON_LOADER        pros::E_CONTROLLER_DIGITAL_Y
#define BUTTON_OVERRIDE      pros::E_CONTROLLER_DIGITAL_UP

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
inline pros::Link     vexnet            (PORT_VEXNET, "2145Z_link", pros::E_LINK_TX);  // This is the VEXnet radio
inline pros::Rotation odom_horiz        (PORT_ODOM_HORIZ);  // This is the horizontal tracking wheel
inline pros::Rotation odom_vert         (PORT_ODOM_VERT);   // This is the vertical tracking wheel
inline pros::Optical  optical_bottom    (PORT_OPTICAL_BOTTOM);  // This is the distance sensor
inline pros::Optical  optical_top       (PORT_OPTICAL_TOP);  // This is the top optical sensor

// three wire port constructors
//inline pros::adi::DigitalOut piston_trapdoor  ({PORT_EXPANDER, PORT_TRAPDOOR});
//inline pros::adi::DigitalOut piston_pto       ({PORT_EXPANDER, PORT_PTO});  // This is the PTO for the drive
//inline pros::adi::DigitalOut piston_hood      ({PORT_EXPANDER, PORT_HOOD});  // This is the hood for the intake
//inline pros::adi::DigitalOut piston_loader    ({PORT_EXPANDER, PORT_LOADER});  // This is the loader for the intake

inline ez::Piston piston_trapdoor (PORT_TRAPDOOR, PORT_EXPANDER);
inline ez::Piston piston_hood     (PORT_HOOD, PORT_EXPANDER);
inline ez::Piston piston_pto      (PORT_PTO, PORT_EXPANDER);
inline ez::Piston piston_loader   (PORT_LOADER, PORT_EXPANDER);

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
//inline ez::tracking_wheel vert_tracker(PORT_ODOM_VERT, ODOM_DIAMETER, OFFSET_VERT);   // This tracking wheel is parallel to the drive wheels

#pragma endregion