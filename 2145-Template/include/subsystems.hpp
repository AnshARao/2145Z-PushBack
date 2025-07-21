#pragma once

/**
* @file subsystems.hpp
* @brief This file contains the definitions and constructors for the robot's subsystems.
* @details This includes the motors, smartwire devices, and three wire port devices.
* @author Ansh Rao - 2145Z
*/

// Define your smartwire device ports here (motors, sensors, etc.)
// Values are from 1-21, and adding a negative sign reverses motors

// Define your drive motor ports here
#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/link.h"
#include "pros/link.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#define PORT_LF 0
#define PORT_LM 0
#define PORT_LB 0
#define PORT_RF 0
#define PORT_RM 0
#define PORT_RB 0

// Define your subsystem ports here
#define PORT_SUBSYSTEM_1    0 
#define PORT_SUBSYSTEM_2    0

// Define your other smartwire device ports
#define PORT_IMU        0
#define PORT_VEXNET     0
#define PORT_SENSOR_1   0
#define PORT_SENSOR_2   0

// Define your three wire ports
// Values are from 'A' - 'H'

// Define your piston ports here
#define PORT_PISTON_1   'A'
#define PORT_PISTON_2   'B'

// Define your three wire sensors ports here
#define PORT_LIMIT  'C'
#define PORT_BUTTON 'D'

// Define other robot constants here
#define WHEEL_DIAMETER  0   // (2.0, 2.75, 3.25, or 4.0)
#define TRACK_WIDTH     0   // Drivetrain width in inches (holes / 2s)
#define DRIVE_RPM       0   // (motor rpm * motor gear / wheel gear)

// These are the *default* speeds which should be good. They are out of 127
#define DRIVE_SPEED     110 // Default drive speed
#define TURN_SPEED      90  // Default turn speed
#define SWING_SPEED     110 // Default swing speed
// 0 for linear, increase for a higher curve
#define DRIVE_CURVE     0.0

// Define your buttons here
#define BUTTON_INTAKE   pros::E_CONTROLLER_DIGITAL_L1
#define BUTTON_OUTTAKE  pros::E_CONTROLLER_DIGTIAL_L2

// Controller constructors
inline pros::Controller master  (pros::E_CONTROLLER_MASTER);
inline pros::Controller partner (pros::E_CONTROLLER_PARTNER);

// Construct your smartwire devices here

// Drive motor constructors
// (port previously defined, motor cartridge (red, green, or blue))
inline pros::Motor motor_LF     (PORT_LF, pros::v5::MotorGears::blue);
inline pros::Motor motor_LM     (PORT_LM, pros::v5::MotorGears::blue);
inline pros::Motor motor_LB     (PORT_LB, pros::v5::MotorGears::blue);
inline pros::Motor motor_RF     (PORT_RF, pros::v5::MotorGears::blue);
inline pros::Motor motor_RM     (PORT_RM, pros::v5::MotorGears::blue);
inline pros::Motor motor_RB     (PORT_RB, pros::v5::MotorGears::blue);

// Drive motorgroup constructors
// Could be useful with other templates or custom drive functions
inline pros::MotorGroup motorgroup_L({PORT_LF, PORT_LM, PORT_LB,});
inline pros::MotorGroup motorgroup_R({PORT_RF, PORT_RM, PORT_RB,});

// Construct your subsustem motors here
// (port prevously defined, motor cartridge (red, green, or blue))
inline pros::Motor motor_subsystem_1    (PORT_SUBSYSTEM_1, pros::v5::MotorGears::green);
inline pros::Motor motor_subsystem_2    (PORT_SUBSYSTEM_2, pros::v5::MotorGears::green);

// Construct your other smartwire devices here
// Look through the pros documentation for all different types of sensors
// For now, we'll use an IMU, Link, Optical, and Distance sensor
inline pros::Imu imu            (PORT_IMU);
// replace "2156_Template_link" with your own team (ex. "2145z_Link")
inline pros::Link vexnet        (PORT_VEXNET, "2145_Template_link", pros::E_LINK_TX);
inline pros::Optical optical    (PORT_SENSOR_1);
inline pros::Distance distance  (PORT_SENSOR_2);

// Construct your three wire devices here

// Construct your pistons here, note that these use digitalOut
inline pros::adi::DigitalOut piston_1   (PORT_PISTON_1);
inline pros::adi::DigitalOut piston_2   (PORT_PISTON_2);

// Constructor for chassis, this should be good by itself
inline ez::Drive chassis(
    {PORT_LF, PORT_LM, PORT_LB},    // Left Chassis Ports
    {-PORT_RF, PORT_RM, PORT_RB},  // Right Chassis Ports

    PORT_IMU,                                                          // IMU Port
    WHEEL_DIAMETER,                                              // Wheel Diameter
    DRIVE_RPM
);
    
// if you have odom, uncomment the lines below

//#define PORT_ODOM_HORI      0
//#define PORT_ODOM_VERT      0
//#define ODOM_DIAMETER       0
//#define OFFSET_VERT         0
//#define OFFSET_HORI         0
//inline pros::Rotation odom_horiz        (PORT_ODOM_HORI);
//inline pros::Rotation odom_vert         (PORT_ODOM_VERT);
//inline ez::tracking_wheel horiz_tracker(PORT_ODOM_HORIZ, ODOM_DIAMETER, OFFSET_HORI);
//inline ez::tracking_wheel vert_tracker(PORT_ODOM_VERT, ODOM_DIAMETER, OFFSET_VERT);