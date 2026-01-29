#pragma once

/**
* @file subsystems.hpp
* @brief This file contains the definitions and constructors for the robot's subsystems.
* @details This includes the motors, smartwire devices, three wire port devices, and object constructors.
* @author Ansh Rao - 2145Z
*/

#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "subsystems.hpp"
#pragma region constants

//drive motor ports
#define PORT_MOTOR_L1 0
#define PORT_MOTOR_L2 0
#define PORT_MOTOR_L3 0
#define PORT_MOTOR_R1 0
#define PORT_MOTOR_R2 0
#define PORT_MOTOR_R3 0

//subsystem motor ports
#define PORT_MOTOR_INTAKE1 0
#define PORT_MOTOR_INTAKE2 0

//smartwire ports
#define PORT_IMU        0
#define PORT_ODOM_V     0
#define PORT_ODOM_H     0
#define PORT_DISTNANCE1 0
#define PORT_DISTANCE2  0
#define PORT_OPTICAL    0

//three wire ports
#define PORT_LOADER 0
#define PORT_WING   0


//drive constants
#define TRACK_WIDTH     0
#define DRIVE_DIAMETER  0
#define DRIVE_RPM       0
#define HORI_DRIFT      0

//odom constants
#define ODOM_DIAMETER 0
#define ODOM_OFFSET_V 0
#define ODOM_OFFSET_H 0

//auto speed constants
#define DRIVE_SPEED 0
#define TURN_SPEED  0
#define SWING_SPEED 0

//button constants
#define BUTTON_INTAKE   pros::E_CONTROLLER_DIGITAL
#define BUTTON_OUTTAKE  pros::E_CONTROLLER_DIGITAL
#define BUTTTON_SCORE   pros::E_CONTROLLER_DIGITAL
#define BUTTON_LOADER   pros::E_CONTROLLER_DIGITAL
#define BUTTON_WING     pros::E_CONTROLLER_DIGITAL

#pragma endregion

#pragma region constructors

inline pros::Controller master(pros::E_CONTROLLER_MASTER);
inline pros::Controller partner(pros::E_CONTROLLER_PARTNER);

//drive motor constructors
inline pros::Motor motor_L1(PORT_MOTOR_L1, pros::v5::MotorGears::blue);
inline pros::Motor motor_L2(PORT_MOTOR_L2, pros::v5::MotorGears::blue);
inline pros::Motor motor_L3(PORT_MOTOR_L3, pros::v5::MotorGears::blue);
inline pros::Motor motor_R1(PORT_MOTOR_R1, pros::v5::MotorGears::blue);
inline pros::Motor motor_R2(PORT_MOTOR_R2, pros::v5::MotorGears::blue);
inline pros::Motor motor_R3(PORT_MOTOR_R3, pros::v5::MotorGears::blue);

//drive motorgroup constructors
inline pros::MotorGroup motorgroup_L({PORT_MOTOR_L1, PORT_MOTOR_L2, PORT_MOTOR_L3,});
inline pros::MotorGroup motorgroup_R({PORT_MOTOR_R1, PORT_MOTOR_R2, PORT_MOTOR_R3,});

//subsystem motor constructors
inline pros::Motor motor_intake1(PORT_MOTOR_INTAKE1, pros::v5::MotorGears::blue);
inline pros::Motor motor_intake2(PORT_MOTOR_INTAKE2, pros::v5::MotorGears::blue);

//smartwire device constructors
inline pros::Imu IMU(PORT_IMU);
inline pros::Rotation rotation_V(PORT_ODOM_V);
inline pros::Rotation rotation_H(PORT_ODOM_H);
inline pros::Distance distance1(PORT_DISTNANCE1);
inline pros::Distance distance2(PORT_DISTANCE2);
inline pros::Optical optical(PORT_OPTICAL);

//three wire port device constructors
inline pros::adi::DigitalOut loader(PORT_LOADER);
inline pros::adi::DigitalOut wing(PORT_WING);

//lemlib chassis constructors

inline lemlib::Drivetrain drivetrain(
    &motorgroup_L,
    &motorgroup_R,
    TRACK_WIDTH,
    DRIVE_DIAMETER,
    DRIVE_RPM,
    HORI_DRIFT
);

inline lemlib::TrackingWheel odom_V(&rotation_V, ODOM_DIAMETER, ODOM_OFFSET_V);
inline lemlib::TrackingWheel odom_H(&rotation_H, ODOM_DIAMETER, ODOM_OFFSET_H);

inline lemlib::OdomSensors odomSensors(&odom_V, nullptr, &odom_H, nullptr, &IMU);


// lateral PID controller
inline lemlib::ControllerSettings lat_PID(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
inline lemlib::ControllerSettings ang_PID(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
inline lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lat_PID, // lateral PID settings
                        ang_PID, // angular PID settings
                        odomSensors // odometry sensors
);

#pragma endregion

