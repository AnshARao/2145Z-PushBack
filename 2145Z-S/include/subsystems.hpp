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
#include "robodash/api.h"

inline rd::Console console1;

// declaring global variables
enum MatchStates {DISABLED = 0, AUTO = 1, DRIVER = 2};
inline MatchStates matchState = DISABLED;
enum Alliances {BLUE = 0, NONE = 1, RED = 2};
inline Alliances allianceColor = Alliances::NONE; // for now
enum RollerStates {INTAKE = 0, OUTTAKE = 1, SCORE = 2, SCORE_MID = 3, STOP = 4};

#pragma region constants

//drive motor ports
#define PORT_MOTOR_L1 -6
#define PORT_MOTOR_L2 -19
#define PORT_MOTOR_L3 -16
#define PORT_MOTOR_R1 1
#define PORT_MOTOR_R2 17
#define PORT_MOTOR_R3 20


//subsystem motor ports
#define PORT_MOTOR_INTAKE1 -11
#define PORT_MOTOR_INTAKE2 -3
#define PORT_MOTOR_INTAKE3 -13

//smartwire ports
#define PORT_IMU        21
#define PORT_ODOM_V     12
#define PORT_ODOM_H     14
#define PORT_DISTNANCE1 0
#define PORT_DISTANCE2  0
#define PORT_OPTICAL    0

//three wire ports
#define PORT_LOADER 'H'
#define PORT_WING   'F'
#define PORT_HOOD   'G'

//drive constants
#define TRACK_WIDTH     10.5
#define DRIVE_DIAMETER  3.25
#define DRIVE_RPM       450
#define HORI_DRIFT      2.0

//odom constants
#define ODOM_DIAMETER_V 2.75
#define ODOM_DIAMETER_H 2.0
#define ODOM_OFFSET_V -0.21
#define ODOM_OFFSET_H 0.55

//auto speed constants
#define DRIVE_SPEED 110
#define TURN_SPEED  90
#define SWING_SPEED 110

//drive curve constant
#define DRIVE_CURVE 1.0

//button constants
#define BUTTON_INTAKE   pros::E_CONTROLLER_DIGITAL_R1
#define BUTTON_OUTTAKE  pros::E_CONTROLLER_DIGITAL_R2
#define BUTTON_SCORE   pros::E_CONTROLLER_DIGITAL_L1
#define BUTTON_SCORE_MIDDLE pros::E_CONTROLLER_DIGITAL_B
#define BUTTON_LOADER   pros::E_CONTROLLER_DIGITAL_DOWN
#define BUTTON_WING     pros::E_CONTROLLER_DIGITAL_L2

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
inline pros::Motor motor_intake3(PORT_MOTOR_INTAKE3, pros::v5::MotorGears::blue);

//smartwire device constructors
inline pros::Imu IMU(PORT_IMU);
inline pros::Rotation rotation_V(PORT_ODOM_V);
inline pros::Rotation rotation_H(PORT_ODOM_H);
inline pros::Distance distance1(PORT_DISTNANCE1);
inline pros::Distance distance2(PORT_DISTANCE2);
inline pros::Optical optical(PORT_OPTICAL);

//three wire port device constructors
inline pros::adi::DigitalOut piston_loader(PORT_LOADER);
inline pros::adi::DigitalOut piston_wing(PORT_WING);
inline pros::adi::DigitalOut piston_hood(PORT_HOOD);

//lemlib chassis constructors

inline lemlib::Drivetrain drivetrain(
    &motorgroup_L,
    &motorgroup_R,
    TRACK_WIDTH,
    DRIVE_DIAMETER,
    DRIVE_RPM,
    HORI_DRIFT
);

inline EmaFilter filter(.9);
inline EmaFilter filter2(.9);

inline lemlib::TrackingWheel odom_V(&rotation_V, &filter, ODOM_DIAMETER_V, ODOM_OFFSET_V);
inline lemlib::TrackingWheel odom_H(&rotation_H, &filter2, ODOM_DIAMETER_H, ODOM_OFFSET_H);

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

// input curve for throttle input during driver control
inline lemlib::ExpoDriveCurve lat_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
inline lemlib::ExpoDriveCurve ang_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.03 // expo curve gain
);

// create the chassis
inline lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lat_PID, // lateral PID settings
                        ang_PID, // angular PID settings
                        odomSensors, // odometry sensors
                        &lat_curve,
                        &ang_curve
);

#pragma endregion

#include "lib/lights.hpp"
#include "lib/color.hpp"

inline lib::Color color = lib::Color();

inline std::vector<int> blueGradient = interpolateDouble(HSV(120, 0.8, 0.7), HSV(340, 0.8, 0.7), 120);
inline std::vector<int> redGradient = interpolateDouble(HSV(310, 0.9, 0.7), HSV(400, 0.9, 0.7), 120);
inline std::vector<int> rainbow = interpolateSingle(HSV(0, 0.75, 0.85), HSV(360, 0.75, 0.85), 150);
inline lib::FlowingGradient armBraceLeds('A', 33, redGradient, blueGradient);
inline lib::FlowingGradient intakeLed('B', 33, redGradient, blueGradient); 
inline lib::FlowingGradient clampLed('C', 30, redGradient, blueGradient);
inline lib::FlowingGradient leftDriveLed('D', 40, redGradient, blueGradient);
inline lib::FlowingGradient rightDriveLed('E', 40, redGradient, blueGradient);
//inline lib::Pulser armBraceLeds('A', 33, HSV(0, 0.95, 1), HSV(200, 0.9, 0.95), -0.08);
//inline lib::Pulser intakeLed('B', 33, HSV(0, 0.95, 1), HSV(200, 0.9, 0.95), -0.08);
//inline lib::Pulser clampLed('C', 30, HSV(0, 0.95, 1), HSV(200, 0.9, 0.95), -0.08);
//inline lib::EmaPulser leftDriveLed('D', 40, HSV(0, 0.95, 1), HSV(200, 0.9, 0.95));
//inline lib::EmaPulser rightDriveLed('E', 40, HSV(0, 0.95, 1), HSV(200, 0.9, 0.95));

inline lib::LightManager lights(&armBraceLeds, &intakeLed, &clampLed, &leftDriveLed, &rightDriveLed);