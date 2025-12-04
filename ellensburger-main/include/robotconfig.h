#pragma once
#include "lemlib/api.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lib/lights.hpp"
#include "main.h"
#include "lib/color.hpp"
#include "lib/intake.hpp"
#include "lib/lights.hpp"
#include "lib/lift.hpp"
#include "pros/abstract_motor.hpp"
#include "robodash/api.h"
#include "pros/adi.hpp"
#include "pros/optical.h"
#include "pros/rotation.hpp"
#include "robodash/views/selector.hpp"
#include "robotconfig.h"
#include <iostream>
#include <vector>

enum class team {red, blue, none};
inline team teamColor = team::none;
inline std::optional<rd::Selector::routine_t> current_auto;
inline bool liftButtonHeld = false;

// controller
inline pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
inline pros::MotorGroup leftMotors({-1, 5, -3}, pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
inline pros::MotorGroup rightMotors({13, 14, -15}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 10
inline pros::Imu imu(11);

inline pros::Distance dist = pros::Distance(3);

// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
inline pros::Rotation horizontal(-17);
inline EmaFilter filter(.9);

inline pros::Rotation vertical(-16);
inline EmaFilter filter2(.9);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
inline lemlib::TrackingWheel verticalwheel(&vertical, &filter2, 2.015, 0);
inline lemlib::TrackingWheel horizontalwheel(&horizontal, &filter, 2.028, 1.5);

// drivetrain settings
inline lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              6 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
inline lemlib::ControllerSettings linearController(6.125, // proportional gain (kP)
                                            0.01, // integral gain (kI)
                                            1.25, // derivative gain (kD)
                                            3, // anti windup
                                            .5, // small error range, in inches
                                            35, // small error range timeout, in milliseconds
                                            1.0, // large error range, in inches
                                            160, // large error range timeout, in milliseconds
                                            7 // maximum acceleration (slew)
);

// angular motion controller
inline lemlib::ControllerSettings angularController(4.6, // proportional gain (kP)
                                             0.01, // integral gain (kI)
                                             52, // derivative gain (kD)
                                             3, // anti windup
                                             1.5, // small error range, in degrees
                                             35, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             100, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
inline lemlib::OdomSensors sensors(&verticalwheel, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontalwheel, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
inline lemlib::ExpoDriveCurve throttleCurve(0, // joystick deadband out of 127
                                     0, // minimum output where drivetrain will move out of127
                                     1.0005 // expo curve gain
);

// input curve for steer input during driver control
inline lemlib::ExpoDriveCurve steerCurve(0, // joystick deadband out of 127
                                  0, // minimum output where drivetrain will move out of 127
                                  1.0005 // expo curve gain
);

// create the chassis
inline lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);


inline lib::Color color = lib::Color();

inline pros::MotorGroup intakeMotor({-20});
inline lib::Intake intake(&intakeMotor, &color);

inline pros::MotorGroup armMotors({-18,19}, pros::v5::MotorGears::red);
inline pros::Rotation armrot(9);
inline lib::Lift lift = lib::Lift(&armMotors, &armrot);

inline pros::adi::Pneumatics rightDoinker('H', false);
inline pros::adi::Pneumatics leftDoinker('G', false);
inline pros::adi::Pneumatics clamp('F', false);


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