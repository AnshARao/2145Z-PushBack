#pragma once

#include "lemlib/api.hpp"
#include "pros/distance.hpp"
#include "pros/motor_group.hpp"
#include "pros/imu.hpp"
#include "lemlib/chassis/chassis.hpp"
#include <vector>

extern pros::Controller controlla;
extern pros::MotorGroup left_motor_group;
extern pros::MotorGroup right_motor_group;
extern lemlib::Drivetrain drivetrain;
extern pros::Imu imu;
extern lemlib::OdomSensors sensors;
extern lemlib::Chassis chassis;

double curve_function(double x, double curve_scale);
void set_tank(int l_stick, int r_stick);
void tank_drive(double curve);

const int KEY = 267267;

enum class DriveDirection { FWD, REV };

class Coordinate {
    public:
        double x = 0;
        double y = 0;
        double t = 0;
        double right = 127;
        double left = 0;
        lemlib::AngularDirection behavior = lemlib::AngularDirection::CW_CLOCKWISE;
};

extern Coordinate currentPoint;
extern std::vector<Coordinate> autonPath;

// Internal math
double get_distance(Coordinate point1, Coordinate point2);
double get_theta(Coordinate point1, Coordinate point2, DriveDirection direction);
double get_velocity(double voltage);
double get_time_point(double distance, double velocity);
Coordinate get_point(Coordinate startPoint, double distance);
Coordinate get_point(Coordinate startPoint, double v_left, double v_right, double time);
std::vector<Coordinate> injectPoint(Coordinate startPoint, Coordinate endPoint, lemlib::AngularDirection direction, double left, double right, double theta, double lookAhead);
std::vector<Coordinate> injectPath(std::vector<Coordinate> coordList, double lookAhead);

void set_position(double x, double y, double t);

void wait();
void wait(double ms);
void wait_until(float dist);
void wait_until(Coordinate target);

void set_raw(int speed);
void set_drive(float distance, float timeout, lemlib::MoveToPointParams params = {}, bool sync = true);
void set_turn(float theta, int timeout, lemlib::TurnToHeadingParams params = {}, bool async = true);
void set_turn(float x, float y, int timeout, lemlib::TurnToPointParams params = {}, bool async = true);
void set_point(float x, float y, int timeout, lemlib::MoveToPointParams params = {}, bool sync = true);
void set_pose(float x, float y, float theta, int timeout, lemlib::MoveToPoseParams params = {}, bool sync = true);
void set_swing(float theta, DriveSide lockedSide, int timeout, lemlib::SwingToHeadingParams params = {},bool sync = true);
void set_swing(float x, float y, DriveSide lockedSide, int timeout, lemlib::SwingToPointParams params = {}, bool sync = true);

void resetLeft();
void resetRight();
void resetBack();
void resetFront();

extern pros::Distance distanceFront;
extern pros::Distance distanceBack;
extern pros::Distance distanceLeft;
extern pros::Distance distanceRight;