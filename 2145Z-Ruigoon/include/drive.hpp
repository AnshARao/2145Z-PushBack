#pragma once

#include "EZ-Template/api.hpp"
#include "EZ-Template/util.hpp"
#include "drive.hpp"
#include "pros/colors.h"
#include "subsystems.hpp"

const int KEY = 267267;

class Coordinate {
    public: 
        double x = 0;
        double y = 0;
        double t = 0;
        double right = 127;
        double left = 0;
        ez::e_angle_behavior behavior = ez::cw;
};

extern Coordinate currentPoint;
extern vector<Coordinate> autonPath;

// Internal math
double get_distance(Coordinate point1, Coordinate point2);
double get_theta(Coordinate point1, Coordinate point2, ez::drive_directions direction);
double get_velocity(double voltage);
double get_time_point(double distance, double velocity);
Coordinate get_point(Coordinate startPoint, double distance);
Coordinate get_point(Coordinate startPoint, double v_left, double v_right, double time);
std::vector<Coordinate> injectPoint(Coordinate startPoint, Coordinate endPoint, e_angle_behavior behavior, double left, double right, double theta, double lookAhead);
std::vector<Coordinate> injectPath(std::vector<Coordinate> coordList, double lookAhead);

// Set position wrappers
void set_position(double x, double y);
void set_position(double x, double y, double t = 0);

enum Wait {
    WAIT = 0,
    QUICK = 1,
    CHAIN = 2
};

// Wait wrappers
void wait(Wait type = WAIT);
void wait(int millis, bool ignore = false);
void wait_until(double target);
void wait_until(Coordinate coordinate);

// Move to point wrappers
void set_mtp(Coordinate newpoint, int speed, ez::drive_directions direction = fwd, bool slew = false);
void set_boom(Coordinate newpoint, int speed, ez::drive_directions direction = fwd, bool slew = false);

// Drive set wrappers
void set_drive(double distance, int speed = DRIVE_SPEED, bool slew = false, bool correction = true);
void set_drive(int speed);

// Turn set wrappers
void set_turn(double theta, int speed = TURN_SPEED, e_angle_behavior behavior = shortest, bool slew = false);
void set_turn(Coordinate point, drive_directions direction, int speed, e_angle_behavior behavior = shortest, bool use_slew = false);
void set_turn_relative(double theta, int speed, e_angle_behavior behavior);
void set_turn_relative(double theta, int speed);

// Swing set wrappers
void set_swing(ez::e_swing side, double theta, double main, double opp, ez::e_angle_behavior behavior);
void set_swing(ez::e_swing side, double theta, double main, ez::e_angle_behavior behavior);
void set_swing(ez::e_swing side, double theta, double main, double opp);
void set_swing(ez::e_swing side, double theta, double main);

// Print path
void get_path();
void get_path_injected();