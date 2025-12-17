#pragma once

#include "lemlib/chassis/chassis.hpp"
#include "pros/colors.h"
#include "subsystems.hpp"
#include <vector>

const int KEY = 267267;

class Coordinate {
    public: 
        double x = 0;
        double y = 0;
        double t = 0;
        double right = 127;
        double left = 0;
        lemlib::AngularDirection behavior = lemlib::AngularDirection::AUTO;
};

extern Coordinate currentPoint;
extern std::vector<Coordinate> autonPath;

// Internal math
double get_distance(Coordinate point1, Coordinate point2);
double get_theta(Coordinate point1, Coordinate point2, bool fwd);
double get_velocity(double voltage);
double get_time_point(double distance, double velocity);
Coordinate get_point(Coordinate startPoint, double distance);
Coordinate get_point(Coordinate startPoint, double v_left, double v_right, double time);
std::vector<Coordinate> injectPoint(Coordinate startPoint, Coordinate endPoint, lemlib::AngularDirection behavior, double left, double right, double theta, double lookAhead);
std::vector<Coordinate> injectPath(std::vector<Coordinate> coordList, double lookAhead);

// Set position wrappers
void set_position(double x, double y);
void set_position(double x, double y, double t);

enum Wait {
    WAIT = 0,
    QUICK = 1,
    CHAIN = 2
};

// Wait wrappers
void wait();
void wait(int millis, bool ignore = false);
void wait_until(double target);
void wait_until(Coordinate coordinate);

// Move to point wrappers
void set_mtp(Coordinate newpoint, float speed, int timeout = 9999, bool fwd = true, bool slew = false);
void set_boom(Coordinate newpoint, int speed, int timeout = 9999, bool fwd = true, bool slew = false);
void set_turn_drive(Coordinate newpoint, float speed, int timeout = 9999, bool fwd = true);

// Drive set wrappers
void set_drive(double distance, int speed = DRIVE_SPEED, int timeout = 9999, bool slew = false, bool correction = true);
void set_drive(int speed);

// Turn set wrappers
void set_turn(double theta, int speed = TURN_SPEED, int timeout = 9999, lemlib::AngularDirection behavior = lemlib::AngularDirection::AUTO, bool slew = false);
void set_turn(Coordinate newpoint, bool fwd, int speed, int timeout = 9999, lemlib::AngularDirection behavior = lemlib::AngularDirection::AUTO, bool slew = false);
void set_turn_relative(double theta, int speed, lemlib::AngularDirection behavior);
void set_turn_relative(double theta, int speed);

// Print path
void getPath();
void getPathInjected();