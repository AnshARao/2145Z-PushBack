#include "drive.hpp"
#include <cmath>
#include <cstdlib>
#include <type_traits>
#include "controls.hpp"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/util.hpp"
#include "main.h"  // IWYU pragma: keep
#include "subsystems.hpp"

/**
 * @file drive.cpp
 * @brief This file contains the drive functions for the robot.
 * @details This includes the internal math functions, function wrappers, and path injection.
 * @author Ansh Rao - 2145Z, with a BIG inspiration from Jordan - 21S
 */

Coordinate currentPoint = {0, 0, 0};
std::vector<Coordinate> autonPath = {};

//
// Internal math
//

double get_distance(Coordinate point1, Coordinate point2) {
	double errorX = point2.x - point1.x;
	double errorY = point2.y - point1.y;
	return sqrt((errorX * errorX) + (errorY * errorY));
}

double get_theta(Coordinate point1, Coordinate point2, bool fwd) {
	auto new_direction = fwd == false ? 180 : 0;
	double errorX = point2.x - point1.x;
	double errorY = point2.y - point1.y;
	double theta = (atan2(errorX, errorY) * 180 / M_PI) + new_direction;
	theta = fmod(theta, 360);
	if(theta < 0) theta += 360;
	return theta;
}

double get_velocity(double voltage) { return (2 * M_PI * (voltage / 127 * DRIVE_RPM) * DRIVE_DIAMETER) / 120; }

double get_time_point(double distance, double velocity) { return distance / velocity; }

Coordinate get_point(Coordinate startPoint, double distance) {
	// Get the x and y error between the new point and the current point
	double errorX = distance * (sin(startPoint.t * M_PI / 180));
	double errorY = distance * (cos(startPoint.t * M_PI / 180));

	// Add the error to the start point to create the end point
	Coordinate endPoint = startPoint;
	endPoint.x += errorX;
	endPoint.y += errorY;

	return endPoint;
}

Coordinate get_point(Coordinate startPoint, double v_left, double v_right, double time) {
	// Get the coordinate within the reference frame of the robot of the end point
	double radius = (v_right + v_left) / (v_right - v_left) * ((double)TRACK_WIDTH / 2);
	double theta = ((v_right - v_left) / TRACK_WIDTH * time) + (startPoint.t * M_PI / 180);

	double relative_x = -((-radius * cos(theta) + radius) - (-radius * cos(startPoint.t * M_PI / 180) + radius));
	double relative_y = -((radius * sin(theta)) - (radius * sin(startPoint.t * M_PI / 180)));

	theta *= 180 / M_PI;
	theta = fmod(theta, 360);
	if(theta < 0) theta += 360;

	Coordinate point_relative = {relative_x, relative_y, theta};

	// Translate the point's x and y values by the start point's x and y values
	point_relative.x += startPoint.x;
	point_relative.y += startPoint.y;

	return point_relative;
}

std::vector<Coordinate> injectPoint(Coordinate startPoint, Coordinate endPoint, AngularDirection behavior, double left, double right, double theta,
									double lookAhead) {
	// Make sure theta is positive
	if(startPoint.t < 0) startPoint.t += 360;

	// Get wheel velocities and proper time
	double v_left = get_velocity(left);
	double v_right = get_velocity(right);
	double v_all = (v_left + v_right) / 2;
	if(v_all == 0) v_all = v_left;

	double time = abs(get_time_point(lookAhead, v_all));

	std::vector<Coordinate> pointsBar;
	Coordinate newPoint = startPoint;
	double iter = 0;

	theta = fmod(theta, 360);
	if(theta < 0) theta += 360;

	if(left != KEY) {
		if(left != right) {
			// Make sure the robot travels in the correct direction
			if(left == -right)
				time *= -1;
			else if(((left > right && behavior == lemlib::AngularDirection::CW_CLOCKWISE) || (right > left && behavior == lemlib::AngularDirection::CCW_COUNTERCLOCKWISE)))
				time *= -1;
			// Inject points along curve
			while(!(newPoint.t > theta - abs((v_right - v_left) / TRACK_WIDTH * time * 180 / M_PI) &&
					newPoint.t < theta + abs((v_right - v_left) / TRACK_WIDTH * time * 180 / M_PI))) {
				newPoint = get_point(startPoint, v_left, v_right, iter);
				newPoint.left = left;
				newPoint.right = right;
				iter += time;
				pointsBar.push_back(newPoint);
			}
		} else {
			// Set direction
			if(left < 0) lookAhead *= -1;

			// Inject points along straight line
			while(get_distance(startPoint, newPoint) < get_distance(startPoint, endPoint)) {
				newPoint = get_point(startPoint, iter);
				newPoint.left = left;
				newPoint.right = right;
				iter += lookAhead;
				pointsBar.push_back(newPoint);
			}
		}
	} else
		pointsBar.push_back(endPoint);

	return pointsBar;
}

std::vector<Coordinate> injectPath(std::vector<Coordinate> coordList, double lookAhead) {
	if(coordList.size() > 1) {
		std::vector<Coordinate> injectedList = {};
		for(int i = 0; i < coordList.size() - 1; i++) {
			std::vector<Coordinate> segList = injectPoint(coordList[i], coordList[i + 1], coordList[i + 1].behavior, coordList[i + 1].left,
														  coordList[i + 1].right, coordList[i + 1].t, lookAhead);
			injectedList.insert(injectedList.end(), segList.begin(), segList.end());
		}
		injectedList.push_back(coordList.back());
		return injectedList;
	}
	return coordList;
}

//
// Set position wrappers
//

void set_position(double x, double y) {
	currentPoint.x = x;
	currentPoint.y = y;
	currentPoint.t = currentPoint.t;
	
	if(matchState != MatchStates::DISABLED) chassis.setPose(currentPoint.x, currentPoint.y, currentPoint.t);
	autonPath.push_back(currentPoint);
}

void set_position(double x, double y, double t) {
	currentPoint.x = x;
	currentPoint.y = y;
	currentPoint.t = t;
	
	if(matchState != MatchStates::DISABLED) chassis.setPose(currentPoint.x, currentPoint.y, t);
	autonPath.push_back(currentPoint);
}

//
// Wait wrappers
//

void wait() {
	chassis.waitUntilDone();
}

void wait(int millis, bool ignore) {
	switch(matchState) {
		case MatchStates::AUTO:
			pros::delay(millis);
			break;
		default:
			break;
	}
	if(!ignore) {
	currentPoint.left = KEY;
	currentPoint.right = millis;
	autonPath.push_back(currentPoint);
	}
}

void wait_until(double target) {
	switch(matchState) {
		case MatchStates::AUTO:
			chassis.waitUntil(target);
			break;
		default:
			break;
	}
}

//
// Move to point wrappers
//

void set_mtp(Coordinate newpoint, float speed, int timeout, bool fwd, bool slew) {
	switch(matchState) {
		case AUTO:
			chassis.moveToPoint(newpoint.x, newpoint.y, timeout, {.forwards = fwd, .maxSpeed = speed}); 
			currentPoint.t = get_theta({currentPoint.x, currentPoint.y}, newpoint, fwd);
			currentPoint.x = newpoint.x;
			currentPoint.y = newpoint.y;
			currentPoint.left = speed * (fwd == true ? 1 : -1);
			currentPoint.right = speed * (fwd == true ? 1 : -1);
			autonPath.push_back(currentPoint);
			break;
		default:
			set_turn(get_theta(currentPoint, newpoint, fwd));
			wait();
			set_drive(get_distance(currentPoint, newpoint), speed);
			break;
	}
}

void set_boom(Coordinate newpoint, int speed, int timeout, bool fwd, bool slew) {
	switch(matchState) {
		case AUTO:
				chassis.moveToPose(newpoint.x, newpoint.y, newpoint.t, timeout);		
				currentPoint.t = get_theta({currentPoint.x, currentPoint.y}, newpoint, fwd);
				currentPoint.x = newpoint.x;
				currentPoint.y = newpoint.y;
				currentPoint.left = speed * (fwd == true ? 1 : -1);
				currentPoint.right = speed * (fwd == true ? 1 : -1);
				autonPath.push_back(currentPoint);
			break;
		default:
			set_turn(get_theta(currentPoint, newpoint, fwd));
			wait();
			set_drive(get_distance(currentPoint, newpoint), speed, slew);
			wait();
			set_turn(newpoint.t, speed);
			break;
	}
}

void set_turn_drive(Coordinate newpoint, float speed, int timeout, bool fwd) {
	chassis.straightlineToPoint(newpoint.x, newpoint.y, timeout, {.forwards = fwd, .maxSpeed = speed});
	currentPoint.t = get_theta({currentPoint.x, currentPoint.y}, newpoint, fwd);
			currentPoint.x = newpoint.x;
			currentPoint.y = newpoint.y;
			currentPoint.left = speed * (fwd == true ? 1 : -1);
			currentPoint.right = speed * (fwd == true ? 1 : -1);
			autonPath.push_back(currentPoint);
}

//
// Drive set wrappers
//
void set_drive(int speed) {
	bool fwd = speed < 0 ? false : true;
	motorgroup_L.move(speed);
	motorgroup_R.move(speed);
	currentPoint.left = speed * (fwd == true ? 1 : -1);
	currentPoint.right = speed * (fwd == true ? 1 : -1);
	currentPoint.t = currentPoint.t;
	autonPath.push_back(currentPoint);
}

void set_drive(double distance, int speed, int timeout, bool slew, bool correction) {
	bool fwd = distance > 0 ? true : false;
			float legX = distance * sin(currentPoint.t);
			float legY = distance * cos(currentPoint.t);

			if (correction == false) {
				chassis.moveToPoint(currentPoint.x + legX, currentPoint.y + legY, timeout);
			} else {
				chassis.moveToPose(currentPoint.x + legX, currentPoint.y + legY, currentPoint.t, timeout);

			}
			currentPoint.x = chassis.getPose().x;
			currentPoint.y = chassis.getPose().y;

	currentPoint = get_point(currentPoint, distance);
	currentPoint.left = speed * (fwd == true ? 1 : -1);
	currentPoint.right = speed * (fwd == true ? 1 : -1);
	autonPath.push_back(currentPoint);
}

//
// Turn set wrappers
//

void set_turn(double theta, int speed, int timeout, AngularDirection behavior, bool slew) {
	switch(matchState) {
		case MatchStates::AUTO:
			chassis.turnToHeading(theta, timeout, {.direction = behavior, .maxSpeed = speed});
			break;
		default:
			break;
	}

	if(behavior == AngularDirection::AUTO) behavior = (lemlib::angleError(theta, currentPoint.t, false) > 0) ? AngularDirection::CW_CLOCKWISE : AngularDirection::CCW_COUNTERCLOCKWISE;

	if(behavior == AngularDirection::CCW_COUNTERCLOCKWISE) speed *= -1;

	currentPoint.t = theta;
	currentPoint.left = speed;
	currentPoint.right = -speed;
	currentPoint.behavior = behavior;
	autonPath.push_back(currentPoint);
}

void set_turn(Coordinate newpoint, bool fwd, int speed, int timeout, AngularDirection behavior, bool slew) {
	switch(matchState) {
		case MatchStates::AUTO:
			chassis.turnToPoint(newpoint.x, newpoint.y, timeout);
			break;
		default:
			break;
			currentPoint.t = get_theta({currentPoint.x, currentPoint.y}, newpoint, fwd);
			currentPoint.x = newpoint.x;
			currentPoint.y = newpoint.y;
			currentPoint.left = speed * (fwd == true ? 1 : -1);
			currentPoint.right = speed * (fwd == true ? 1 : -1);
			autonPath.push_back(currentPoint);
	}
}

//
// Print path
//

void getPath() {
	std::cout << "===========================================" << std::endl;
	for(auto point : autonPath) {
		std::cout << "(" << point.x << ", " << point.y << ")" << std::endl;
	}
	std::cout << "===========================================" << std::endl;
}

void getPathInjected() {
	auto injected = injectPath(autonPath, 2);
	std::cout << "===========================================" << std::endl;
	for(auto point : injected) {
		std::cout << "(" << point.x << ", " << point.y << ")" << std::endl;
	}
	std::cout << "===========================================" << std::endl;
}