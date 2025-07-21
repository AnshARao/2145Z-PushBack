#include "drive.hpp"
#include "controls.hpp"
#include "main.h"  // IWYU pragma: keep
#include "okapi/api/units/QAngle.hpp"
#include "subsystems.hpp"

/**
 * @file drive.cpp
 * @brief This file contains the drive functions for the robot.
 * @details This includes the internal math functions, function wrappers, and path injection.
 * @author Ansh Rao - 2145Z, with a BIG inspiration from Jordan - 21S
 */

Coordinate currentPoint = {0, 0, 0};
vector<Coordinate> autonPath = {};

void flip() {
	currentPoint.x = -currentPoint.x;
	currentPoint.y = -currentPoint.y;
	currentPoint.t = 360 -currentPoint.t;
	if (currentPoint.t == 0) {
		currentPoint.t = 180;
		return;
	}
	if (currentPoint.t == 180) {
		currentPoint.t = 0;
		return;
	}
	return;
}

//
// Internal math
//

double get_distance(Coordinate point1, Coordinate point2) {
	double errorX = point2.x - point1.x;
	double errorY = point2.y - point1.y;
	return sqrt((errorX * errorX) + (errorY * errorY));
}

double get_theta(Coordinate point1, Coordinate point2, drive_directions direction) {
	auto new_direction = direction == rev ? 180 : 0;
	double errorX = point2.x - point1.x;
	double errorY = point2.y - point1.y;
	double theta = (atan2(errorX, errorY) * 180 / M_PI) + new_direction;
	theta = fmod(theta, 360);
	if(theta < 0) theta += 360;
	return theta;
}

double get_velocity(double voltage) { return (2 * M_PI * (voltage / 127 * chassis.drive_rpm_get()) * WHEEL_DIAMETER) / 120; }

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
	double radius = (v_right + v_left) / (v_right - v_left) * (TRACK_WIDTH / 2);
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

std::vector<Coordinate> injectPoint(Coordinate startPoint, Coordinate endPoint, e_angle_behavior behavior, double left, double right, double theta,
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
			else if(((left > right && behavior == cw) || (right > left && behavior == ccw)))
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

void set_position(double x, double y, double t) {
	currentPoint.x = x;
	currentPoint.y = y;
	currentPoint.t = t;
	
	if(matchState != MatchStates::DISABLED) chassis.odom_xyt_set(currentPoint.x, currentPoint.y, t);
	autonPath.push_back(currentPoint);
}

void set_position(double x, double y) {
	currentPoint.x = x;
	currentPoint.y = y;	
	currentPoint.t = 0;
	if(matchState != MatchStates::DISABLED) chassis.odom_xyt_set(currentPoint.x, currentPoint.y, currentPoint.t);
	autonPath.push_back(currentPoint);
}

//
// Wait wrappers
//

void wait(Wait type) {
	switch(matchState) {
		case MatchStates::AUTO_PID:
		case MatchStates::AUTO_ODOM:
			switch(type) {
				case Wait::QUICK:
					chassis.pid_wait_quick();
					break;
				case Wait::CHAIN:
					chassis.pid_wait_quick_chain();
					break;
				default:
					chassis.pid_wait();
					break;
			}
			break;
		default:
			break;
	}
}

void wait_until(okapi::QLength distance) {
	switch(matchState) {
		case MatchStates::AUTO_PID:
		case MatchStates::AUTO_ODOM:
			chassis.pid_wait_until(distance);
			break;
		default:
			break;
	}
}

void wait_until(okapi::QAngle theta) {
	switch(matchState) {
		case MatchStates::AUTO_PID:
		case MatchStates::AUTO_ODOM:
			chassis.pid_wait_until(theta);
			break;
		default:
			break;
	}
}

void wait_until(double target) {
	switch(matchState) {
		case MatchStates::AUTO_PID:
		case MatchStates::AUTO_ODOM:
			chassis.pid_wait_until(target);
			break;
		default:
			break;
	}
}

void wait_until(Coordinate coordinate) {
	switch(matchState) {
		case MatchStates::AUTO_PID:
		case MatchStates::AUTO_ODOM:
			chassis.pid_wait_until({coordinate.x * okapi::inch, coordinate.y * okapi::inch});
			break;
		default:
			break;
	}
}

void wait(int millis, bool ignore) {
	switch(matchState) {
		case MatchStates::AUTO_PID:
		case MatchStates::AUTO_ODOM:
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

void wait(int millis) {
	wait(millis, false);
}

//
// Move to point wrappers
//

void move_point(Coordinate newpoint, drive_directions direction, int speed) {
	bool slew_state = false;
	switch(matchState) {
		case MatchStates::AUTO_ODOM:
			chassis.pid_odom_set({{newpoint.x * okapi::inch, newpoint.y * okapi::inch}, direction, speed});
			currentPoint.t = get_theta({currentPoint.x, currentPoint.y}, newpoint, direction);
			currentPoint.x = newpoint.x;
			currentPoint.y = newpoint.y;
			currentPoint.left = speed * (direction == fwd ? 1 : -1);
			currentPoint.right = speed * (direction == fwd ? 1 : -1);
			autonPath.push_back(currentPoint);
			break;
		default:
			set_turn(get_theta(currentPoint, newpoint, direction), speed);
			wait(Wait::CHAIN);
			set_drive((get_distance(currentPoint, newpoint) * (direction == rev ? -1 : 1)), speed);
			break;
	}
}

//
// Drive set wrappers
//

void set_drive(double distance, int speed, bool slew) {
	drive_directions direction = distance < 0 ? rev : fwd;
	switch(matchState) {
		case MatchStates::AUTO_PID:
			chassis.pid_drive_set(distance * okapi::inch, speed, slew);
			currentPoint.x = chassis.odom_x_get();
			currentPoint.y = chassis.odom_y_get();
			break;
		case MatchStates::AUTO_ODOM:
			chassis.pid_odom_set(distance * okapi::inch, speed, slew);
			currentPoint.x = chassis.odom_x_get();
			currentPoint.y = chassis.odom_y_get();
			break;
		default:
			break;
	}
	currentPoint = get_point(currentPoint, distance);
	currentPoint.left = speed * (direction == fwd ? 1 : -1);
	currentPoint.right = speed * (direction == fwd ? 1 : -1);
	autonPath.push_back(currentPoint);
}

void set_drive(double distance, int speed) {
	bool slew = abs(distance) > 48 ? true : false;
	set_drive(distance, speed, slew);
}

//
// Turn set wrappers
//

void set_turn(double theta, int speed, e_angle_behavior behavior) {
	switch(matchState) {
		case MatchStates::AUTO_PID:
		case MatchStates::AUTO_ODOM:
			//allianceColor == RED ? theta = theta : theta = theta - 180; // auto flip based on color
			chassis.pid_turn_set(theta * okapi::degree, speed, behavior);
			break;
		default:
			break;
	}

	if(behavior == shortest) behavior = (util::turn_shortest(theta, currentPoint.t) < currentPoint.t) ? ccw : cw;

	if(behavior == ccw) speed *= -1;

	currentPoint.t = theta;
	currentPoint.left = speed;
	currentPoint.right = -speed;
	currentPoint.behavior = behavior;
	autonPath.push_back(currentPoint);
}

void set_turn(double theta, int speed) {
	e_angle_behavior behavior = (util::turn_shortest(theta, currentPoint.t) < currentPoint.t) ? ccw : cw;
	switch(matchState) {
		case MatchStates::AUTO_PID:
		case MatchStates::AUTO_ODOM:
			behavior = (util::turn_shortest(theta, chassis.odom_theta_get()) < chassis.odom_theta_get()) ? ccw : cw;
			break;
		default:
			break;
	}
	set_turn(theta, speed, behavior);
}

void set_turn(Coordinate point, drive_directions direction, int speed, e_angle_behavior behavior) {
	double theta = get_theta(currentPoint, point, direction);
	switch(matchState) {
		case MatchStates::AUTO_PID:
		case MatchStates::AUTO_ODOM:
			theta = get_theta({chassis.odom_x_get(), chassis.odom_y_get(), chassis.odom_theta_get()}, point, direction);
			break;
		default:
			break;
	}
	set_turn(theta, speed, behavior);
}

void set_turn(Coordinate point, drive_directions direction, int speed) {
	double theta = get_theta(currentPoint, point, direction);
	e_angle_behavior behavior = (util::turn_shortest(theta, currentPoint.t) < currentPoint.t) ? ccw : cw;
	switch(matchState) {
		case MatchStates::AUTO_PID:
		case MatchStates::AUTO_ODOM:
			behavior = (util::turn_shortest(theta, chassis.odom_theta_get()) < chassis.odom_theta_get()) ? ccw : cw;
			theta = get_theta({chassis.odom_x_get(), chassis.odom_y_get(), chassis.odom_theta_get()}, point, direction);
			break;
		default:
			break;
	}
	set_turn(theta, speed, behavior);
}

void set_turn_relative(double theta, int speed, e_angle_behavior behavior) {
	switch(matchState) {
		case MatchStates::AUTO_PID:
		case MatchStates::AUTO_ODOM:
			theta += chassis.odom_theta_get();
			break;
		default:
			theta += currentPoint.t;
			break;
	}
	fmod(theta, 360);
	if(theta < 0) theta += 360;
	set_turn(theta, speed, behavior);
}

void set_turn_relative(double theta, int speed) {
	e_angle_behavior behavior = (util::turn_shortest(theta, currentPoint.t) < 0) ? ccw : cw;
	switch(matchState) {
		case MatchStates::AUTO_PID:
		case MatchStates::AUTO_ODOM:
			behavior = (util::turn_shortest(theta, chassis.odom_theta_get()) < 0) ? ccw : cw;
			theta += chassis.odom_theta_get();
			break;
		default:
			theta += currentPoint.t;
			break;
	}
	fmod(theta, 360);
	if(theta < 0) theta += 360;
	set_turn(theta, speed, behavior);
}

//
// Swing set wrappers
//

void set_swing(e_swing side, double theta, double main, double opp, e_angle_behavior behavior) {
	switch(matchState) {
		case MatchStates::AUTO_PID:
		case MatchStates::AUTO_ODOM:
			chassis.pid_swing_set(side, theta * okapi::degree, main, opp, behavior);
			break;
		default:
			break;
	}

	// Convert main/opposite voltages to left/right voltages
	double right = side == RIGHT_SWING ? main : opp;
	double left = side == LEFT_SWING ? main : opp;

	// Convert voltage to velocity
	double v_left = get_velocity(left);
	double v_right = get_velocity(right);
	double v_all = (v_left + v_right) / 2;

	// Get radius and arc length
	double new_t = theta - currentPoint.t;
	fmod(new_t, 360);
	if(new_t < 0) new_t += 360;
	double radius = (v_right + v_left) / (v_right - v_left) * (TRACK_WIDTH / 2);
	double arcLength = radius * new_t * M_PI / 180;

	currentPoint = get_point(currentPoint, v_left, v_right, get_time_point(arcLength, v_all));

	currentPoint.left = left;
	currentPoint.right = right;
	currentPoint.behavior = behavior;
	autonPath.push_back(currentPoint);
}

void set_swing(ez::e_swing side, double theta, double main, ez::e_angle_behavior behavior) { set_swing(side, theta, main, 0, behavior); }

void set_swing(ez::e_swing side, double theta, double main, double opp) {
	e_angle_behavior behavior = (util::turn_shortest(theta, currentPoint.t) < 0) ? ccw : cw;
	switch(matchState) {
		case MatchStates::AUTO_PID:
		case MatchStates::AUTO_ODOM:
			behavior = (util::turn_shortest(theta, chassis.odom_theta_get()) < 0) ? ccw : cw;
			break;
		default:
			break;
	}
	set_swing(side, theta, main, opp, behavior);
}

void swingSet(ez::e_swing side, double theta, double main) {
	e_angle_behavior behavior = (util::turn_shortest(theta, currentPoint.t) < 0) ? ccw : cw;
	switch(matchState) {
		case MatchStates::AUTO_PID:
		case MatchStates::AUTO_ODOM:
			behavior = (util::turn_shortest(theta, chassis.odom_theta_get()) < 0) ? ccw : cw;
			break;
		default:
			break;
	}
	set_swing(side, theta, main, 0, behavior);
}

//
// Print path
//

void getPath() {
	cout << "===========================================" << endl;
	for(auto point : autonPath) {
		cout << "(" << point.x << ", " << point.y << ")" << endl;
	}
	cout << "===========================================" << endl;
}

void getPathInjected() {
	auto injected = injectPath(autonPath, 2);
	cout << "===========================================" << endl;
	for(auto point : injected) {
		cout << "(" << point.x << ", " << point.y << ")" << endl;
	}
	cout << "===========================================" << endl;
}