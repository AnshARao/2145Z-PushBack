#include "chassis.hpp"
#include <cmath>
#include <cstdlib>
#include <type_traits>
#include "EZ-Template/util.hpp"
#include "main.h"  // IWYU pragma: keep
#include "okapi/api/units/QAngle.hpp"
#include "subsystems.hpp"
#include "lemlib/api.hpp"

/**
 * @file drive.cpp
 * @brief This file contains the drive functions for the robot.
 * @details This includes the internal math functions, function wrappers, and path injection.
 * @author Ansh Rao - 2145Z, with a BIG inspiration from Jordan - 21S
 */

#define PORT_LF -18
#define PORT_LM -19
#define PORT_LB -20
#define PORT_RF 15
#define PORT_RM -16
#define PORT_RB 17
#define PORT_IMU 9

#define DRIVE_DIAMETER      lemlib::Omniwheel::NEW_325
#define TRACK_WIDTH         9.5
#define DRIVE_RPM           450

// Controller constructors
pros::Controller controlla (pros::E_CONTROLLER_MASTER);  // Master controller
pros::Controller controlla2(pros::E_CONTROLLER_PARTNER);  // Partner controller

// Motor constructors
pros::Motor motor_LF     (PORT_LF, pros::v5::MotorGears::blue);
pros::Motor motor_LM     (PORT_LM, pros::v5::MotorGears::blue);
pros::Motor motor_LB     (PORT_LB, pros::v5::MotorGears::blue);
pros::Motor motor_RF     (PORT_RF, pros::v5::MotorGears::blue);
pros::Motor motor_RM     (PORT_RM, pros::v5::MotorGears::blue);
pros::Motor motor_RB     (PORT_RB, pros::v5::MotorGears::blue);


// Drive motorgroup constructors
pros::MotorGroup motorgroup_L({PORT_LF, PORT_LM, PORT_LB,}, pros::v5::MotorGears::blue);
pros::MotorGroup motorgroup_R({PORT_RF, PORT_RM, PORT_RB,}, pros::v5::MotorGears::blue);

// smartwire device constructors
pros::Imu imu(PORT_IMU);

// Chassis constructor
ez::Drive EzChassis(
    {PORT_LF, PORT_LM, PORT_LB},    // Left Chassis Ports
    {PORT_RF, PORT_RM, PORT_RB},  // Right Chassis Ports

    PORT_IMU,   // IMU Port
    DRIVE_DIAMETER, // Wheel Diameter
    DRIVE_RPM); // Wheel RPM

void default_constants() {
  // P, I, D, and Start I
  EzChassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  EzChassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  EzChassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  EzChassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  EzChassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  EzChassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  EzChassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  EzChassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  EzChassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  EzChassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  EzChassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  EzChassis.pid_turn_chain_constant_set(3_deg);
  EzChassis.pid_swing_chain_constant_set(5_deg);
  EzChassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  EzChassis.slew_turn_constants_set(3_deg, 70);
  EzChassis.slew_drive_constants_set(3_in, 70);
  EzChassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  EzChassis.odom_turn_bias_set(0.9);

  EzChassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  EzChassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  EzChassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  EzChassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

// drivetrain settings
lemlib::Drivetrain drivetrain(&motorgroup_L, // left motor group
	&motorgroup_R, // right motor group
	TRACK_WIDTH, // 10 inch track width
	DRIVE_DIAMETER, // using new 4" omnis
	DRIVE_RPM, // drivetrain rpm is 360
	2 // horizontal drift is 2 (for now)
);

// odometry settings
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
  nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
  nullptr, // horizontal tracking wheel 1
  nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
  &imu // inertial sensor
);

lemlib::ControllerSettings lateral_controller(6.5, // proportional gain (kP)
					0, // integral gain (kI)
					20, // derivative gain (kD)
					3, // anti windup
					1, // small error range, in inches
					100, // small error range timeout, in milliseconds
					3, // large error range, in inches
					500, // large error range timeout, in milliseconds
					20 // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(5.0, // proportional gain (kP)
					0, // integral gain (kI)
					36.0, // derivative gain (kD)
					3, // anti windup
					1, // small error range, in inches
					100, // small error range timeout, in milliseconds
					3, // large error range, in inches
					500, // large error range timeout, in milliseconds
					0 // maximum acceleration (slew)
);                         


// create the chassis
lemlib::Chassis LLChassis(drivetrain, // drivetrain settings
lateral_controller, // lateral PID settings
angular_controller, // angular PID settings
sensors // odometry sensors
);

double curve_function(double x, double curve_scale) {
return (powf(2.718, -(curve_scale / 10)) + powf(2.718, (fabs(x) - 127) / 10) * (1 - powf(2.718, -(curve_scale / 10)))) * x;
}

void set_tank(int l_stick, int r_stick) {
	motorgroup_L.move_voltage(l_stick * (12000.0 / 127.0));
	motorgroup_R.move_voltage(r_stick * (12000.0 / 127.0));
}

void tank_drive(double curve) {
	// Put the joysticks through the curve function
	double lYcord = controlla.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
	double rYcord = controlla.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

	double lXCord;
	double rXCord;

	lYcord > 0 ? lXCord = abs(controlla.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)) : lXCord = -abs(controlla.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
	rYcord > 0 ? rXCord = abs(controlla.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) : rXCord = -abs(controlla.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

	int l_stick = curve_function(lYcord + lXCord, curve);
	int r_stick = curve_function(rYcord + rXCord, curve);

	// Set robot to l_stick and r_stick, check joystick threshold, set active brake
	set_tank(l_stick, r_stick);
}

// distance sensors
pros::Distance distanceFront(1);
pros::Distance distanceBack(6);
pros::Distance distanceLeft(5);
pros::Distance distanceRight(8);
// distance offsets
float offsetFront = 6.5;
float offsetBack = 4.0;
float offsetLeft = 3.4;
float offsetRight = 3.4;

void reset(pros::Distance sensor, float sensorOffset, float headingOffset) {

	float halfField = 70.25; // half of the field width in inches

	if (matchState == DISABLED) return; // does not play on brain screen
	// get distance and convert mm to inches
	float dist = sensor.get_distance() / 25.4f; 

	if (dist < 0 || dist > 150) return; // check if the distance is valid

	// get heading and convert degrees to radians
	float heading = (EzChassis.odom_theta_get() * 0.0174533f); // deg→rad, abs

	// if heading is greater than 45 degrees, subtract 45 degrees for cos function
    while (heading > (1.57079632679f / 2.0f)) heading -= 1.57079632679f; // ≤ 45°

	// calculate the distance to reset
	float resetDist = (dist + sensorOffset) * cosf(heading);

	// Direction the sensor is pointing in world frame (0=+Y/top, 90=+X/right, 180=-Y/bottom, 270=-X/left).
	// Use signed angle so quadrant is correct (no fabs).
	float sh = EzChassis.odom_theta_get() + headingOffset;
	int sensorHeading = (int)sh;
	sensorHeading = (sensorHeading % 360 + 360) % 360;

	// determine which wall we're facing and which axis to reset
    bool resettingX = false;
    double wallSign = 1.0;
    
    if (315 <= sensorHeading || sensorHeading <= 45) {
        // Top wall - reset Y position
        resettingX = false;
        wallSign = 1.0;
    }
    else if (45 < sensorHeading && sensorHeading <= 135) {
        // Right wall - reset X position
        resettingX = true;
        wallSign = 1.0;
    }
    else if (135 < sensorHeading && sensorHeading <= 225) {
        // Bottom wall - reset Y position
        resettingX = false;
        wallSign = -1.0;
    }
    else {
        // Left wall - reset X position
        resettingX = true;
        wallSign = -1.0;
    }
	// calculate the new position
	float newPos = wallSign * (halfField - resetDist);

	double theta = EzChassis.odom_theta_get();
	if (resettingX) {
		set_position(newPos, EzChassis.odom_y_get(), theta);
		currentPoint.x = newPos;
	} else {
		set_position(EzChassis.odom_x_get(), newPos, theta);
		currentPoint.y = newPos;
	}
}

void resetFront() {
	reset(distanceFront, offsetFront, 0);
}
void resetBack() {
	reset(distanceBack, offsetBack, 180);
}
void resetLeft() {
	reset(distanceLeft, offsetLeft, 270);
}
void resetRight() {
	reset(distanceRight, offsetRight, 90);
}


Coordinate currentPoint = {0, 0, 0};
vector<Coordinate> autonPath = {};

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

double get_velocity(double voltage) { return (2 * M_PI * (voltage / 127 * EzChassis.drive_rpm_get()) * DRIVE_DIAMETER) / 120; }

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
	
	if(matchState != MatchStates::DISABLED) {
		EzChassis.odom_xyt_set(x, y, t);
		LLChassis.setPose({static_cast<float>(x), static_cast<float>(y), static_cast<float>(t)});
	}
	autonPath.push_back(currentPoint);
}

//
// Wait wrappers
//

void wait(Wait type) {
	switch (matchState) {
		case AUTO:
			switch (type) {
				case WAIT:
					EzChassis.pid_wait();
					break;
				case QUICK:
					EzChassis.pid_wait_quick();
					break;
				case CHAIN:
					EzChassis.pid_wait_quick_chain();
					break;
				case LEMLIB:
					LLChassis.waitUntilDone();
					break;
			}
		default:
			break;
	}
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
			EzChassis.pid_wait_until(target);
			break;
			break;
		default:
			break;
	}
}

void wait_until_LL(double target) {
	switch(matchState) {
		case MatchStates::AUTO:
			LLChassis.waitUntil(target);
			break;
			break;
		default:
			break;
	}
}

//
// Drive set wrappers
//
void set_raw(int speed1, int speed2) {
	drive_directions direction1 = speed1 < 0 ? rev : fwd;
	drive_directions direction2 = speed2 < 0 ? rev : fwd;
	EzChassis.drive_set(speed1, speed2);
	currentPoint.left = speed1 * (direction1 == fwd ? 1 : -1);
	currentPoint.right = speed2 * (direction2 == fwd ? 1 : -1);
	currentPoint.t = currentPoint.t;
	autonPath.push_back(currentPoint);
}

void set_raw(int speed) {
	set_raw(speed, speed);
}

void set_drive(double distance, int speed, bool slew, bool correction) {
	drive_directions direction = distance > 0 ? fwd : rev;
	switch(matchState) {
		case MatchStates::AUTO:
			if (correction == false) {
				EzChassis.pid_drive_set(distance * okapi::inch, speed, slew, correction);
			} else {
				EzChassis.pid_odom_set(distance * okapi::inch, speed, slew);

			}
			currentPoint.x = EzChassis.odom_x_get();
			currentPoint.y = EzChassis.odom_y_get();
			break;
		default:
			break;
	}
	currentPoint = get_point(currentPoint, distance);
	currentPoint.left = speed * (direction == fwd ? 1 : -1);
	currentPoint.right = speed * (direction == fwd ? 1 : -1);
	autonPath.push_back(currentPoint);
}

//
// Turn set wrappers
//	

void set_turn(double theta, int speed, e_angle_behavior behavior, bool slew) {
	switch(matchState) {
		case MatchStates::AUTO:
			EzChassis.pid_turn_set(theta * okapi::degree, speed, behavior, slew);
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

void set_turn(int x, int y, drive_directions direction, int speed, e_angle_behavior behavior, bool slew) {
	switch(matchState) {
		case MatchStates::AUTO:
			EzChassis.pid_turn_set({x * okapi::inch, y * okapi::inch}, direction, speed, behavior, slew);
			break;
		default:
			break;
			currentPoint.t = get_theta({currentPoint.x, currentPoint.y }, {x * 1.0, y * 1.0}, direction);
			currentPoint.x = x;
			currentPoint.y = y;
			currentPoint.left = speed * (direction == fwd ? 1 : -1);
			currentPoint.right = speed * (direction == fwd ? 1 : -1);
			autonPath.push_back(currentPoint);
	}
}

void set_turn_relative(double theta, int speed, e_angle_behavior behavior) {
	switch(matchState) {
		case MatchStates::AUTO:
			theta += EzChassis.odom_theta_get();
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
		case MatchStates::AUTO:
			behavior = (util::turn_shortest(theta, EzChassis.odom_theta_get()) < 0) ? ccw : cw;
			theta += EzChassis.odom_theta_get();
			break;
		default:
			theta += currentPoint.t;
			break;
	}
	fmod(theta, 360);
	if(theta < 0) theta += 360;
	set_turn(theta, speed, behavior);
}

// function to determine if cw or ccw is the shortest way to turn
double turn_shortest(double target, double current) {
	double delta = target - current; // get the difference between the target and current
	delta = fmod(delta + 540.0, 360.0) - 180.0; // wrap around to 0-360 and subtract 180
	return delta;
}

// function to set the turn to heading
void set_turn(float theta, int timeout, lemlib::TurnToHeadingParams params, bool sync) {
	if (matchState != MatchStates::DISABLED) {
		LLChassis.turnToHeading(theta, timeout, params, !sync);
	}

	// determine the direction to turn
	params.direction = (turn_shortest(theta, currentPoint.t) < currentPoint.t) ? 
	lemlib::AngularDirection::CCW_COUNTERCLOCKWISE : lemlib::AngularDirection::CW_CLOCKWISE;
	// if the direction is ccw, multiply the max speed by -1 for the brain screen
	if (params.direction == lemlib::AngularDirection::CCW_COUNTERCLOCKWISE) params.maxSpeed *= -1;

	// set the current point
	currentPoint.t = theta; // set the theta
	currentPoint.left = params.maxSpeed; // set the left speed
	currentPoint.right = -params.maxSpeed; // set the right speed
	currentPoint.behavior = (params.direction == lemlib::AngularDirection::CCW_COUNTERCLOCKWISE) ? ez::ccw : ez::cw;
	autonPath.push_back(currentPoint); // add the current point to the path
}

// function to set the turn to point
void set_turn(float x, float y, int timeout, lemlib::TurnToPointParams params, bool sync) {
	if (matchState != MatchStates::DISABLED) {
		LLChassis.turnToPoint(x, y, timeout, params, !sync);
	}
	currentPoint.x = x; // set the x
	currentPoint.y = y; // set the y
	currentPoint.t = get_theta({currentPoint.x, currentPoint.y}, {x, y},
		 params.forwards ? fwd : rev); // set the theta
	currentPoint.left = params.maxSpeed; // set the left speed
	currentPoint.right = -params.maxSpeed; // set the right speed
	currentPoint.behavior = (params.direction == lemlib::AngularDirection::CCW_COUNTERCLOCKWISE) ? ez::ccw : ez::cw;
	autonPath.push_back(currentPoint); // add the current point to the path
}

//
// Swing set wrappers
//

void set_swing(e_swing side, double theta, double main, double opp, e_angle_behavior behavior) {
	switch(matchState) {
		case MatchStates::AUTO:
			EzChassis.pid_swing_set(side, theta * okapi::degree, main, opp, behavior);
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
	double radius = (v_right + v_left) / (v_right - v_left) * ((double)TRACK_WIDTH / 2);
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
		case MatchStates::AUTO:
			behavior = (util::turn_shortest(theta, EzChassis.odom_theta_get()) < 0) ? ccw : cw;
			break;
		default:
			break;
	}
	set_swing(side, theta, main, opp, behavior);
}

void set_swing(ez::e_swing side, double theta, double main) {
	e_angle_behavior behavior = (util::turn_shortest(theta, currentPoint.t) < 0) ? ccw : cw;
	switch(matchState) {
		case MatchStates::AUTO:
			behavior = (util::turn_shortest(theta, EzChassis.odom_theta_get()) < 0) ? ccw : cw;
			break;
		default:
			break;
	}
	set_swing(side, theta, main, 0, behavior);
}

//
// Move to point wrappers
//

void set_mtp(int x, int y, int speed, drive_directions direction, bool slew) {
	switch(matchState) {
		case AUTO:
			EzChassis.pid_odom_set({{x * okapi::inch, y * okapi::inch}, direction, speed}, slew);
			currentPoint.t = get_theta({currentPoint.x, currentPoint.y }, {x * 1.0, y * 1.0}, direction);
			currentPoint.x = x;
			currentPoint.y = y;
			currentPoint.left = speed * (direction == fwd ? 1 : -1);
			currentPoint.right = speed * (direction == fwd ? 1 : -1);
			autonPath.push_back(currentPoint);
			break;
		default:
			set_turn(get_theta(currentPoint, {x * 1.0, y * 1.0}, direction));
			wait();
			set_drive(get_distance(currentPoint, {x * 1.0, y * 1.0}), speed, slew);
			break;
	}
}

// function to set the point
void set_mtp(float x, float y, int timeout, lemlib::MoveToPointParams params, bool sync) {
	if (matchState != MatchStates::DISABLED) {
		LLChassis.moveToPoint(x, y, timeout, params, !sync); // move to the point
	}
	currentPoint.x = x; // set the x
	currentPoint.y = y; // set the y
	currentPoint.t = get_theta({currentPoint.x, currentPoint.y}, {x, y}, 
		params.forwards ? fwd : rev); // set the theta
	currentPoint.left = params.maxSpeed; // set the left speed
	currentPoint.right = -params.maxSpeed; // set the right speed
	autonPath.push_back(currentPoint); // add the current point to the path
}


void set_boom(int x, int y, int t, int speed, drive_directions direction, bool slew) {
	switch(matchState) {
		case AUTO:
			EzChassis.pid_odom_boomerang_set({{x * okapi::inch, y * okapi::inch, t * okapi::degree}, direction, speed}, slew);
				currentPoint.t = get_theta({currentPoint.x, currentPoint.y }, {x * 1.0, y * 1.0}, direction);
				currentPoint.x = x;
				currentPoint.y = y;
				currentPoint.left = speed * (direction == fwd ? 1 : -1);
				currentPoint.right = speed * (direction == fwd ? 1 : -1);
				autonPath.push_back(currentPoint);
			break;
		default:
			set_turn(get_theta(currentPoint, {x * 1.0, y * 1.0}, direction));
			wait();
			set_drive(get_distance(currentPoint, {x * 1.0, y * 1.0}), speed, slew);
			wait();
			set_turn(t, speed);
			break;
	}
}

void set_boom(float x, float y, float theta, int timeout, lemlib::MoveToPoseParams params, bool sync) {
	if (matchState != MatchStates::DISABLED) {
		LLChassis.moveToPose(x, y, theta, timeout, params, !sync); // move to the pose
	}
	currentPoint.x = x; // set the x
	currentPoint.y = y; // set the y
	currentPoint.t = theta; // set the theta
	currentPoint.left = params.maxSpeed; // set the left speed
	currentPoint.right = -params.maxSpeed; // set the right speed
	autonPath.push_back(currentPoint); // add the current point to the path
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