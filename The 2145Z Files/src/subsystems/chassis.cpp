#include "lemlib/chassis/chassis.hpp"
#include "fmt/format.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "main.h"
#include "pros/distance.hpp"
#include <cmath>
#include <vector>

pros::Controller controlla(pros::E_CONTROLLER_MASTER);
// imu
pros::Imu imu(10);

// left motor group
pros::MotorGroup left_motor_group({-11, -12, 13}, pros::MotorGears::blue);
// right motor group
pros::MotorGroup right_motor_group({18, -19, 20}, pros::MotorGears::blue);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              9.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// odometry settings
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              25, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              13, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);                           


// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

double curve_function(double x, double curve_scale) {
    return (powf(2.718, -(curve_scale / 10)) + powf(2.718, (fabs(x) - 127) / 10) * (1 - powf(2.718, -(curve_scale / 10)))) * x;
}

void set_tank(int l_stick, int r_stick) {
    left_motor_group.move_voltage(l_stick * (12000.0 / 127.0));

    right_motor_group.move_voltage(r_stick * (12000.0 / 127.0));
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

pros::Distance distanceFront(5);
pros::Distance distanceBack(1);
pros::Distance distanceLeft(3);
pros::Distance distanceRight(9);

float offsetFront = 0;
float offsetBack = 4.0;
float offsetLeft = 4.0;
float offsetRight = 4.0;

float halfField = 70.25;

void reset(pros::Distance sensor, float sensorOffset, float headingOffset) {
	if (curMatchState == DISABLED) {
		return;
	}
	// convert mm to inches
	float dist = sensor.get_distance() / 25.4f;
	// check if the distance is valid
	if (dist < 0 || dist > 150) {
		print("invalid distance sensor reading");
		return;
	}
	float heading = (chassis.getPose().theta * 0.0174533f); // deg→rad, abs
    while (heading > (1.57079632679f / 2.0f)) heading -= 1.57079632679f; // ≤ 45°

	float resetDist = (dist + sensorOffset) * cosf(heading);

	float sensorHeading = fabs(chassis.getPose().theta + headingOffset);
	sensorHeading = (int)(sensorHeading + 360) % 360;

	// Determine which wall we're facing and which axis to reset
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

	float newPos = wallSign * (halfField - resetDist);

	if (resettingX) {
		chassis.setPose(newPos, chassis.getPose().y, chassis.getPose().theta);
	} else {
		chassis.setPose(chassis.getPose().x, newPos, chassis.getPose().theta);
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
std::vector<Coordinate> autonPath = {};

//
// Internal math
//

double get_distance(Coordinate point1, Coordinate point2) {
	double errorX = point2.x - point1.x;
	double errorY = point2.y - point1.y;
	return sqrt((errorX * errorX) + (errorY * errorY));
}

double get_theta(Coordinate point1, Coordinate point2, DriveDirection direction) {
	auto new_direction = direction == DriveDirection::REV ? 180 : 0;
	double errorX = point2.x - point1.x;
	double errorY = point2.y - point1.y;
	double theta = (atan2(errorX, errorY) * 180 / M_PI) + new_direction;
	theta = fmod(theta, 360);
	if(theta < 0) theta += 360;
	return theta;
}

double get_velocity(double voltage) { return (2 * M_PI * (voltage / 127 * drivetrain.rpm) * drivetrain.wheelDiameter) / 120; }

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
	double radius = (v_right + v_left) / (v_right - v_left) * ((double)drivetrain.trackWidth / 2);
	double theta = ((v_right - v_left) / drivetrain.trackWidth * time) + (startPoint.t * M_PI / 180);

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

std::vector<Coordinate> injectPoint(Coordinate startPoint, Coordinate endPoint, lemlib::AngularDirection behavior, double left, double right, double theta,
									double lookAhead) {
	// Make sure theta is positive
	if(startPoint.t < 0) startPoint.t += 360;

	// Get wheel velocities and proper time
	double v_left = get_velocity(left);
	double v_right = get_velocity(right);
	double v_all = (v_left + v_right) / 2;
	if(v_all == 0) v_all = v_left;

	double time = std::abs(get_time_point(lookAhead, v_all));

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
			while(!(newPoint.t > theta - std::abs((v_right - v_left) / drivetrain.trackWidth * time * 180 / M_PI) &&
					newPoint.t < theta + std::abs((v_right - v_left) / drivetrain.trackWidth * time * 180 / M_PI))) {
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
	
	if(curMatchState != DISABLED) chassis.setPose(x, y, t);
	autonPath.push_back(currentPoint);
}

void wait() {
	if (curMatchState != DISABLED) {
		chassis.waitUntilDone();
	}
}

void wait(double ms) {
	if (curMatchState != DISABLED) {
		pros::delay(ms);
	}
	currentPoint.left = KEY;
	currentPoint.right = ms;
	autonPath.push_back(currentPoint);
}

void wait_until(float dist) {
	if (curMatchState != DISABLED) {
		chassis.waitUntil(dist);
	}
}

void set_raw(int speed) {
	bool forwards = speed >= 0;
	if (curMatchState != DISABLED) {
		left_motor_group.move_voltage(speed * (forwards ? 1 : -1) * 12000);
		right_motor_group.move_voltage(speed * (forwards ? 1 : -1) * 12000);
	}
	currentPoint.left = speed * (forwards ? 1 : -1);
	currentPoint.right = speed * (forwards ? 1 : -1);
	currentPoint.t = currentPoint.t;
	autonPath.push_back(currentPoint);
} 


void set_drive(float distance, float timeout, lemlib::MoveToPointParams params, bool sync) {
	if (curMatchState != DISABLED) {
		chassis.moveToPoint(chassis.getPose().x + distance * cos((-chassis.getPose().theta+90) * M_PI / 180.0),
                              chassis.getPose().y + distance * sin((-chassis.getPose().theta+90) * M_PI / 180.0),
                              timeout, params, !sync);
	}
	currentPoint = get_point(currentPoint, distance);
	currentPoint.left = params.maxSpeed * (params.forwards ? 1 : -1);
	currentPoint.right = params.maxSpeed * (params.forwards	 ? 1 : -1);
	autonPath.push_back(currentPoint);
}

double turn_shortest(double target, double current) {
	double delta = target - current;
	delta = fmod(delta + 540.0, 360.0) - 180.0;
	return delta;
}

void set_turn(float theta, int timeout, lemlib::TurnToHeadingParams params, bool sync) {
	if (curMatchState != DISABLED) {
		chassis.turnToHeading(theta, timeout, params, !sync);
	}


	params.direction = (turn_shortest(theta, currentPoint.t) < currentPoint.t) ? lemlib::AngularDirection::CCW_COUNTERCLOCKWISE : lemlib::AngularDirection::CW_CLOCKWISE;
	if (params.direction == lemlib::AngularDirection::CCW_COUNTERCLOCKWISE) params.maxSpeed *= -1;

	currentPoint.t = theta;
	currentPoint.left = params.maxSpeed;
	currentPoint.right = -params.maxSpeed;
	currentPoint.behavior = params.direction;
	autonPath.push_back(currentPoint);
}

void set_turn(float x, float y, int timeout, lemlib::TurnToPointParams params, bool sync) {
	if (curMatchState != DISABLED) {
		chassis.turnToPoint(x, y, timeout, params, !sync);
	}
	currentPoint.x = x;
	currentPoint.y = y;
	currentPoint.t = atan2(y, x);
	currentPoint.left = params.maxSpeed;
	currentPoint.right = -params.maxSpeed;
	currentPoint.behavior = params.direction;
	autonPath.push_back(currentPoint);
}

void set_point(float x, float y, int timeout, lemlib::MoveToPointParams params, bool sync) {
	if (curMatchState != DISABLED) {
		chassis.moveToPoint(x, y, timeout, params, !sync);
	}
	currentPoint.x = x;
	currentPoint.y = y;
	currentPoint.t = get_theta({currentPoint.x, currentPoint.y}, {x, y}, params.forwards ? DriveDirection::FWD : DriveDirection::REV);
	currentPoint.left = params.maxSpeed * (params.forwards ? 1 : -1);
	currentPoint.right = params.maxSpeed * (params.forwards ? 1 : -1);
	autonPath.push_back(currentPoint);
}

void set_pose(float x, float y, float theta, int timeout, lemlib::MoveToPoseParams params, bool sync) {
	if (curMatchState != DISABLED) {
		chassis.moveToPose(x, y, theta, timeout, params, !sync);
	}
	currentPoint.x = x;
	currentPoint.y = y;
	currentPoint.t = theta;
	currentPoint.left = params.maxSpeed * (params.forwards ? 1 : -1);
	currentPoint.right = params.maxSpeed * (params.forwards ? 1 : -1);
	autonPath.push_back(currentPoint);
}

void set_swing(float theta, DriveSide lockedSide, int timeout, lemlib::SwingToHeadingParams params, bool sync) {
	if (curMatchState != DISABLED) {
		chassis.swingToHeading(theta, lockedSide, timeout, params, !sync);
	}
	params.direction = (turn_shortest(theta, currentPoint.t) < currentPoint.t) ? lemlib::AngularDirection::CCW_COUNTERCLOCKWISE : lemlib::AngularDirection::CW_CLOCKWISE;
	if (params.direction == lemlib::AngularDirection::CCW_COUNTERCLOCKWISE) params.maxSpeed *= -1;

	currentPoint.t = theta;
	currentPoint.left = params.maxSpeed;
	currentPoint.right = -params.maxSpeed;
	autonPath.push_back(currentPoint);

	/*
	// Convert main/opposite voltages to left/right voltages
	double right = side == RIGHT_SWING ? main : opp;
	double left = side == LEFT_SWING ? main : opp;
	right = util::clamp(right, 117);
	left = util::clamp(left, 117);

	// Convert voltage to velocity
	double v_left = getVelocity(left);
	double v_right = getVelocity(right);
	double v_all = (v_left + v_right) / 2;

	// Get radius and arc length
	double new_t = theta - currentPoint.t;
	fmod(new_t, 360);
	if(new_t < 0) new_t += 360;
	double radius = (v_right + v_left) / (v_right - v_left) * (ROBOT_WIDTH / 2);
	double arcLength = radius * new_t * M_PI / 180.0;

	currentPoint = getPoint(currentPoint, v_left, v_right, getTimeToPoint(arcLength, v_all));

	currentPoint.left = left;
	currentPoint.right = right;
	currentPoint.behavior = behavior;
	autonPath.push_back(currentPoint);
	*/
}

void set_swing(float x, float y, DriveSide lockedSide, int timeout, lemlib::SwingToPointParams params, bool sync) {
	if (curMatchState != DISABLED) {
		chassis.swingToPoint(x, y, lockedSide, timeout, params, !sync);
	}
	currentPoint.x = x;
	currentPoint.y = y;
	currentPoint.t = atan2(y, x);
	currentPoint.left = params.maxSpeed * (lockedSide == DriveSide::RIGHT ? 1 : -1);
	currentPoint.right = params.maxSpeed * (lockedSide == DriveSide::RIGHT ? -1 : 1);
	autonPath.push_back(currentPoint);
}