#include "lemlib/chassis/chassis.hpp"
#include "fmt/format.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/pid.hpp"
#include "main.h"
#include "pros/distance.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

pros::Controller controlla(pros::E_CONTROLLER_MASTER);
// imu
pros::Imu imu(9);

// left motor group
pros::MotorGroup left_motor_group({-18, 19, -20}, pros::MotorGears::blue);
// right motor group
pros::MotorGroup right_motor_group({15, -16, 17}, pros::MotorGears::blue);

pros::Motor left1(-18, pros::MotorGears::blue);
pros::Motor left2(19, pros::MotorGears::blue);
pros::Motor left3(-20, pros::MotorGears::blue);
pros::Motor right1(15, pros::MotorGears::blue);
pros::Motor right2(-16, pros::MotorGears::blue);
pros::Motor right3(17, pros::MotorGears::blue);

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

	if (curMatchState == DISABLED) return; // does not play on brain screen
	// get distance and convert mm to inches
	float dist = sensor.get_distance() / 25.4f; 

	if (dist < 0 || dist > 200) return; // check if the distance is valid

	// Fold robot θ to [0°, 360°) before rad math (same as sensorHeading; avoids negative rad heading).
	float thetaDegForCos = static_cast<float>(chassis.getPose().theta);
	while (thetaDegForCos < 0.0f) thetaDegForCos += 360.0f;
	while (thetaDegForCos >= 360.0f) thetaDegForCos -= 360.0f;

	float heading = thetaDegForCos * 0.0174533f;
	while (heading > (1.57079632679f / 2.0f)) heading -= 1.57079632679f;

	// calculate the distance to reset
	float resetDist = (dist + sensorOffset) * cosf(heading);

	// Direction the sensor is pointing in world frame (0=+Y/top, 90=+X/right, 180=-Y/bottom, 270=-X/left).
	// Use signed angle so quadrant is correct (no fabs).
	float sh = chassis.getPose().theta + headingOffset;
	float sensorHeading = sh;
	// Fold into [0, 360). Do not use fmod here: fmod copies the sign of the dividend, so negative
	// angles stay negative; -0.0f also skips a plain "while (x < 0)" loop. While-loops are exact.
	while (sensorHeading < 0.0f) sensorHeading += 360.0f;
	while (sensorHeading >= 360.0f) sensorHeading -= 360.0f;

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

	double theta = chassis.getPose().theta;
	if (resettingX) {
		chassis.setPose(newPos, chassis.getPose().y, theta);
		currentPoint.x = newPos;
	} else {
		chassis.setPose(chassis.getPose().x, newPos, theta);
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

// function to set the raw speed
void set_raw(int speed) {
	bool forwards = speed >= 0;
	if (curMatchState != DISABLED) {
		left_motor_group.move_voltage(speed * 12000); // set the left speed
		right_motor_group.move_voltage(speed  * 12000); // set the right speed
	}
	currentPoint.left = speed * (forwards ? 1 : -1); // set the left speed
	currentPoint.right = speed * (forwards ? 1 : -1); // set the right speed
	currentPoint.t = currentPoint.t; // set the theta
	autonPath.push_back(currentPoint); // add the current point to the path
} 

void set_raw(int speed1, int speed2) {
	bool forwards1 = speed1 >= 0;
	bool forwards2 = speed2 >= 0;
	if (curMatchState != DISABLED) {
		left_motor_group.move_voltage(speed1 * 12000); // set the left speed
		right_motor_group.move_voltage(speed2  * 12000); // set the right speed
	}
	currentPoint.left = speed1 * (forwards1 ? 1 : -1); // set the left speed
	currentPoint.right = speed2 * (forwards2 ? 1 : -1); // set the right speed
	currentPoint.t = currentPoint.t; // set the theta
	autonPath.push_back(currentPoint); // add the current point to the path
} 

namespace {

double wheel_inches_per_motor_degree() {
	return M_PI * static_cast<double>(drivetrain.wheelDiameter) / 360.0;
}

double angle_wrap_180(double deg) {
	double x = std::fmod(deg + 180.0, 360.0);
	if (x < 0.0) x += 360.0;
	return x - 180.0;
}

double avg_traveled_inches(double left_start_deg, double right_start_deg) {
	const double dl = left_motor_group.get_position() - left_start_deg;
	const double dr = right_motor_group.get_position() - right_start_deg;
	const double k = wheel_inches_per_motor_degree();
	return 0.5 * (dl + dr) * k;
}

void scale_to_max(float& l, float& r, float lim) {
	const float m = std::max(std::fabs(l), std::fabs(r));
	if (m > lim && m > 1e-6f) {
		const float s = lim / m;
		l *= s;
		r *= s;
	}
}

} // namespace

// Encoder-average distance PID + IMU heading hold (EZ/RW style). Blocking.
void set_drive_pid(float distance_inches, int timeout_ms, lemlib::MoveToPointParams params) {
	if (curMatchState == DISABLED) return;

	lemlib::PID distPid(lateral_controller.kP, lateral_controller.kI, lateral_controller.kD, lateral_controller.windupRange);
	lemlib::PID headPid(angular_controller.kP, angular_controller.kI, angular_controller.kD, angular_controller.windupRange);

	const double l0 = left_motor_group.get_position();
	const double r0 = right_motor_group.get_position();
	const double start_heading = imu.get_rotation();

	const uint32_t t0 = pros::millis();
	float prev_l = 0;
	float prev_r = 0;
	const float slew_step = lateral_controller.slew;
	int settle_count = 0;

	while (true) {
		const uint32_t now = pros::millis();
		if (now - t0 >= static_cast<uint32_t>(timeout_ms)) break;

		const double traveled = avg_traveled_inches(l0, r0);
		const double err = static_cast<double>(distance_inches) - traveled;

		if (params.minSpeed > 0.0f && std::fabs(err) < static_cast<double>(params.earlyExitRange)) break;

		float lin = distPid.update(static_cast<float>(err));
		if (params.minSpeed > 0.0f && std::fabs(lin) < params.minSpeed && std::fabs(err) > 0.05) {
			lin = (err > 0.0 ? 1.0f : -1.0f) * params.minSpeed;
		}

		const float h_err = static_cast<float>(angle_wrap_180(imu.get_rotation() - start_heading));
		const float corr = headPid.update(h_err);

		float l_out = lin + corr;
		float r_out = lin - corr;

		if (slew_step > 0.0f) {
			l_out = std::clamp(l_out, prev_l - slew_step, prev_l + slew_step);
			r_out = std::clamp(r_out, prev_r - slew_step, prev_r + slew_step);
		}

		scale_to_max(l_out, r_out, params.maxSpeed);

		left_motor_group.move_voltage(static_cast<std::int32_t>(l_out * (12000.0 / 127.0)));
		right_motor_group.move_voltage(static_cast<std::int32_t>(r_out * (12000.0 / 127.0)));
		prev_l = l_out;
		prev_r = r_out;

		if (std::fabs(err) < 0.35) {
			settle_count++;
			if (settle_count >= 15) break;
		} else {
			settle_count = 0;
		}

		pros::delay(10);
	}

	left_motor_group.brake();
	right_motor_group.brake();
}

// function to set the drive (LemLib moveToPoint)
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

// function to determine if cw or ccw is the shortest way to turn
double turn_shortest(double target, double current) {
	double delta = target - current; // get the difference between the target and current
	delta = fmod(delta + 540.0, 360.0) - 180.0; // wrap around to 0-360 and subtract 180
	return delta;
}

// function to set the turn to heading
void set_turn(float theta, int timeout, lemlib::TurnToHeadingParams params, bool sync) {
	if (curMatchState != DISABLED) {
		chassis.turnToHeading(theta, timeout, params, !sync);
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
	currentPoint.behavior = params.direction; // set the behavior
	autonPath.push_back(currentPoint); // add the current point to the path
}

// function to set the turn to point
void set_turn(float x, float y, int timeout, lemlib::TurnToPointParams params, bool sync) {
	if (curMatchState != DISABLED) {
		chassis.turnToPoint(x, y, timeout, params, !sync);
	}
	currentPoint.x = x; // set the x
	currentPoint.y = y; // set the y
	currentPoint.t = get_theta({currentPoint.x, currentPoint.y}, {x, y},
		 params.forwards ? DriveDirection::FWD : DriveDirection::REV); // set the theta
	currentPoint.left = params.maxSpeed; // set the left speed
	currentPoint.right = -params.maxSpeed; // set the right speed
	currentPoint.behavior = params.direction; // set the behavior
	autonPath.push_back(currentPoint); // add the current point to the path
}
// function to set the point
void set_point(float x, float y, int timeout, lemlib::MoveToPointParams params, bool sync) {
	if (curMatchState != DISABLED) {
		chassis.moveToPoint(x, y, timeout, params, !sync); // move to the point
	}
	currentPoint.x = x; // set the x
	currentPoint.y = y; // set the y
	currentPoint.t = get_theta({currentPoint.x, currentPoint.y}, {x, y}, 
		params.forwards ? DriveDirection::FWD : DriveDirection::REV); // set the theta
	currentPoint.left = params.maxSpeed * (params.forwards ? 1 : -1); // set the left speed
	currentPoint.right = params.maxSpeed * (params.forwards ? 1 : -1); // set the right speed
	autonPath.push_back(currentPoint); // add the current point to the path
}

// function to set the pose
void set_pose(float x, float y, float theta, int timeout, lemlib::MoveToPoseParams params, bool sync) {
	if (curMatchState != DISABLED) {
		chassis.moveToPose(x, y, theta, timeout, params, !sync); // move to the pose
	}
	currentPoint.x = x; // set the x
	currentPoint.y = y; // set the y
	currentPoint.t = theta; // set the theta
	currentPoint.left = params.maxSpeed * (params.forwards ? 1 : -1); // set the left speed
	currentPoint.right = params.maxSpeed * (params.forwards ? 1 : -1); // set the right speed
	autonPath.push_back(currentPoint); // add the current point to the path
}

// function to set the swing to heading
void set_swing(float theta, DriveSide lockedSide, int timeout, lemlib::SwingToHeadingParams params, bool sync) {
	if (curMatchState != DISABLED) {
		chassis.swingToHeading(theta, lockedSide, timeout, params, !sync); // swing to the heading
	}
	params.direction = (turn_shortest(theta, currentPoint.t) < currentPoint.t) ? 
	lemlib::AngularDirection::CCW_COUNTERCLOCKWISE : lemlib::AngularDirection::CW_CLOCKWISE; // determine the direction to swing
	// if the direction is ccw, multiply the max speed by -1 for the brain screen
	if (params.direction == lemlib::AngularDirection::CCW_COUNTERCLOCKWISE) params.maxSpeed *= -1; 

	currentPoint.t = theta; // set the theta
	currentPoint.left = params.maxSpeed; // set the left speed
	currentPoint.right = -params.maxSpeed; // set the right speed
	autonPath.push_back(currentPoint); // add the current point to the path

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

// function to set the swing to point
void set_swing(float x, float y, DriveSide lockedSide, int timeout, lemlib::SwingToPointParams params, bool sync) {
	if (curMatchState != DISABLED) {
		chassis.swingToPoint(x, y, lockedSide, timeout, params, !sync); // swing to the point
	}
	currentPoint.x = x; // set the x
	currentPoint.y = y; // set the y
	currentPoint.t = atan2(y, x); // set the theta
	currentPoint.left = params.maxSpeed * (lockedSide == DriveSide::RIGHT ? 1 : -1); // set the left speed
	currentPoint.right = params.maxSpeed * (lockedSide == DriveSide::RIGHT ? -1 : 1); // set the right speed
	autonPath.push_back(currentPoint); // add the current point to the path
}