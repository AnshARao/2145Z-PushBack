#include "main.h"
#include "autos.hpp"
#include "pros/device.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "screen.hpp"
#include "subsystems/misc.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "subsystems/chassis.hpp"
#include "subsystems/intake.hpp"

void checkSensor(pros::Distance d) {
	float dist = d.get_distance() / 25.4f; 

	if (dist < 0 || dist > 200)	{
		errormsg = "Sensor NOT Working";
	}
}

void checkAllSensors() {
	checkSensor(distanceFront);
	checkSensor(distanceLeft);
	checkSensor(distanceBack);
	checkSensor(distanceBack);
}

void initialize() {
	curMatchState = DISABLED;
	chassis.calibrate();

	auton_sel.selector_populate({
		{doNothing, "2145Z"},
		{left43mid, "L43M"},
		{left43long, "L43L"},
		{right43, "R4+3"},
		{left7, "L7"},
		{right7, "R7"},
		{left4, "L4"},
		{right4, "R4"},
		{techRush, "Tch Rsh"},
		{sawp, "Sawp", green},
		{doubleMid, "Dbl Mid"},
		{skills, "Skills", black},
	});

	uiInit();
	auton_sel.selector_callback =left43long;
	auton_sel.selector_name = "L43L";


	pros::Task angleCheck(angleCheckTask);
	pros::Task pathViewer(pathViewerTask);
	pros::Task contrSel(controllerTask);

	intake1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	intake2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	hoodDown.set_value(true);
	setWingFront(false);
	setWingBack(false);

	bool drifting = false;
	double initial = chassis.getPose().theta;
	wait(500);
	if(std::abs(chassis.getPose().theta - initial) > 1) drifting = true;

	controlla.rumble(!imu.is_calibrating() && !drifting ? "." : "---");
	if (!imu.is_calibrating() && !drifting) {
		errormsg = "IMU good";
	} else {
		errormsg = "IMU BAD";
	}
	//checkAllSensors();
}

void disabled() {
	curMatchState = DISABLED;
}

void competition_initialize() {
	curMatchState = DISABLED;
}

void autonomous() {
	curMatchState = AUTO;
	chassis.setBrakeMode(MOTOR_BRAKE_HOLD);
	auton_sel.selector_callback();

	// // set position to x:0, y:0, heading:0
    // chassis.setPose(0, 0, 0);
    // // turn to face heading 90 with a very long timeout
    // chassis.moveToPose(0, 24, 0, 10000);
}

void opcontrol() {
	//autonomous();
	curMatchState = DRIVER;
	chassis.setBrakeMode(MOTOR_BRAKE_COAST);
	while (true) {
		tank_drive(7);
		intakeControl();
		miscControl();
		if (controlla.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
			resetLeft();
			resetBack();
		}
		if (controlla.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
			resetFront();
			resetRight();
		}
		if (descoreState == true) {
			setLoader(false);
			setTech(false);
		}
		pros::delay(10);
	}
}