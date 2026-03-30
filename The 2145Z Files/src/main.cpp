#include "main.h"
#include "autos.hpp"
#include "pros/misc.h"
#include "screen.hpp"
#include "subsystems/misc.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "subsystems/chassis.hpp"
#include "subsystems/intake.hpp"

void initialize() {
	curMatchState = DISABLED;
	chassis.calibrate();

	auton_sel.selector_populate({
		{doNothing, "2145Z"},
		{sawp, "Solo Awp", green},
		{left7, "Left 7"},
		{left43mid, "Left 4+3 Middle"},
		{left43long, "Left 4+3 Long"},
		{right7, "Right 7"},
		{right43, "Right 4+3"},
		{skills, "Skills", black},
	});

	uiInit();
	auton_sel.selector_callback = left43mid;


	pros::Task angleCheck(angleCheckTask);
	pros::Task pathViewer(pathViewerTask);

	intake1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	intake2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	hoodDown.set_value(true);

	controlla.clear();

	bool drifting = false;
	double initial = chassis.getPose().theta;
	pros::delay(100);
	if(std::abs(chassis.getPose().theta - initial) > 1) drifting = true;

	controlla.rumble(!imu.is_calibrating() && !drifting ? "." : "---");
	controlla.set_text(0, 0, "2145Z");
	if (!imu.is_calibrating() && !drifting) {
		controlla.print(1, 0, "Imu is chilling");
	} else {
		controlla.print(1, 0, "Imu is NOT chilling");
	}
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
		if (controlla.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
			resetFront();
			resetRight();
		}
		if (loaderState == true) {
			setDescore(false);
		}
		pros::delay(10);
	}
}