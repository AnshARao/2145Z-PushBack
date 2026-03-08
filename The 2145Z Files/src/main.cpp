#include "main.h"
#include "autos.hpp"
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
		{right7, "Right 7", green},
		{skills, "Skills", black},
	});

	chassis.calibrate();
	uiInit();

	pros::Task angleCheck(angleCheckTask);
	pros::Task pathViewer(pathViewerTask);

	intake1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	intake2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	bool drifting = false;
	double initial = chassis.getPose().theta;
	pros::delay(100);
	if(std::abs(chassis.getPose().theta - initial) > 1) drifting = true;

	// Initialize auton selector, and tasks
	uiInit();

	controlla.rumble(!imu.is_calibrating() && !drifting ? "." : "---");
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
}

void opcontrol() {
	curMatchState = DRIVER;
	chassis.setBrakeMode(MOTOR_BRAKE_COAST);
	while (true) {
		tank_drive(7);
		intakeControl();
		miscControl();
		pros::delay(10);
	}
}