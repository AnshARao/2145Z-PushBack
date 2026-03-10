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
		{test1, "Test 1", red},
		{sawp, "Solo Awp"},
		{skills, "Skills", black},
	});

	uiInit();
	auton_sel.selector_callback = sawp;


	pros::Task angleCheck(angleCheckTask);
	pros::Task pathViewer(pathViewerTask);

	intake1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	intake2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	bool drifting = false;
	double initial = chassis.getPose().theta;
	pros::delay(100);
	if(std::abs(chassis.getPose().theta - initial) > 1) drifting = true;

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
		if (controlla.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
			resetLeft();
			resetBack();
		}
		pros::delay(10);
	}
}