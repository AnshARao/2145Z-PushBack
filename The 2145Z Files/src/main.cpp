#include "main.h"
#include "autons.hpp"
#include "chassis.hpp"
#include "lemlib/api.hpp"
#include "screen.hpp"
#include "chassis.hpp"

void initialize() {
  matchState = DISABLED;
  pros::delay(500);
  EzChassis.initialize();
  LLChassis.calibrate(false);


  // Set the drive to your own constants from autons.cpp!
  default_constants();
  auton_sel.selector_populate({
		{doNothing, "2145Z"},
		{skills, "Skills", black},
	});

  uiInit();
  auton_sel.selector_callback = doNothing;
  pathViewerTask();
  angleCheckTask();

  intake1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	intake2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	hoodDown.set_value(true);
	setWingFront(false);
	setWingBack(false);

  // Initialize chassis and auton selector
  master.rumble(EzChassis.drive_imu_calibrated() ? "." : "---");
}


void disabled() {
  matchState = DISABLED;
}

void competition_initialize() {
  // . . .
}

void autonomous() {
  matchState = AUTO;
  EzChassis.pid_targets_reset();                // Resets PID targets to 0
  EzChassis.drive_imu_reset();                  // Reset gyro position to 0
  EzChassis.drive_sensor_reset();               // Reset drive sensors to 0
  EzChassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  EzChassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  auton_sel.selector_callback();
}

void opcontrol() {
  matchState = DRIVER;
  EzChassis.drive_brake_set(MOTOR_BRAKE_COAST);

  while (true) {
    tank_drive(5);
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

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
