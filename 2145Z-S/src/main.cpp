#include "main.h"
#include "autons.hpp"
#include "pros/motors.h"
#include "robotconfig.h"
#include "screen.hpp"
#include "subsystems.hpp"

void initialize() {

  pros::delay(500);
  chassis.calibrate();

  auton_sel.selector_populate({
      {doNothing, "2145Z", pink},
      {measure_offsets, "Measure Offsets", blue},
      {tune, "Tune", blue}
    });

    uiInit();
    auton_sel.selector_callback = doNothing;
    pros::Task pathViewer(pathViewerTask);
    pros::Task angleChecker(angleCheckTask);
    motor_intake1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    motor_intake2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

}

void disabled() {
}


void autonomous() {
  chassis.setPose(0,0,0);
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  auton_sel.selector_callback();
}

void opcontrol() {
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
  while (true) {
      tank_drive(DRIVE_CURVE, master);

      if (master.get_digital(BUTTON_INTAKE)) {
        motor_intake1.move(127);
        motor_intake2.move(64);
        motor_intake3.move(64);
      } else if (master.get_digital(BUTTON_OUTTAKE)) {
        motor_intake1.move(-127);
        motor_intake2.move(-127);
        motor_intake3.move(-127);
      } else if (master.get_digital(BUTTON_SCORE)) {
        motor_intake1.move(127);
        motor_intake2.move(127);
        motor_intake3.move(127);
      } else if (master.get_digital(BUTTON_SCORE_MIDDLE)) {
        motor_intake1.move(127);
        motor_intake2.move(127);
        motor_intake3.move(-127);
      } else {
        motor_intake1.move(0);
        motor_intake2.move(0);
        motor_intake3.move(0);
      }
    pros::delay(10);
  }
}
