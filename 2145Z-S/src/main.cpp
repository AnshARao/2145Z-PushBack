#include "main.h"
#include "autons.hpp"
#include "controls.hpp"
#include "pros/motors.h"
#include "robotconfig.h"
#include "screen.hpp"
#include "subsystems.hpp"
#include "robodash/api.h"

rd::Selector selector({
    {"Right 9", right9},
    {"Right 4" , right4},
    {"SAWP", SAWP},
    {"Skills", skills},
    });

void initialize() {

  pros::delay(500);
  chassis.calibrate();

  // auton_sel.selector_populate({
  //     {doNothing, "2145Z", pink},
  //     {SAWP, "SAWP", blue},
  //     {measure_offsets, "Measure Offsets", blue},
  //     {tune, "Tune", blue}
  //   });

    //uiInit();
    //auton_sel.selector_callback = SAWP;
    //pros::Task pathViewer(pathViewerTask);
    //pros::Task angleChecker(angleCheckTask);
    motor_intake1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    motor_intake2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    selector.focus();
}

void disabled() {
}


void autonomous() {
  chassis.setPose(0,0,0);
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  //auton_sel.selector_callback();
  selector.run_auton();
}

void opcontrol() {
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
  while (true) {
      tank_drive(DRIVE_CURVE, master);

      control_loader();
      control_wing();
      control_rollers();

    pros::delay(10);
  }
}
