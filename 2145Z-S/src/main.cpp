#include "main.h"
#include "autons.hpp"
#include "controls.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "robodash/views/console.hpp"
#include "robodash/views/image.hpp"
#include "screen.hpp"
#include "subsystems.hpp"
#include "robodash/api.h"



rd::Selector selector({
    {"right 6 + 3", right63},
    {"left 4+5", left45},
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
    chassis.setPose(-50, -17, 90);
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
      // get left y and right x positions
        // int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        // int leftX = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

        // // move the robot
        // chassis.curvature(leftY, leftX);
      control_loader();
      control_wing();
      control_rollers();
      if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        console1.printf("X: %2f, Y: %2f, T: %2f", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
        console1.println("");
      }

    pros::delay(10);
  }
}
