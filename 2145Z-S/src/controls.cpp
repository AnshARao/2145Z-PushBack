#include "main.h"
#include "pros/misc.h"
#include <stdlib.h>
#include <sys/types.h>

#pragma region Driver Control

// sets drive motors to coast, helps driver control smooothness
void set_drive_coast() {
    motorgroup_L.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    motorgroup_R.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

// sets drive motors to hold, used for auto consistency
void set_drive_hold() {
    motorgroup_L.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    motorgroup_R.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

// scales raw joystick inputs based on curve value
double input_scaling(double input, double curve) {
     if (input != 0) {
    // if (CURVE_TYPE)
    return (powf(2.718, -(curve / 10)) + powf(2.718, (fabs(input) - 127) / 10) * (1 - powf(2.718, -(curve / 10)))) * input;
    // else
    // return powf(2.718, ((abs(x)-127)*RIGHT_CURVE_SCALE)/100)*x;
  }
  return input;
}

// sets the motors to the left and right inputs
void set_tank(int left, int right) {
    motorgroup_L.move_voltage(left * 12000 / 127);
    motorgroup_R.move_voltage(right * 12000 / 127);
}

// gets the joystick inputs, scales them, then sets the motors to adjusted values
void tank_drive(double scale, pros::Controller& controller) {
    double lY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    double rY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    double lX;
    double rX;

    lY > 0 ? lX = abs(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)) : lX = -abs(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
    rY > 0 ? rX = abs(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) : rX = -abs(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

    int left = input_scaling(lY + lX, scale);
    int right = input_scaling(rY + rX, scale);

    set_tank(left, right);
}

#pragma endregion