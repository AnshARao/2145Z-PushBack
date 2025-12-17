#include "controls.hpp"
#include "main.h"
#include "pros/misc.h"
#include "subsystems.hpp"
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

void set_wing(bool state) {
    piston_wing.set_value(state);
}

void control_wing() {
    if (master.get_digital(BUTTON_WING)) {
        set_wing(true);
    } else {
        set_wing(false);
    }
}

void set_loader(bool state) {
    state_loader = state;
    piston_loader.set_value(state_loader);
}

void control_loader() {
    if (master.get_digital_new_press(BUTTON_LOADER)) {
        set_loader(!state_loader);
    }
}

void set_hood(bool state) {
    piston_hood.set_value(state);
}

void set_rollers(int vltg1, int vltg2) {
    motor_intake1.move(vltg1);
    motor_intake2.move(vltg1);
    motor_intake3.move(vltg2);
} 

void set_rollers(int vltg) {
    motor_intake1.move(vltg);
    motor_intake2.move(vltg);
    motor_intake3.move(vltg);
}

void set_rollers(RollerStates state) {
    switch (state) {
        case INTAKE:
            set_rollers(127);
            set_hood(false);
            break;
        case OUTTAKE:
            set_rollers(-127);
            break;
        case SCORE:
            set_rollers(127);
            set_hood(true);
            break;
        case SCORE_MID:
            set_rollers(127, -127);
            break;
        case STOP:
            set_rollers(0);
            break;
    }
}

void control_rollers() {
    if (master.get_digital(BUTTON_INTAKE)) {
        set_rollers(INTAKE);
    } else if (master.get_digital(BUTTON_OUTTAKE)) {
        set_rollers(OUTTAKE);
    } else if (master.get_digital(BUTTON_SCORE)) {
        set_rollers(SCORE);
    } else if (master.get_digital(BUTTON_SCORE_MIDDLE)) {
        set_rollers(SCORE_MID);
    } else {
        set_rollers(STOP);
    }
}