#include "controls.hpp"  // IWYU pragma: keep
#include <functional>  // IWYU pragma: keep
#include <string>  // IWYU pragma: keep
#include <vector>  // IWYU pragma: keep
#include "EZ-Template/piston.hpp"
#include "EZ-Template/util.hpp"  // IWYU pragma: keep
#include "drive.hpp"  // IWYU pragma: keep
#include "main.h"   // IWYU pragma: keep
#include "pros/abstract_motor.hpp"  // IWYU pragma: keep
#include "pros/adi.hpp"  // IWYU pragma: keep
#include "pros/device.hpp"  // IWYU pragma: keep
#include "pros/misc.h"  // IWYU pragma: keep
#include "pros/motors.h"  // IWYU pragma: keep
#include "pros/motors.hpp"  // IWYU pragma: keep
#include "pros/optical.hpp"  // IWYU pragma: keep
#include "pros/rtos.hpp"  // IWYU pragma: keep
#include "screen.hpp"  // IWYU pragma: keep
#include "subsystems.hpp"  // IWYU pragma: keep

#pragma region motors

void set_motor(pros::Motor& motor, int vltg) {
    motor.move_voltage(vltg);
}

void set_rollers(int vltg1, int vltg2) {
    set_motor(motor_intake, vltg1);
    set_motor(motor_scorer, vltg2);
}

void set_rollers(int vltg) {
    set_motor(motor_intake, vltg);
    set_motor(motor_scorer, vltg);
}

void set_rollers(RollerStates state) {
    switch (state) {
        case INTAKE:
            set_rollers(12000, -12000);
            set_piston(piston_scorer, true);
            break;
        case OUTTAKE:
            set_rollers(-12000);
            break;
        case SCORE_TOP:
            set_rollers(12000);
            set_piston(piston_scorer, true);
            break;
        case SCORE_MID:
            set_rollers(12000);
            set_piston(piston_scorer, false);
            break;
        case STOP:
            set_rollers(0);
            break;
    }
}

void control_rollers() {
    if (controlla.get_digital(BUTTON_INTAKE)) {
        set_rollers(INTAKE);
    }   else if (controlla.get_digital(BUTTON_OUTTAKE)) {
        set_rollers(OUTTAKE);
    }   else if (controlla.get_digital(BUTTON_SCORE_TOP)) {
        set_rollers(SCORE_TOP);
    }   else if (controlla.get_digital(BUTTON_SCORE_MID)) {
        set_rollers(SCORE_MID);
    }   else {
        set_rollers(STOP);
    }
}

#pragma endregion

#pragma region pistons 

void set_piston(ez::Piston& piston, bool state) {
    piston.set(state);
}

void control_piston(ez::Piston& piston, pros::controller_digital_e_t button) {
    if (controlla.get_digital_new_press(button)) {
        set_piston(piston, !piston.get());
    }
}

#pragma endregion