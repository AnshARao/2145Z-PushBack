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

int vltg_intake = 0;
bool jam_toggle = true;
bool jammed = false;

#pragma region motors

void set_rollers(int vltg1, int vltg2, int vltg3) {
    motor_intake1.move(vltg1);
    motor_intake2.move(vltg2);
    motor_intake3.move(vltg3);
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
            set_rollers(127, 64);
            set_piston(piston_scorer, false);
            set_piston(piston_wing, true);
            break;
        case OUTTAKE:
            set_rollers(-127);
            break;
        case SCORE:
            set_rollers(127);
            set_piston(piston_scorer, true);
            set_piston(piston_wing, false);
            break;
        case SCORE_MID:
            set_rollers(127, -95);
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
    } else if (master.get_digital(BUTTON_SCORE_MID)) {
        set_rollers(SCORE_MID);
    } else {
        set_rollers(STOP);
    }
}

#pragma endregion

#pragma region pistons 

void set_piston(ez::Piston& piston, bool state) {
    piston.set(state);
}

void control_piston_toggle(ez::Piston& piston, pros::controller_digital_e_t button) {
    if (controlla.get_digital_new_press(button)) {
        set_piston(piston, !piston.get());
    }
}

void control_piston_hold(ez::Piston& piston, pros::controller_digital_e_t button) {
    if (controlla.get_digital(button)) {
        set_piston(piston, true);
    } else {
        set_piston(piston, false);
    }
}

#pragma endregion