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

void intake_t() {
    
    int jam_timer = 0;
    const int jam_timer_threshold = 200;
    const int min_velo_threshold = 5;

    while(true) {
        motor_intake.move_voltage(vltg_intake);
        pros::delay(10);
        

        
        if((abs)(motor_intake.get_actual_velocity()) < min_velo_threshold && !(vltg_intake <= 0) && jam_toggle/* && matchState == AUTO*/) {
            jam_timer += 10;
            if (jam_timer > jam_timer_threshold) {
                jammed = true;
                set_motor(motor_intake, -12000);
                pros::delay(100);
                set_motor(motor_intake, vltg_intake);
                jam_timer = 0;
                jammed = false;
            } else {
                jam_timer = 0;
            }
        } 
    }
}

pros::Task intakeTask(intake_t);

#pragma region motors

void set_motor(pros::Motor& motor, int vltg) {
    motor.move_voltage(vltg);
}

void set_rollers(int vltg1, int vltg2) {
    vltg_intake = vltg1;
    set_motor(motor_scorer, vltg2);
}

void set_rollers(int vltg) {
    vltg_intake = vltg;
    set_motor(motor_scorer, vltg);
}

void set_rollers(RollerStates state) {
    switch (state) {
        case INTAKE:
            set_rollers(12000, -6000);
            set_piston(piston_scorer, false);
            break;
        case OUTTAKE:
            set_rollers(-12000);
            break;
        case SCORE:
            set_rollers(12000);
            break;
        case STOP:
            set_rollers(0);
            break;
    }
}

void control_rollers() {
    if (jammed) return;


    // Default manual control when not auto-outtaking
    if (controlla.get_digital(BUTTON_INTAKE)) {
        set_rollers(INTAKE);
    } else if (controlla.get_digital(BUTTON_OUTTAKE)) {
        set_rollers(OUTTAKE);
    } else if (controlla.get_digital(BUTTON_SCORE)) {
        set_rollers(SCORE);
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