#include "chassis.hpp"
#include "main.h"
#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include "screen.hpp"

pros::Motor intake1(4, pros::v5::MotorGears::blue);
pros::Motor intake2(2, pros::v5::MotorGears::blue);

pros::adi::DigitalOut hoodDown('C', false);
pros::adi::DigitalOut stopper('D', false);
pros::adi::DigitalOut intakeLift('B', false);

bool midSlow = false;

void intake() {
    if (matchState != DISABLED) {
        intake1.move(127);
        intake2.move(127);
        hoodDown.set_value(true);
        stopper.set_value(true);
    }
}

void outtake() {
    if (matchState != DISABLED) {
        if (midSlow) {
            intakeLift.set_value(true);
            intake1.move_velocity(-51);
            intake2.move(-127);
        } else {
            intakeLift.set_value(false);
            intake1.move(-127);
            intake2.move(-127);
        }
    }
}

void score() {
    if (matchState != DISABLED) {
        intakeLift.set_value(true);
        intake1.move(127);
        intake2.move(127);
        hoodDown.set_value(false);
        stopper.set_value(false);
    }
}

void scoreMiddle() {
    if (matchState != DISABLED) {
        if (midSlow) {
            intake1.move(127);
            intake2.move_velocity(100);
        } else {
            intake1.move(127);
            intake2.move(127);
        }
        hoodDown.set_value(true);
        stopper.set_value(false);
        //intakeLift.set_value(false);
    }
}

void stop() {
    if (matchState != DISABLED) {
        intake1.move(0);
        intake2.move(0);
    }
}

void intakeControl() {
    if (controlla.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        intake();
    } else if (controlla.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        outtake();
    } else if (controlla.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        score();
    } else if (controlla.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        scoreMiddle();
    } else if (controlla.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        midSlow = !midSlow;
    } else {
        stop();
    }
    if (midSlow) {
        controlla.rumble(".");
    }
}