#include "chassis.hpp"
#include "main.h"
#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include "screen.hpp"

pros::Motor intake1(2, pros::v5::MotorGears::blue);

pros::Motor intake2(1, pros::v5::MotorGears::blue);
//pros::Motor intake2(1, pros::v5::MotorGears::green);
//pros::Motor intake3(3, pros::v5::MotorGears::green);

pros::adi::DigitalOut hood('A', false);
pros::adi::DigitalOut tunnel('E', false);
pros::adi::DigitalOut intakeLift('C', false);

bool midSlow = false;

void intake() {
    if (curMatchState != DISABLED) {
        intake1.move(127);
        intake2.move(127);
        //intake3.move(0);
        hood.set_value(false);
        tunnel.set_value(false);
        intakeLift.set_value(false);
    }
}

void outtake() {
    if (curMatchState != DISABLED) {
        if (midSlow) {
            intakeLift.set_value(true);
            intake1.move_velocity(-51);
            intake2.move(-127);
            //intake3.move(-127);
        } else {
            intakeLift.set_value(false);
            intake1.move(-127);
            intake2.move(-127);
            //intake3.move(-127);
        }
    }
}

void score() {
    if (curMatchState != DISABLED) {
        intakeLift.set_value(true);
        intake1.move(127);
        intake2.move(127);
        //intake3.move(127);
        hood.set_value(true);
        tunnel.set_value(false);
        intakeLift.set_value(false);
    }
}

void scoreMiddle() {
    if (curMatchState != DISABLED) {
        if (midSlow) {
            intake1.move(127);
            intake2.move_velocity(-100);
            //intake2.move(127);
            //intake3.move_velocity(-100);
        } else {
            intake1.move(127);
            intake2.move(-127);
            //intake2.move(127);
            //intake3.move(-127);
        }
        hood.set_value(false);
        tunnel.set_value(true);
        intakeLift.set_value(false);
    }
}

void stop() {
    if (curMatchState != DISABLED) {
        intake1.move(0);
        intake2.move(0);
        //intake3.move(0);
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