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
    if (curMatchState == DISABLED) return;

    intake1.move(127);
    hoodDown.set_value(true);
    stopper.set_value(true);

    static int stallTicks = 0;

    bool highCurrent = intake2.get_current_draw() > 9000;
    bool lowSpeed = std::abs(intake2.get_actual_velocity()) < 20;
    bool stalled = highCurrent && lowSpeed;

    if (stalled) stallTicks++;
    else stallTicks = 0;

    if (stallTicks >= 6) intake2.move(15);   // or 0
    else intake2.move(127);
}

void outtake() {
    if (curMatchState != DISABLED) {
        if (midSlow) {
            intakeLift.set_value(true);
            intake1.move_velocity(-300);
            intake2.move(-127);
        } else {
            intakeLift.set_value(false);
            intake1.move(-127);
            intake2.move(-127);
        }
    }
}

void score() {
    if (curMatchState != DISABLED) {
        intakeLift.set_value(true);
        intake1.move(127);
        intake2.move(127);
        hoodDown.set_value(false);
        stopper.set_value(false);
    }
}

void scoreMiddle() {
    if (curMatchState != DISABLED) {
        if (midSlow) {
            intake1.move(127);
            intake2.move_velocity(200);
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
    if (curMatchState != DISABLED) {
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
    } else if (controlla.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
        midSlow = !midSlow;
    } else {
        stop();
    }
    if (midSlow) {
        controlla.rumble(".");
    }
}