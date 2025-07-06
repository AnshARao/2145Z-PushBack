#include "intake.hpp"
#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/rtos.hpp"

bool isRed = true;

#define INTAKE_PORT 1
#define OPTICAL_PORT 15

pros::Motor intake(INTAKE_PORT);
pros::Optical optical(OPTICAL_PORT);

void intakeSpin(int speed) {
    intake.move(speed); //moves the intake at the speed from the variable
}

void intakeControl() {
    if (wrongRing == true) {
        return;
    }
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { // if forward button
        intakeSpin(127);    //spin forward
    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { // if reverse btn
        intakeSpin(-127);   //spin reverse
    }
    else {  // if no intake buttons are being pressed
        intakeSpin(0);  // do not spin
    }
}

void colorSortLoop() {
    const int PROX_THRESHOLD = 100; // Min prox for the color sort to start
    int currentProx = optical.get_proximity();  // Current proximity values
    int currentHue = optical.get_hue(); // current color values

    if (currentProx > PROX_THRESHOLD) { // if the currentProx is above the threshold
        if (isRed == true && currentHue >= 210 && currentHue <= 240) {
            sortOutRing();
        }
        else if (isRed == false &&  (currentHue >= 340 || currentHue <= 20)) {
            sortOutRing();
        }
    }
}

bool wrongRing = false; // declare wrongRight boolean

void sortOutRing() {
    wrongRing = true;       //wrong ring is there, locks driver ctrl
    const int WAIT_TIME = 250;  // time for ring to reach the top
    int originalSpeed = intake.get_target_velocity(); //orignal speed to set back to

    intakeSpin(127);    //set intake to full speed for consistency
    pros::delay(WAIT_TIME); //wait for ring to be at the top
    intakeSpin(-127);   // reverse intake
    pros::delay(200);   //only for 0.2 seconds
    intakeSpin(originalSpeed);  // set intake to original speed
    wrongRing = false;  // wrong ring is no longer there, unlocks driver ctrl
}

void colorSort_t() {    // color sort task
    while (true) {      // loop is always running
        colorSortLoop();    //run the color sort loop code
    }
}