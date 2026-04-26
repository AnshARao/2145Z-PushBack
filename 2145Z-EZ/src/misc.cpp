#include "pros/misc.h"
#include "main.h"
#include "pros/adi.hpp"

pros::adi::DigitalOut loader('F', false);
pros::adi::DigitalOut wing('H', false);
pros::adi::DigitalOut descore('A', false);

bool loaderState = false;
bool descoreState = false;

void setLoader(bool state) {
    if (curMatchState != DISABLED) {
        loader.set_value(state);
    }
}

void setWing(bool state) {
    if (curMatchState != DISABLED) {
        wing.set_value(state);
    }
}

void setDescore(bool state) {
    if (curMatchState != DISABLED) {
        descore.set_value(state);
    }
}

void miscControl() {
    if (controlla.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        loaderState = !loaderState;
        setLoader(loaderState);
    }
    if (controlla.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        setWing(true);
    } else {
        setWing(false);
    }
    if (controlla.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
        descoreState = !descoreState;
        setDescore(descoreState);
    }
}

