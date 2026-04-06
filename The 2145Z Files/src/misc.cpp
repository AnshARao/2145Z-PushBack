#include "pros/misc.h"
#include "main.h"
#include "pros/adi.hpp"

pros::adi::DigitalOut loader('G', false);
pros::adi::DigitalOut wingBack('E', false);
pros::adi::DigitalOut wingFront('A', false);
pros::adi::DigitalOut descore('H', false);

bool loaderState = false;
bool descoreState = false;

void setLoader(bool state) {
    if (matchState != DISABLED) {
        loader.set_value(state);
        loaderState = state;
    }
}

void setWingBack(bool state) {
    if (matchState != DISABLED) {
        wingBack.set_value(state);
    }
}

void setWingFront(bool state) {
    if (matchState != DISABLED) {
        wingFront.set_value(state);
    }
}

void setDescore(bool state) {
    if (matchState != DISABLED) {
        descore.set_value(state);
        descoreState = state;
    }
}

void miscControl() {
    if (controlla.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        loaderState = !loaderState;
        setLoader(loaderState);
    }
    if (controlla.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        setWingFront(false);
        setWingBack(true);
    } else {
        setWingFront(true);
        setWingBack(false);
    }

    if (controlla.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
        descoreState = !descoreState;
        setDescore(descoreState);
    }
}
