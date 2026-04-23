#include "pros/misc.h"
#include "main.h"
#include "pros/adi.hpp"

pros::adi::DigitalOut loader('G', false);
pros::adi::DigitalOut wingBack('E', false);
pros::adi::DigitalOut wingFront('A', false);
pros::adi::DigitalOut descore('H', false);
pros::adi::DigitalOut tech('F', false);

bool techState = false;
bool loaderState = false;
bool descoreState = false;
bool wingFrontState = true;
bool wingBackState = false;

void setLoader(bool state) {
    if (curMatchState != DISABLED) {
        loader.set_value(state);
        loaderState = state;
    }
}

void setWingBack(bool state) {
    if (curMatchState != DISABLED) {
        wingBack.set_value(state);
        wingBackState = state;
    }
}

void setWingFront(bool state) {
    if (curMatchState != DISABLED) {
        wingFront.set_value(state);
        wingFrontState = state;
    }
}

void setDescore(bool state) {
    if (curMatchState != DISABLED) {
        descore.set_value(state);
        descoreState = state;
    }
}

void setTech(bool state) {
    if (curMatchState != DISABLED) {
        tech.set_value(state);
        techState = state;
    }
}

void miscControl() {
    if (controlla.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        loaderState = !loaderState;
        setLoader(loaderState);
    }
    if (controlla.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        wingFrontState = !wingFrontState;
        setWingFront(wingFrontState);
    }
    if (controlla.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        setWingBack(true);
    } else {
        setWingBack(false);
    }
    if (controlla.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        descoreState = !descoreState;
        setDescore(descoreState);
    }
    if (controlla.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        techState = !techState;
        setTech(techState);
    }
}