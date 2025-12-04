#include "pros/distance.hpp"
#include "pros/optical.hpp"
#pragma once

namespace lib {



class Color {

private:
    pros::Optical optic = pros::Optical(4);
    pros::Distance dist = pros::Distance(8);

public:
    Color() {
        optic.set_integration_time(5);
        optic.set_led_pwm(100);
    }
    bool seesRed() {
        return (optic.get_hue() < 30 || optic.get_hue() > 300) && dist.get_distance() < 25;
    }
    bool seesBlue() {
        return (optic.get_hue() > 180 && optic.get_hue() < 330) && dist.get_distance() < 25;
    }
};
};


