#include "chassis.hpp"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"

// LED VARIABLES
constexpr int LED_COUNT = 42;
constexpr int HALF = LED_COUNT / 2; // 21
pros::adi::LED leds('B', LED_COUNT); // Change 42 if your strip has a different LED count
// RED LED COLORS 
const uint32_t BACKGROUND = 0x200000;
const uint32_t FAINT_RED   = 0x200000;
const uint32_t MID_RED     = 0x500000;
const uint32_t BRIGHT_RED  = 0x900000;
const uint32_t BRIGHTER_RED = 0xD00000;
const uint32_t MAX_RED     = 0xFF0000; 
// BLUE LED COLORS 
const uint32_t BACKGROUNDB = 0x000020;
const uint32_t FAINT_BLUE    = 0x000020;
const uint32_t MID_BLUE      = 0x000050;
const uint32_t BRIGHT_BLUE   = 0x000090;
const uint32_t BRIGHTER_BLUE = 0x0000D0;
const uint32_t MAX_BLUE      = 0x0000FF; 
// GREEN LED COLORS
const uint32_t FAINT_GREEN   = 0x002000;
const uint32_t MID_GREEN     = 0x005000;
const uint32_t BRIGHT_GREEN  = 0x009000;
const uint32_t BRIGHTER_GREEN= 0x00E000;
//YELLOW LED COLORS 
const uint32_t FAINT_YELLOW   = 0x202000;
const uint32_t MID_YELLOW     = 0x505000;
const uint32_t BRIGHT_YELLOW  = 0x909000;
const uint32_t BRIGHTER_YELLOW= 0xE0E000;
//ORANGE LED COLORS
const uint32_t FAINT_ORANGE   = 0x502000;
const uint32_t MID_ORANGE     = 0x905000;
const uint32_t BRIGHT_ORANGE  = 0xC09000;
const uint32_t BRIGHTER_ORANGE= 0xE0C000;
//RED FOR TEMP COLORS 
const uint32_t FAINT_RED_TEMP   = 0x400000;
const uint32_t MID_RED_TEMP     = 0x800000;
const uint32_t BRIGHT_RED_TEMP  = 0xC00000;
const uint32_t BRIGHTER_RED_TEMP= 0xFF0000;

double avgTemp = (left_motor_group.get_temperature() + right_motor_group.get_temperature()) / 2.0;

void blinkColor(uint32_t faint, uint32_t mid, uint32_t bright, uint32_t brighter, int delayMs = 50) {
    leds.set_all(faint); leds.update(); pros::delay(delayMs);
    leds.set_all(mid); leds.update(); pros::delay(delayMs);
    leds.set_all(bright); leds.update(); pros::delay(delayMs);
    leds.set_all(brighter); leds.update(); pros::delay(delayMs);
    leds.set_all(bright); leds.update(); pros::delay(delayMs);
    leds.set_all(mid); leds.update(); pros::delay(delayMs);
}
// Main temperature LED blink function
void tempMonitorBlink() {
    double avgTemp = (left_motor_group.get_temperature() + right_motor_group.get_temperature()) / 2.0;

    if (avgTemp < 55) {
        blinkColor(FAINT_GREEN, MID_GREEN, BRIGHT_GREEN, BRIGHTER_GREEN);
    } else if (avgTemp < 60) {
        blinkColor(FAINT_YELLOW, MID_YELLOW, BRIGHT_YELLOW, BRIGHTER_YELLOW);
    } else if (avgTemp < 65) {
        blinkColor(FAINT_ORANGE, MID_ORANGE, BRIGHT_ORANGE, BRIGHTER_ORANGE);
    } else {
        blinkColor(FAINT_RED_TEMP, MID_RED_TEMP, BRIGHT_RED_TEMP, BRIGHTER_RED_TEMP);
    }
}
void red_wave() {

    static int pos = 0;

    auto setBoth = [&](uint32_t color, int index) {

        int i1 = (index + LED_COUNT) % LED_COUNT;
        int i2 = (i1 + HALF) % LED_COUNT;

        leds.set_pixel(color, i1);
        leds.set_pixel(color, i2);
    };

    setBoth(MAX_RED, pos);
    setBoth(BRIGHTER_RED, pos - 1);
    setBoth(BRIGHT_RED, pos - 2);
    setBoth(MID_RED, pos - 3);
    setBoth(FAINT_RED, pos - 4);

    leds.update();

    pos = (pos + 1) % HALF;  

    pros::delay(30);
}
void blue_wave() {

    static int pos = 0;

    auto setBoth = [&](uint32_t color, int index) {

        int i1 = (index + LED_COUNT) % LED_COUNT;
        int i2 = (i1 + HALF) % LED_COUNT;

        leds.set_pixel(color, i1);
        leds.set_pixel(color, i2);
    };

    setBoth(MAX_BLUE, pos);
    setBoth(BRIGHTER_BLUE, pos - 1);
    setBoth(BRIGHT_BLUE, pos - 2);
    setBoth(MID_BLUE, pos - 3);
    setBoth(FAINT_BLUE, pos - 4);

    leds.update();

    pos = (pos + 1) % HALF;

    pros::delay(25);
}
void red_waveR() {

    static int pos = 0;

    //set the dim background for all LEDs
    for (int i = 0; i < LED_COUNT; i++) {
        leds.set_pixel(BACKGROUND, i);
    }

    // Function to mirror 
    auto setBoth = [&](uint32_t color, int index) {
        int i1 = (index + LED_COUNT) % LED_COUNT;
        int i2 = (i1 + HALF) % LED_COUNT;
        leds.set_pixel(color, i1);
        leds.set_pixel(color, i2);
    };


    setBoth(MAX_RED, pos);
    setBoth(BRIGHTER_RED, pos - 1);
    setBoth(BRIGHT_RED, pos - 2);
    setBoth(MID_RED, pos - 3);
    setBoth(FAINT_RED, pos - 4);

    leds.update();

    // Reverse direction
    pos = (pos - 1 + HALF) % HALF;

    pros::delay(25);
}
void blue_waveR() {

    static int pos = 0;


    for (int i = 0; i < LED_COUNT; i++) {
        leds.set_pixel(BACKGROUNDB, i);
    }

    auto setBoth = [&](uint32_t color, int index) {
        int i1 = (index + LED_COUNT) % LED_COUNT;
        int i2 = (i1 + HALF) % LED_COUNT;
        leds.set_pixel(color, i1);
        leds.set_pixel(color, i2);
    };

 
    setBoth(MAX_BLUE, pos);
    setBoth(BRIGHTER_BLUE, pos - 1);
    setBoth(BRIGHT_BLUE, pos - 2);
    setBoth(MID_BLUE, pos - 3);
    setBoth(FAINT_BLUE, pos - 4);

    leds.update();


    pos = (pos - 1 + HALF) % HALF;

    pros::delay(25);
}

constexpr double LEAD_DEGREES = 150; // adjust if needed

int motorToLed(double degrees) {
    degrees += LEAD_DEGREES;
    if (degrees < 0) degrees = 0;
    if (degrees > 900) degrees = 900;
    return (int)(degrees / 900.0 * (HALF - 1));
}
void red_motor_track(pros::Motor &motor) {

    double deg = motor.get_position(); 
    int ledPos = motorToLed(deg);

 
    for (int i = 0; i < LED_COUNT; i++) {
        leds.set_pixel(FAINT_RED, i);
    }

    for (int i = 0; i <= ledPos; i++) {
        uint32_t color = FAINT_RED;

        if (i == ledPos) color = MAX_RED;       
        else if (i >= ledPos - 1) color = BRIGHTER_RED;
        else if (i >= ledPos - 2) color = BRIGHT_RED;
        else if (i >= ledPos - 3) color = MID_RED;

        int i1 = i;
        int i2 = i + HALF;
        leds.set_pixel(color, i1);
        leds.set_pixel(color, i2);
    }

    leds.update();
}
void blue_motor_track(pros::Motor &motor) {

    double deg = motor.get_position(); 
    int ledPos = motorToLed(deg);

    
    for (int i = 0; i < LED_COUNT; i++) {
        leds.set_pixel(FAINT_BLUE, i);
    }

  
    for (int i = 0; i <= ledPos; i++) {
        uint32_t color = FAINT_BLUE;

        if (i == ledPos) color = MAX_BLUE;        
        else if (i >= ledPos - 1) color = BRIGHTER_BLUE;
        else if (i >= ledPos - 2) color = BRIGHT_BLUE;
        else if (i >= ledPos - 3) color = MID_BLUE;

      
        int i1 = i;
        int i2 = i + HALF;
        leds.set_pixel(color, i1);
        leds.set_pixel(color, i2);
    }

    leds.update();
}
