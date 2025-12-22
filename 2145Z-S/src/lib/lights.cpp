#include "lib/lights.hpp"
#include "pros/misc.h"
#include "subsystems.hpp"
#include <algorithm>
#include <cmath>
#include <vector>


RGB hsvToRgb(const HSV &hsv) {
    RGB rgb(0, 0, 0);

    if (hsv.s == 0) { // zero saturation
        rgb.r = rgb.g = rgb.b = hsv.v * 255;
        return rgb;
    }
    
    double c = hsv.v * hsv.s; // Chroma
    double x = c * (1 - fabs(fmod(hsv.h / 60.0, 2) - 1));
    double m = hsv.v - c; // Match value

    double r, g, b;

    // Determine the RGB components based on the hue value
    if (hsv.h >= 0 && hsv.h < 60) {
        r = c; g = x; b = 0;
    } else if (hsv.h >= 60 && hsv.h < 120) {
        r = x; g = c; b = 0;
    } else if (hsv.h >= 120 && hsv.h < 180) {
        r = 0; g = c; b = x;
    } else if (hsv.h >= 180 && hsv.h < 240) {
        r = 0; g = x; b = c;
    } else if (hsv.h >= 240 && hsv.h < 300) {
        r = x; g = 0; b = c;
    } else { // hsv.h >= 300 && hsv.h < 360
        r = c; g = 0; b = x;
    }

    // Adjust the RGB values by adding m
    rgb.r = (r + m) * 255;
    rgb.g = (g + m) * 255;
    rgb.b = (b + m) * 255;

    return rgb;
}


int RgbToHex(RGB color) {
    return (static_cast<uint32_t>(color.r) << 16) | (static_cast<uint32_t>(color.g) << 8) | static_cast<uint32_t>(color.b);
}


std::vector<int> readImage(const char *filename) {
    FILE *f = fopen(filename, "rb");
    unsigned char info[54];
  
    // read the 54-byte header
    fread(info, sizeof(unsigned char), 54, f);
  
    // extract image height and width from header
    int width = *(int *)&info[18];
    int height = *(int *)&info[22];
  
    // allocate 3 bytes per pixel
    int size = 3 * width * height;
    unsigned char *data = new unsigned char[size];
  
    // read the rest of the data at once
    fread(data, sizeof(unsigned char), size, f);
    fclose(f);
    
    std::vector<int> output;
    output.reserve(width * height);
    int i = 0;
    while (i < size) {
        output.push_back(RgbToHex(RGB(data[i], data[i + 1], data[i + 2])));
        i += 3;
    }
    return output;
}


int wrapDegrees(float degrees) {
    while (degrees > 360) {
        degrees -= 360;
    }
    while (degrees < 0) {
        degrees += 360;
    }
    return degrees;
}


float lerpf(float from, float to, float alpha) {
    return from * (1.0 - alpha) + to * alpha;
}



HSV lerpHSV(HSV from, HSV to, float alpha) {
    return HSV(lerpf(from.h, to.h, alpha), lerpf(from.s, to.s, alpha), lerpf(from.v, to.v, alpha));
}



std::vector<int> interpolateSingle(HSV start, HSV end, int length) {
    std::vector<int> result;
    result.reserve(length);

    for (int i = 0; i < length; i++) {
        float alpha = static_cast<float>(i) / static_cast<float>(length);

        HSV hsv(lerpf(start.h, end.h, alpha), lerpf(start.s, end.s, alpha), lerpf(start.v, end.v, alpha));

        hsv.h = wrapDegrees(hsv.h);

        result.emplace_back(RgbToHex(hsvToRgb(hsv)));
    }
    return result;
}



std::vector<RGB> interpolateSingle2(HSV start, HSV end, int length) {
    std::vector<RGB> result;
    result.reserve(length);

    for (int i = 0; i < length; i++) {
        float alpha = static_cast<float>(i) / static_cast<float>(length);

        HSV hsv = lerpHSV(start, end, alpha);

        hsv.h = wrapDegrees(hsv.h);

        result.emplace_back(hsvToRgb(hsv));
    }
    return result;
}



std::vector<int> interpolateDouble(HSV start, HSV end, int length) {

    if (length % 2 == 1) { length++; }

    std::vector<int> result;
    result.reserve(length);

    for (int i = 0; i < length / 2; i++) {
        float alpha = static_cast<float>(i) / static_cast<float>(length);

        HSV hsv = lerpHSV(start, end, alpha);

        hsv.h = wrapDegrees(hsv.h);

        result.emplace_back(RgbToHex(hsvToRgb(hsv)));
    }
    for (int i = length / 2; i > 0; i--) {
        float alpha = static_cast<float>(i) / static_cast<float>(length);

        HSV hsv = lerpHSV(start, end, alpha);

        hsv.h = wrapDegrees(hsv.h);

        result.emplace_back(RgbToHex(hsvToRgb(hsv)));
    }
    return result;
}



void lib::LightManager::loop() {
    while (true) {
        arm->abstractInput(1.0 ? master.get_digital(pros::E_CONTROLLER_DIGITAL_Y) 
                        || master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) || master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)
                        || master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) || master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) : 0.0);
        arm->update();
        pros::delay(10);
        intake->abstractInput(1.0 ? master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) || master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) : 0.0);
        intake->update();
        pros::delay(10);
        clamp->abstractInput(1.0 ? master.get_digital(pros::E_CONTROLLER_DIGITAL_B) : 0.0);
        clamp->update();
        pros::delay(10);
        left->abstractInput(std::max(abs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)), abs(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X))) / 100.0);
        left->update();
        pros::delay(10);
        right->abstractInput(std::max(abs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)), abs(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X))) / 100.0);
        right->update();
        pros::delay(10);
    }
}



void lib::FlowingGradient::update() {
    ticksPassed++; 
    for (int i = 0; i < length(); i++) {
        set_pixel(
            allianceColor == BLUE ? blueGradient[(i + ticksPassed) % blueGradient.size()]
            : redGradient[(i + ticksPassed) % redGradient.size()],
            i);
    }
}



void lib::BreathingGradient::update() {
    ticksPassed++; 
    set_all( allianceColor == BLUE ? blueGradient[ticksPassed % blueGradient.size()]
            : redGradient[ticksPassed % redGradient.size()]);
}



void lib::Pulser::update() {
    alpha = std::clamp(alpha + delta, 0.0, 1.0);
    
    set_all(allianceColor == BLUE ? RgbToHex(hsvToRgb(lerpHSV(HSV(blue.h, blue.s, 0), blue, alpha)))
                : RgbToHex(hsvToRgb(lerpHSV(HSV(red.h, red.s, 0), red, alpha))));
}



void lib::EmaPulser::abstractInput(double input) {
    alpha = lerpf(alpha, fmin(1.0, input), 0.1);
    set_all(allianceColor == BLUE ? RgbToHex(hsvToRgb(lerpHSV(HSV(blue.h, blue.v, 0), blue, alpha)))
                : RgbToHex(hsvToRgb(lerpHSV(HSV(red.h, red.v, 0), red, alpha))));
}



void lib::AnimationReader::update() {
    if (playDirection) { playPosition = std::min(playPosition + 1, animationLength); }
    else { playPosition = std::max(int(playPosition) - 1, 0); }
    
    for (int i = 0; i < length(); i++) {
        set_pixel(
            allianceColor == BLUE ? blueAnimation[i * animationLength + playPosition]
            : redAnimation[i * animationLength + playPosition],
            i);
    }
}
