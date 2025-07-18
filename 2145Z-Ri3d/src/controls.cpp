#include "controls.hpp"
#include <sys/types.h>
#include <cstdlib>  // IWYU pragma: keep
#include "EZ-Template/util.hpp"
#include "main.h"  // IWYU pragma: keep
#include "pros/device.h"
#include "pros/device.hpp"
#include "pros/rtos.hpp"
#include "screen.hpp"
#include "subsystems.hpp"

// ** @file controls.cpp
// ** @brief This file contains the control functions for the robot.
// ** @details This includes the driver and autonomous controls, as well as the tasks connecting both
// ** @author Ansh Rao - 2145Z

// @brief Sets the default constants for the robot, including PID constants, exit conditions, and slew rates
void default_constants() {
    // P,I,D, start I
    chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
    chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
    chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
    chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
    chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
    chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions
  
    // Exit conditions
    chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
    chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
    chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
    chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
    chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
    chassis.pid_turn_chain_constant_set(3_deg);
    chassis.pid_swing_chain_constant_set(5_deg);
    chassis.pid_drive_chain_constant_set(3_in);
  
    // Slew constants
    chassis.slew_turn_constants_set(3_deg, 70);
    chassis.slew_drive_constants_set(3_in, 70);
    chassis.slew_swing_constants_set(3_in, 80);
  
    // The amount that turns are prioritized over driving in odom motions
    // - if you have tracking wheels, you can run this higher.  1.0 is the max
    chassis.odom_turn_bias_set(0.9);
  
    chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
    chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
    chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are
  
    chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

#pragma region roller

void set_roller_front(int vltg) {
    roller_front_vltg = vltg;
}

void set_roller_top(int vltg) {
    roller_top_vltg = vltg;
}

void set_roller_back(int vltg) {
    roller_back_vltg = vltg;
}

void set_rollers(int vltg) {
    set_roller_front(vltg);
    set_roller_top(vltg);
    set_roller_back(vltg);
}

void set_rollers(int frontAndBackVltg, int topVltg) {
    set_roller_front(frontAndBackVltg);
    set_roller_top(topVltg);
    set_roller_back(frontAndBackVltg);
}

void set_rollers(int frontVltg, int topVltg, int backVltg) {
    set_roller_front(frontVltg);
    set_roller_top(topVltg);
    set_roller_back(backVltg);
}

void set_rollers_intake() {
    set_rollers(12000, 12000, -12000);
}

void set_rollers_outtake() {
    set_rollers(-12000);
}

void set_rollers_score_middle() {
    set_rollers(-6000, 12000, 12000);
}

void set_rollers_score_top() {
    hoodState = false;
    set_rollers(12000);
}

void control_roller_legacy() {
    if (matchState != DRIVER) {return;}
    if (ctrlLock /*&& !overridestate()*/) {return;}
    if (controlla.get_digital(BUTTON_ROLLER)) {set_rollers(12000);}
    else if (controlla.get_digital(BUTTON_OUTROLLER)) {set_rollers(-12000);}
    else if (controlla.get_digital(BUTTON_ROLLER_CENTER)) {set_rollers(12000, -12000);}
    else if (controlla.get_digital(BUTTON_ROLLER_PEZ)) {set_rollers(12000, 12000, -12000);}
    else {set_rollers(0);}
}

void control_roller() {
    if (matchState != MatchStates::DRIVER) {return;}
    if (ctrlLock /*&& !overridestate()*/) {return;}
    if (controlla.get_digital(BUTTON_ROLLER)) {
        set_rollers_intake();
    } else if (controlla.get_digital(BUTTON_OUTROLLER)) {
        set_rollers_outtake();
    } else if (controlla.get_digital(BUTTON_ROLLER_CENTER)) {
        set_rollers_score_middle();
    } else if (controlla.get_digital(BUTTON_ROLLER_PEZ)) {
        set_rollers_score_top();
    } else {
        set_rollers(0);
    }
}

void roller_t() {

    motor_roller_front.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    motor_roller_top.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    motor_roller_back.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    while (true) {
        control_roller();
        motor_roller_front.move_voltage(roller_front_vltg);
        motor_roller_top.move_voltage(roller_top_vltg);
        motor_roller_back.move_voltage(roller_back_vltg);
        
        pros::delay(ez::util::DELAY_TIME);
    }
}

#pragma endregion

#pragma region colorSort

Alliances getTopBlockColor() {
    if (optical_top.get_proximity() > 75) {
        if (optical_top.get_plugged_type() == pros::DeviceType::none) {
            // If the optical sensor is not plugged in, return NONE
            return Alliances::NONE;
        }
        int topColor = optical_top.get_hue();
        if (topColor >= 340 || topColor <= 20) {
            return Alliances::RED;
        } else if (topColor >= 210 && topColor <= 240) {
            return Alliances::BLUE;
        } else {
            return Alliances::NONE;
        }
    }
    return Alliances::NONE;
}

Alliances getBottomBlockColor() {
    if (optical_bottom.get_proximity() > 75) {
                if (optical_bottom.get_plugged_type() == pros::DeviceType::none) {
            // If the optical sensor is not plugged in, return NONE
            return Alliances::NONE;
        }
        int bottomColor = optical_bottom.get_hue();
        if (bottomColor >= 340 || bottomColor <= 20) {
            return Alliances::RED;
        } else if (bottomColor >= 210 && bottomColor <= 240) {
            return Alliances::BLUE;
        } else {
            return Alliances::NONE;
        
        }
    }
    return Alliances::NONE;
}

bool rightBlockDetected(Alliances blockColor) {
    if (blockColor == Alliances::NONE) {return false;}
    if (allianceColor == Alliances::RED && blockColor == Alliances::RED) {return true;}
    else if (allianceColor == Alliances::BLUE && blockColor == Alliances::BLUE) {return true;}
    return false;
}

bool wrongBlockDetected(Alliances blockColor) {
    if (blockColor == Alliances::NONE) {return false;}
    if (allianceColor == Alliances::RED && blockColor == Alliances::BLUE) {return true;}
    else if (allianceColor == Alliances::BLUE && blockColor == Alliances::RED) {return true;}
    return false;
}

void colorSortLoop() {
    Alliances topBlockColor = getTopBlockColor();
    Alliances bottomBlockColor = getBottomBlockColor();
    colorSet( bottomBlockColor, colorInd);
    if (matchState != MatchStates::DISABLED) {
        int temp_roller_front_vltg = roller_front_vltg;
        int temp_roller_top_vltg = roller_top_vltg;
        int temp_roller_back_vltg = roller_back_vltg;
        if (wrongBlockDetected(bottomBlockColor)) {
            ctrlLock = true;  // Lock the controls
            setTrapdoor(true);
            set_rollers(12000);
            setHood(true);
            while (!rightBlockDetected(topBlockColor) && colorSortTime < MAXCOLORSORTTIME) {
                pros::delay(ez::util::DELAY_TIME);
                colorSortTime += ez::util::DELAY_TIME;
            }
            set_rollers(temp_roller_front_vltg, temp_roller_top_vltg, temp_roller_back_vltg);
            ctrlLock = false;
            setTrapdoor(false);
            setHood(false);
            colorSortTime = 0;  // Reset the color sort time
        }
        if (doIntakeUntilTop) {
            while (!rightBlockDetected(topBlockColor) && intakeUntilTime < MAXINTAKEUNTILTIME) {
                pros::delay(ez::util::DELAY_TIME);
                intakeUntilTime += ez::util::DELAY_TIME;
            }
            doIntakeUntilTop = false;  // Reset the intake until top flag
            intakeUntilTime = 0;  // Reset the intake until time
            set_rollers(0);  // Stop the rollers
        }
    }
}

void colorSort_t() {

    optical_bottom.set_integration_time(3);
    optical_top.set_integration_time(3);
    optical_bottom.set_led_pwm(100);
    optical_top.set_led_pwm(100);

    while (true) {
        if (doColorSort) {
            colorSortLoop();
        }
        pros::delay(ez::util::DELAY_TIME);
    }
}

#pragma endregion

#pragma region pto

void setPto(bool state) {
    if (state == true) {
        chassis.drive_rpm_set(90);
    } else if (state == false) {
        chassis.drive_rpm_set(DRIVE_RPM); //450rpm
    }
    ptoState = state;
}

void control_pto() {
    if (matchState != MatchStates::DRIVER) {return;}
    else if (controlla.get_digital_new_press(BUTTON_PTO)) {
        ptoState = !ptoState;
        setPto(ptoState);
    }
}

#pragma endregion

#pragma region loader

void setLoader(bool state) {
    loaderState = state;
}

void control_loader() {
    if (matchState != MatchStates::DRIVER) {return;}
    if (controlla.get_digital_new_press(BUTTON_LOADER)) {
        loaderState = !loaderState;
    }
}

#pragma endregion

#pragma region trapdoor

void setTrapdoor(bool state) {
    loaderState = state;
}

void control_trapdoor() {
    if (matchState != MatchStates::DRIVER) {return;}
    if (controlla.get_digital_new_press(BUTTON_TRAPDOOR)) {
        trapdoorState = !trapdoorState;
    }
}

#pragma endregion

#pragma region hood

void setHood(bool state) {
    hoodState = state;
}

void control_hood() {
    if (matchState != MatchStates::DRIVER) {return;}
    if (controlla.get_digital_new_press(BUTTON_HOOD)) {
        hoodState = !hoodState;
    }
}

#pragma endregion

void misc_t() {

    while (true) {
        control_pto();
        control_loader();
        control_hood();
        piston_hood.set(hoodState);
        piston_loader.set(loaderState);
        piston_pto.set(ptoState);
        pros::delay(ez::util::DELAY_TIME);
    }           
}