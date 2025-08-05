#include "controls.hpp"
#include <functional>
#include <string>
#include <vector>
#include "EZ-Template/util.hpp"
#include "drive.hpp"
#include "main.h"   // IWYU pragma: keep
#include "pros/abstract_motor.hpp"  // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/device.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include "screen.hpp"
#include "subsystems.hpp"

void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(30.0, 0.0, 50.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(30.0, 0.0, 55.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(6.0, 0.05, 45.0, 15.0);     // Turn in place constants
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

    chassis.pid_drive_chain_constant_set(1_in);
    chassis.pid_turn_chain_constant_set(1_deg);
}

#pragma region rollers

void set_roller_front(int vltg) {
    vltg_front = vltg;
}

void set_roller_top(int vltg) {
    vltg_top = vltg;
}

void set_roller_back(int vltg) {
    vltg_back = vltg;
}

void set_rollers(int front, int top, int back) {
    set_roller_front(front);
    set_roller_top(top);
    set_roller_back(back);
}

void set_rollers(int main, int top) {
    set_rollers(main, top, main);
}

void set_rollers(int vltg) {
    set_rollers(vltg, vltg, vltg);
}

void set_rollers(Rollers situation) {
    if (matchState == DISABLED) {
        return;
    }
    switch (situation) {
        case STOP:
            set_rollers(0);
            break;
        case INTAKE:
            set_rollers(12000);
            //doColorSort = true;
            stateHopper = false;
            stateHood = true;
            if (currentHopper == TOP) {
                stateBlocker_top = true;
            }
            break;
        case HOPPER_BOTTOM:
            set_rollers(12000, 12000, -12000);
            stateBlocker_bot = false;
            break;
        case HOPPER_TOP:
            set_rollers(12000);
            stateHood = true;
            stateBlocker_top = true;
            break;
        case OUTTAKE:
            doColorSort = false;
            set_rollers(-12000);
            break;
        case SCORE_TOP:
            doColorSort = false;
            switch (currentHopper) {
                case TOP:
                    stateBlocker_top = false;
                    stateHopper = true;
                    break;
                case BOTTOM:
                    stateBlocker_bot = false;
                    break;
                default:
                    break;
            }
            stateHood = false;
            set_rollers(12000);
            break;
        case SCORE_MID:
            doColorSort = false;
            switch (matchState) {
                case MatchStates::AUTO_PID:
                case MatchStates::AUTO_ODOM:
                    set_rollers(12000, -6000);
                    break;
                default:
                    set_rollers(12000, -12000);
                    break;
            }
            switch (currentHopper) {
                case TOP:
                    stateBlocker_top = false;
                    stateHopper = true;
                    break;
                case BOTTOM:
                    stateBlocker_bot =false;
                    break;
                default:
                    break;
            }
            break;
        case SCORE_BOT:
            set_rollers(-12000);
            switch (currentHopper) {
                case TOP:
                    stateBlocker_top = false;
                    stateHopper = true;
                    break;
                case BOTTOM:
                    stateBlocker_bot =false;
                    break;
                default:
                    break;
            }
        default:
            set_rollers(0);
            break;
    }
    curRoller = situation;
}

void control_rollers() {
    if (matchState != DRIVER) return;
    if (ctrlLock && !override) return;
    if (controlla.get_digital(BUTTON_INTAKE)) {
        set_rollers(INTAKE);
        //ColorSortTask.resume();
    }   else if (controlla.get_digital(BUTTON_OUTTAKE)) {
        //ColorSortTask.suspend();
        set_rollers(OUTTAKE);
    }   else if (controlla.get_digital(BUTTON_SCORE_MID)) {
        //ColorSortTask.suspend();
        set_rollers(SCORE_MID);
    }   else if (controlla.get_digital(BUTTON_SCORE_TOP)) {
        //ColorSortTask.suspend();
        set_rollers(SCORE_TOP);
    }   else {
        //ColorSortTask.suspend();
        set_rollers(STOP);
    }
}

bool isJammed = false;
std::vector<pros::Motor> rollerMotors = {motor_roller_front, motor_roller_top, motor_roller_back};

void checkIfJammed() {
    for (int i = 0; i < rollerMotors.size(); i++) {
        if (rollerMotors[i].get_efficiency() < 10) {
            isJammed = true;
        }
    }

    if (isJammed) {
        int temp_front = vltg_front;
        int temp_top = vltg_top;
        int temp_back = vltg_back;
        ctrlLock = true;
        set_rollers(-vltg_front, -vltg_top, -vltg_back);
        pros::delay(500);
        set_rollers(temp_front, temp_top, temp_back);
        isJammed = false;
        ctrlLock = false;
    }
}

void roller_t() {

    motor_roller_front.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    motor_roller_top.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    motor_roller_back.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    while (true) {

        control_rollers();
        motor_roller_front.move_voltage(vltg_front);
        motor_roller_top.move_voltage(vltg_top);
        motor_roller_back.move_voltage(vltg_back);
        piston_hood.set_value(stateHood);
        piston_hopper.set_value(stateHopper);
        piston_blocker_top.set_value(stateBlocker_top);
        piston_blocker_bot.set_value(stateBlocker_bot);
        pros::delay(ez::util::DELAY_TIME);
    }
}

#pragma endregion

#pragma region colorSort

Alliances get_color_top() {
    if (optical_top.get_proximity() > 20) {
        if (optical_top.get_plugged_type() == pros::DeviceType::none) {
            // If the optical sensor is not plugged in, return NONE
            return Alliances::NONE;
        }
        int color_top = optical_top.get_hue();
        if (color_top >= 340 || color_top <= 20) {
            return Alliances::RED;
        } else if (color_top >= 210 && color_top <= 240) {
            return Alliances::BLUE;
        } else {
            return Alliances::NONE;
        }
    }
    return Alliances::NONE;
}

Alliances get_color_mid() {
    if (optical_mid.get_proximity() > 20) {
                if (optical_mid.get_plugged_type() == pros::DeviceType::none) {
            // If the optical sensor is not plugged in, return NONE
            return Alliances::NONE;
        }
        int color_mid = optical_mid.get_hue();
        if (color_mid >= 340 || color_mid <= 20) {
            return Alliances::RED;
        } else if (color_mid >= 210 && color_mid <= 240) {
            return Alliances::BLUE;
        } else {
            return Alliances::NONE;
        
        }
    }
    return Alliances::NONE;
}

Alliances get_color_bot() {
    if (optical_bot.get_proximity() > 20) {
                if (optical_bot.get_plugged_type() == pros::DeviceType::none) {
            // If the optical sensor is not plugged in, return NONE
            return Alliances::NONE;
        }
        int color_bot = optical_bot.get_hue();
        if (color_bot >= 340 || color_bot <= 20) {
            return Alliances::RED;
        } else if (color_bot >= 180 && color_bot <= 240) {
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
    Alliances blockColor_top = get_color_top();
    Alliances blockColor_mid = get_color_mid();
    Alliances blockColor_bot = get_color_bot();
    colorSet(blockColor_bot, colorInd);
    if (matchState != DISABLED) {
        if (doColorSort) {
        Rollers temp_roller = curRoller;
        if (wrongBlockDetected(blockColor_bot)) {
            ctrlLock = true;
            switch (currentHopper) {
                case TOP:
                    set_rollers(HOPPER_BOTTOM);
                    break;
                case BOTTOM: 
                    set_rollers(HOPPER_TOP);
                    break;
                default:
                    break;
            }
            // int MAXSORTTIME = 1000;
            // while (!rightBlockDetected(blockColor_bot) && sortTime < MAXSORTTIME) {
            //     pros::delay(3);
            //     sortTime += 3;
            // }
            // motor_roller_back.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            // set_rollers(temp_roller);
            // stateBlocker_bot = true;
            // ctrlLock = false;
            // sortTime = 0; 
        }
        else if (rightBlockDetected(blockColor_bot)) {
            //print("right color detected");
            int MAXSORTTIME = 2000;
            ctrlLock = true;
            switch (currentHopper) {
                case TOP:
                    set_rollers(HOPPER_TOP);
                    break;
                case BOTTOM: 
                    set_rollers(HOPPER_BOTTOM);
                    break;
                default:
                    break;
            }
            // while (!wrongBlockDetected(blockColor_bot) && sortTime < MAXSORTTIME) {
            //     pros::delay(3);
            //     sortTime += 3;
            // }
            // set_rollers(temp_roller);
            // ctrlLock = false;
        // }   else {
        //     ctrlLock = false;
        }
        }
    }
}

// TODO: might changed to prox based depending on consistency of color sort and skills
// TODO: might change delayMillies to pros::delay if the screen work or not
void load_until(int blocks) {
    const int MAXLOADTIME = blocks * 750;
    if (matchState == DISABLED) {
        wait(MAXLOADTIME);
        return;
    }
    int loadTime = 0;
    int loadedBlocks = 0;
    set_rollers(INTAKE);
    while (!rightBlockDetected(get_color_bot()) && loadTime < 250) {
        pros::delay(3);
        loadTime += 3;
    }
    loadTime = 0;
    while (loadedBlocks < blocks && loadTime < MAXLOADTIME) {
        if (get_color_bot() == allianceColor) {
            int timer = 0;
            while (get_color_bot() == allianceColor) {
                pros::delay(3);
                timer += 3;
                if (timer > 3000) {
                    return;
                }
            }
            loadedBlocks++;
            print("blocks loaded: " + std::to_string(loadedBlocks));
        }
        loadTime += 3;
        pros::delay(3);
    }
    set_rollers(STOP);
}

// TODO: might change to prox based depending on consistency of color sort and skills
// TODO: might change delayMillies to pros::delay if the screen work or not
void score_until(Rollers situation, int blocks) {
    const int MAXSCORETIME = blocks * 500;
    if (matchState == DISABLED) {
        wait(MAXSCORETIME);
        return;
    }
    int scoreTime = 0;
    int scoredBlocks = 0;
    switch (situation) {
        case SCORE_MID:
            set_rollers(SCORE_MID);
            while (!rightBlockDetected(get_color_mid()) && scoreTime < 250) {
                wait(3);
                scoreTime += 3;
            }
            scoreTime = 0;
            while (scoredBlocks < blocks && scoreTime < MAXSCORETIME) {
                if (get_color_mid() == allianceColor) {
                    int timer = 0;
                    while (get_color_mid() == allianceColor) {
                        pros::delay(3);
                        timer += 3;
                        if (timer > 3000) {
                            return;
                        }
                    }
                    scoredBlocks++;
                    print("scored blocks: " + std::to_string(scoredBlocks));
                }
            }
            set_rollers(HOPPER_TOP);

        case SCORE_TOP:
            set_rollers(SCORE_TOP);
            while (!rightBlockDetected(get_color_top()) && scoreTime < 250) {
                pros::delay(3);
                scoreTime += 3;
            }
            scoreTime = 0;
            while (scoredBlocks < blocks && scoreTime < MAXSCORETIME) {
                if (get_color_top() == allianceColor) {
                    while (get_color_top() == allianceColor) {
                        pros::delay(3);
                    }
                    scoredBlocks++;
                }
            }
            set_rollers(HOPPER_TOP);
        default:
            break;
    }

} 

void colorSort_t() {

    optical_top.set_integration_time(3);
    optical_mid.set_integration_time(3);
    optical_bot.set_integration_time(3);
    optical_top.set_led_pwm(100);
    optical_mid.set_led_pwm(100);
    optical_bot.set_led_pwm(100);

    while (true) {
        if(doColorSort) colorSortLoop();
        //print(3, "Color sort active: " + std::to_string(doColorSort));
        pros::delay(3);
    }
}

#pragma endregion

#pragma region pto

void set_pto(bool state) {
    statePto = state;
    state == true ? chassis.drive_rpm_set(90) : chassis.drive_rpm_set(DRIVE_RPM);
}

void control_pto() {
    if (matchState != DRIVER ) return;
    if (controlla.get_digital_new_press(BUTTON_PTO)) {
        statePto = !statePto;
    }
    set_pto(statePto);
}

#pragma endregion

#pragma region loader 

void set_loader(bool state) {
    stateLoader = state;
}

void control_loader() {
    if (matchState != DRIVER) return;
    if (controlla.get_digital_new_press(BUTTON_LOADER)) {
        stateLoader = !stateLoader;
    }
}

#pragma endregion

#pragma region punchers

void set_puncher_mid(bool state) {
    statePuncherMid = state;
}

void set_puncher_top(bool state) {
    statepuncherTop = state;
}

void control_punchers() {
    if (matchState != DRIVER) return;
    if (controlla.get_digital_new_press(BUTTON_PUNCHER_MID)) {
        set_puncher_mid(!statePuncherMid);
    } else if (controlla.get_digital_new_press(BUTTON_PUNCHER_TOP)) {
        set_puncher_top(!statepuncherTop);   
    }
}

#pragma endregion

void misc_t() {
    while (true) {
        control_loader();
        control_punchers();
        control_pto();
        if (controlla.get_digital_new_press(BUTTON_OVERRIDE)) {
            override = !override;
        }
        piston_loader.set_value(stateLoader);
        piston_puncher_mid.set_value(statePuncherMid);
        piston_puncher_top.set_value(statepuncherTop);
        piston_pto.set_value(statePto);
        print(1, "x " + std::to_string(chassis.odom_pose_get().x));
        print(2, "y " + std::to_string(chassis.odom_pose_get().y));
        print(3, "theta " + std::to_string(chassis.odom_pose_get().theta));
        pros::delay(ez::util::DELAY_TIME);
    }
}