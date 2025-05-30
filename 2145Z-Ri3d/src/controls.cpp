#include "controls.hpp"
#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "subsystems.hpp"

// ** @file controls.cpp
// ** @brief This file contains the control functions for the robot.
// ** @details This includes the driver and autonomous controls, as well as the tasks connecting both
// ** @author Ansh Rao - 2145Z

#pragma region global

// @brief Initializes the robot's subsystems
void initAll() {
    // Initializes the chassis and brain screen
    //calibrationScreenInit();
    chassis.initialize();
    default_constants();
    ez::as::initialize();
    // add more init functions when more subsystems are added
}

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
#pragma endregion

#pragma region intake
// @brief Sets the intake voltage
// @param vltg The voltage to set the intake to
// @details This function sets the voltage of the intake motor to the specified value.
void set_intake(int vltg) { intake_vltg = vltg; }

// @brief Controls the intake based on button presses
// @details This function checks if the intake button is pressed and sets the intake motor to the max voltage.
// If the outtake button is pressed, it sets the intake motor to the negative max voltage. If neither button is pressed, it sets the intake motor to 0.
// @note This function is called in a loop to continuously check for button presses and control the intake motor accordingly.
void control_intake() {
    if (isAuto) {return;}
    else if (controlla.get_digital_new_press(BUTTON_INTAKE)) {set_intake(12000);}   
    else if (controlla.get_digital_new_press(BUTTON_OUTTAKE)) {set_intake(-12000);}
    else {set_intake(0);}
}

void intake_t() {
    pros::delay(100);
    while (true) {
        control_intake();
        motor_intake.move_velocity(intake_vltg);
        pros::delay(ez::util::DELAY_TIME);
    }
}
#pragma endregion

#pragma region rollers
// @brief Sets the rollers voltage
// @param vltg The voltage to set the rollers to
// @details This function sets the voltage of the rollers motor to the specified value.
void set_rollers(int vltg) { rollers_vltg = vltg; }

// @brief Controls the rollers based on button presses
// @details This function checks if the rollers button is pressed and sets the rollers motor to the max voltage.
// If the outrollers button is pressed, it sets the rollers motor to the negative max voltage. If neither button is pressed, it sets the rollers motor to 0.
// @note This function is called in a loop to continuously check for button presses and control the rollers motor accordingly.
void control_rollers() {
    if (isAuto) {return;}
    else if (controlla.get_digital_new_press(BUTTON_ROLLERS)) {set_rollers(12000);}   
    else if (controlla.get_digital_new_press(BUTTON_OUTROLLERS)) {set_rollers(-12000);}
    else {set_rollers(0);}
}

void rollers_t() {
    pros::delay(100);
    while (true) {
        control_rollers();
        motor_roller1.move_voltage(rollers_vltg);
        motor_roller2.move_voltage(rollers_vltg);
        pros::delay(ez::util::DELAY_TIME);
    }
}
#pragma endregion