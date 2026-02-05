#include "vex.h"
#include "motor-control.h"
#include "../custom/include/autonomous.h"
#include "../custom/include/robot-config.h"

#ifdef USE_LVGL
#include "v5lvgl.h"
#endif

// Modify autonomous, driver, or pre-auton code below

void runAutonomous() {
  int auton_selected = 3;
  switch(auton_selected) {
    case 1:
      exampleAuton();
      break;
    case 2:
      exampleAuton2();
      break;  
    case 3:
      redGoalRush();
      break;
    case 4:
      break; 
    case 5:
      break;
    case 6:
      break;
    case 7:
      break;
    case 8:
      break;
    case 9:
      break;
  }
}

// controller_1 input variables (snake_case)
int ch1, ch2, ch3, ch4;
bool l1, l2, r1, r2;
bool button_a, button_b, button_x, button_y;
bool button_up_arrow, button_down_arrow, button_left_arrow, button_right_arrow;
int chassis_flag = 0;

void runDriver() {
  stopChassis(coast);
  heading_correction = false;
  while (true) {
    // [-100, 100] for controller stick axis values
    ch1 = controller_1.Axis1.value();
    ch2 = controller_1.Axis2.value();
    ch3 = controller_1.Axis3.value();
    ch4 = controller_1.Axis4.value();

    // true/false for controller button presses
    l1 = controller_1.ButtonL1.pressing();
    l2 = controller_1.ButtonL2.pressing();
    r1 = controller_1.ButtonR1.pressing();
    r2 = controller_1.ButtonR2.pressing();
    button_a = controller_1.ButtonA.pressing();
    button_b = controller_1.ButtonB.pressing();
    button_x = controller_1.ButtonX.pressing();
    button_y = controller_1.ButtonY.pressing();
    button_up_arrow = controller_1.ButtonUp.pressing();
    button_down_arrow = controller_1.ButtonDown.pressing();
    button_left_arrow = controller_1.ButtonLeft.pressing();
    button_right_arrow = controller_1.ButtonRight.pressing();

    // default tank drive or replace it with your preferred driver code here: 
    driveChassis(ch3 * 0.12, ch2 * 0.12);

    wait(10, msec); 
  }
}

#ifdef USE_LVGL
// LVGL example: button + label. Called once after v5_lv_init().
static void lvgl_btn_cb(lv_event_t * e) {
  lv_obj_t * label = (lv_obj_t *)lv_event_get_user_data(e);
  if (label)
    lv_label_set_text(label, "Button pressed!");
}

static void lvgl_example_screen(void) {
  lv_obj_t * scr = lv_scr_act();
  lv_obj_t * label = lv_label_create(scr);
  lv_label_set_text(label, "LVGL 8.4 Ready");
  lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 40);

  lv_obj_t * btn = lv_btn_create(scr);
  lv_obj_align(btn, LV_ALIGN_CENTER, 0, 20);
  lv_obj_set_size(btn, 120, 50);
  lv_obj_add_event_cb(btn, lvgl_btn_cb, LV_EVENT_CLICKED, label);

  lv_obj_t * btn_label = lv_label_create(btn);
  lv_label_set_text(btn_label, "Click me");
  lv_obj_align(btn_label, LV_ALIGN_CENTER, 0, 0);
}
#endif

void runPreAutonomous() {
    // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

#ifdef USE_LVGL
  // Start LVGL and show example screen (button + label on Brain)
  v5_lv_init();
  lvgl_example_screen();
#endif
  
  // Calibrate inertial sensor
  inertial_sensor.calibrate();

  // Wait for the Inertial Sensor to calibrate
  while (inertial_sensor.isCalibrating()) {
    wait(10, msec);
  }

  double current_heading = inertial_sensor.heading();
#ifndef USE_LVGL
  Brain.Screen.print(current_heading);
#endif
  
  // odom tracking
  resetChassis();
  if(using_horizontal_tracker && using_vertical_tracker) {
    thread odom = thread(trackXYOdomWheel);
  } else if (using_horizontal_tracker) {
    thread odom = thread(trackXOdomWheel);
  } else if (using_vertical_tracker) {
    thread odom = thread(trackYOdomWheel);
  } else {
    thread odom = thread(trackNoOdomWheel);
  }
}