#include "vex.h"
#include "motor-control.h"
#include "../custom/include/autonomous.h"
#include "../custom/include/robot-config.h"

// Modify autonomous, driver, or pre-auton code below

int auton_selected = 1;

std::string getAutoName(int auton_num) {
  switch(auton_num) {
    case 1: return "soloAwp";
    case 2: return "sixThreeLeft";
    case 3: return "sixThreeRight";
    case 4: return "left7";
    case 5: return "right7";
    case 6: return "skills";
    case 7: return "Left Descore";
    case 8: return "Right Descore";
    case 9: return "Skills";
    default: return "Drive Forward";
  }
}

std::string autoName = getAutoName(auton_selected);

void runAutonomous() {
  switch(auton_selected) {
    case 1:
      soloAwp();
      break;
    case 2:
      sixThreeLeft();
      break;  
    case 3:
      sixThreeRight();
      break;
    case 4:
      left7();
      break; 
    case 5:
      right7();
      break;
    case 6:
      skills();
      break;
    case 7:
      descoreLeft();
      break;
    case 8:
      descoreRight();
      break;
    case 9:
      skills();
      break;
    default:
      driveForward();
      break;
  }
}


void nextAuto() {
  auton_selected++;
  if (auton_selected > 9) auton_selected = 1;
}

void prevAuto() {
  auton_selected--;
  if(auton_selected < 1) auton_selected = 9;
}

void autoSel() {
  while (true)
  {
    controller_1.ButtonX.pressed(nextAuto);
    controller_1.ButtonUp.pressed(prevAuto);
  }
  
}

void control_loader() {
  loader.set(!loader.value());
}

void control_descore() {
  descore.set(!descore.value());
}

void intake() {
  intake1.spin(forward, 12, volt);
  intake2.spin(forward, 12, volt);
  intake3.spin(forward, 12, volt);
  hood.set(false);
}

void outtake() {
  intake1.spin(reverse, 12, volt);
  intake2.spin(reverse, 12, volt);
  intake3.spin(forward, 12, volt);
}

void score() {
  intake1.spin(forward, 12, volt);
  intake2.spin(forward, 12, volt);
  intake3.spin(forward, 12, volt);
  hood.set(true);
}

void score_mid() {
  intake1.spin(forward, 12, volt);
  intake2.spin(forward, 12, volt);
  intake3.spin(reverse, 12, volt);
  hood.set(true);
}

void stop(brakeType mode) {
  intake1.stop(mode);
  intake2.stop(mode);
  intake3.stop(mode);
}

double left_curve_function(double x, double left_curve_scale) {
  if (left_curve_scale != 0) {
    // if (CURVE_TYPE)
    return (powf(2.718, -(left_curve_scale / 10)) + powf(2.718, (fabs(x) - 127) / 10) * (1 - powf(2.718, -(left_curve_scale / 10)))) * x;
    // else
    // return powf(2.718, ((abs(x)-127)*RIGHT_CURVE_SCALE)/100)*x;
  }
  return x;
}

void set_tank(int l_stick, int r_stick) {
  left_chassis.spin(forward, l_stick * (12.0 / 127.0), volt);

  right_chassis.spin(forward, r_stick * (12.0 / 127.0), volt);
}

void tank_drive(double curve /* default is 7 in hpp file */) {
    // Put the joysticks through the curve function
    double lYcord = controller_1.Axis3.value();
    double rYcord = controller_1.Axis2.value();

    double lXCord;
    double rXCord;

    lYcord > 0 ? lXCord = abs(controller_1.Axis4.value()) : lXCord = -abs(controller_1.Axis4.value());
    rYcord > 0 ? rXCord = abs(controller_1.Axis1.value()) : rXCord = -abs(controller_1.Axis1.value());

    int l_stick = left_curve_function(lYcord + lXCord, curve);
    int r_stick = left_curve_function(rYcord + rXCord, curve);

    // Set robot to l_stick and r_stick, check joystick threshold, set active brake
    set_tank(l_stick, r_stick);
}

void leftDescore() {
  auton_selected = 7;
  autoName = getAutoName(auton_selected);
  runAutonomous();
}

void rightDescore() {
  auton_selected = 8;
  autoName = getAutoName(auton_selected);
  runAutonomous();
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
    double curve = 5.0;
    //double left_curve = (powf(2.718, -(curve / 10)) + powf(2.718, (fabs(ch3) - 127) / 10) * (1 - powf(2.718, -(curve / 10)))) * ch3;
    //right_curve = (powf(2.718, -(curve / 10)) + powf(2.718, (fabs(ch2) - 127) / 10) * (1 - powf(2.718, -(curve / 10)))) * ch2;
    //driveChassis(left_curve, right_curve);

    tank_drive(curve);
    

    if (l2) {
      wing.set(true);
    } else {
      wing.set(false);
    }

    controller_1.ButtonDown.pressed(control_loader);
    controller_1.ButtonY.pressed(control_descore);

    if (r1) {
      intake();
    } else if (r2) {
      outtake();
    } else if (l1) {
      score();
    } else if (button_b) {
      score_mid();
    } else {
      stop(coast);
    }

    if (button_left_arrow)
    {
      resetPositionBack();
      resetPositionLeft();
      Brain.Screen.printAt(1, 40, "X: %.2f, Y: %.2f, T: %.2f", x_pos, y_pos, inertial_sensor.heading());
    }
    
    if (button_right_arrow)
    {
      resetPositionBack();
      resetPositionRight();
      Brain.Screen.printAt(1, 40, "X: %.2f, Y: %.2f, T: %.2f", x_pos, y_pos, inertial_sensor.heading());
    }

    //controller_1.ButtonLeft.pressed(leftDescore);
    //controller_1.ButtonA.pressed(rightDescore);

    wait(10, msec); 
  }
}

void stayInSizeThread() {
  while (true)
  {
    if ((loader.value() == true) && (descore.value() == true))
    {
      loader.set(false);
    }
    
  }
  
}

void runPreAutonomous() {
    // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  // Calibrate inertial sensor
  inertial_sensor.calibrate();

  // Wait for the Inertial Sensor to calibrate
  while (inertial_sensor.isCalibrating()) {
    wait(10, msec);
  }
  controller_1.rumble(".");

  double current_heading = inertial_sensor.heading();
  Brain.Screen.print(current_heading);
  
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

  thread Size = thread(stayInSizeThread);
}