#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>
#include <thread>

#include "../include/autonomous.h"
#include "motor-control.h"
#include "../include/user.h"

// IMPORTANT: Remember to add respective function declarations to custom/include/autonomous.h
// Call these functions from custom/include/user.cpp
// Format: returnType functionName() { code }

void driveForward() {
  driveTo(4, 1000, true);
}

void exampleAuton() {
  // Use this for tuning linear and turn pid
  driveTo(40, 4000);
  turnToAngle(90, 2000);
  turnToAngle(135, 2000);
  turnToAngle(150, 2000);
  turnToAngle(160, 2000);
  turnToAngle(165, 2000);
  turnToAngle(0, 2000);
  driveTo(-40, 3000);
}

void exampleAuton2() {
  moveToPoint(24, 24, 1, 2000, false);
  moveToPoint(48, 48, 1, 2000, true);
  moveToPoint(24, 24, -1, 2000, true);
  moveToPoint(0, 0, 1, 2000, true);
  correct_angle = 0;
  driveTo(24, 2000, false, 8);
  turnToAngle(90, 800, false);
  turnToAngle(180, 800, true);
}

void loader_async_SAWP() {
  wait(600, msec);
  loader.set(true);
  wait(300, msec);
  loader.set(false);
  wait(600, msec);
  loader.set(true);
}


void descoreLeft() {
  turnToAngle(-1, 1000, true);
  driveTo(16, 2000, true, 8);

}

void descoreRight() {
  driveTo(3, 800, false, 6);
  turnToAngle(-150, 800, false, 6);
  correct_angle = normalizeTarget(-110);
  driveTo(-4, 1500, false, 4);
  turnToAngle(-177, 500, true);
  driveTo(-23, 2000, true, 8);
}

void soloAwp() {
  resetPositionLeft();
  resetPositionBack();
  Brain.Screen.printAt(1, 20, "X: %.2f, Y: %.2f, T: %.2f", x_pos, y_pos, inertial_sensor.heading());
  intake();
  moveToPoint(-48, 3, 1, 500, false);
  stop(coast);
  moveToPoint(-48, -46.5, -1, 2000, true);
  loader.set(true);
  turnToAngle(-90, 500, true);
  intake();
  driveTo(30, 1000, true);
  correct_angle = -89;
  driveTo(-32, 1000, true);
  outtake();
  wait(50, msec); 
  score();
  wait(1000, msec);
  loader.set(false);
  resetPositionLeft();
  stop(coast);

  turnToAngle(5, 1000, true);
  resetPositionLeft();
  resetPositionBack();
  intake();
  thread la1 = thread(loader_async_SAWP);
  moveToPoint(-25, -23, 1, 1000, false, 12);

  correct_angle = 0;
  driveTo(48, 2000, true);
  correct_angle = -50;
  driveTo(-24, 1000, true);
  outtake();
  intake1.spin(forward, 1, volt);
  turnToAngle(-45, 1000, true);  
  score_mid();
  wait(250, msec);
  outtake();
  wait(75, msec);
  score_mid();
  resetPositionFront();
  wait(250, msec);

  intake();
  driveTo(53, 2000, true);
  turnToAngle(-90, 1000, true);
  driveTo(20, 1000, false);
  correct_angle = -91;
  driveTo(-32, 1000, true);
  outtake();
  wait(100, msec);
  score();
  wait(500, msec);
  outtake();
  wait(100, msec);
  score();
  wait(500, msec);
}

void fourGoals() {
  resetPositionLeft();
  resetPositionBack();
  Brain.Screen.printAt(1, 20, "X: %.2f, Y: %.2f, T: %.2f", x_pos, y_pos, inertial_sensor.heading());
  intake();
  moveToPoint(-48, 3, 1, 500, false);
  moveToPoint(-48, -48, -1, 10000, false);
  loader.set(true);
  turnToAngle(-89, 500, true);
  driveTo(16, 500, true);
  wait(350, msec);
  driveTo(-32, 1000, true);
  score();
  wait(1000, msec);
  loader.set(false);
  resetPositionLeft();
  stop(coast);


  //driveTo(5, 500, false);
  turnToAngle(5, 1000, true);
  resetPositionLeft();
  resetPositionBack();
  intake1.spin(forward, 12, volt);
  intake2.spin(reverse, 12, volt);
  intake3.spin(reverse, 12, volt);
  moveToPoint(-25, -23, 1, 1000, true, 9);
  loader.set(true);
  turnToPoint(-6.5,-6.5, 1, 500);
  loader.set(false);
  driveTo(16, 1000, true);
  outtake();
  wait(1000, msec);


  driveTo(-8, 1000, false);
  turnToAngle(-5, 1000, true);
  resetPositionLeft();
  resetPositionBack();
  moveToPoint(-22, 25, 1, 2000, true);
  loader.set(true);
  turnToAngle(-15, 1000, false);
  correct_angle = -30;
  driveTo(-24, 1000, true);
  correct_angle = -45;
  score_mid();
  wait(1000, msec);

  intake();
  moveToPoint(-50, 48, 1, 3000, true);

}

void loader_async_l7() {
  wait(550, msec);
  loader.set(true);
}

void left7() {
  resetPositionBack();
  resetPositionLeft();
  intake();
  wing.set(true);
  thread la1 = thread(loader_async_l7);
  moveToPoint(-21.75, -27.25, 1, 2000, false, 12);
  loader.set(true);
  driveChassis(6, 6);
  wait(150, msec);
  moveToPoint(-38, -58, 1, 2000, false, 12);
  loader.set(true);
  correct_angle = normalizeTarget(180);
  driveTo(36, 500, false, 7);
  driveTo(36, 500, false, 4);
  correct_angle = normalizeTarget(179);
  driveTo(-36, 1000, false, 9);
  outtake();
  wait(100, msec);
  score();
  wait(500, msec);
  outtake();
  wait(100, msec);
  score();
  wait(650, msec);
  loader.set(false);
  wing.set(false);
  driveTo(3, 800, false, 6);
  turnToAngle(-150, 800, false, 6);
  correct_angle = normalizeTarget(-110);
  driveTo(-4, 1500, false, 4);
  turnToAngle(-177, 500, true);
  driveTo(-23, 2000, true, 8);
  stopChassis(hold);
}

void loader_async_r7() {
  wait(550, msec);
  loader.set(true);
}

void right7() {
  // X = 13; Y = 48.75
  resetPositionBack();
  resetPositionRight();
  intake();
  wing.set(true);
  thread la1 = thread(loader_async_r7);
  moveToPoint(21.75, -27.25, 1, 2000, false, 12);
  loader.set(true);
  driveChassis(6, 6);
  wait(150, msec);
  moveToPoint(56, -58, 1, 2000, false, 12);
  loader.set(true);
  correct_angle = normalizeTarget(180);
  driveTo(1000, 500, false, 7);
  driveTo(1000, 500, false, 4);
  correct_angle = normalizeTarget(179);
  driveTo(-1000, 1300, false, 9);
  outtake();
  wait(100, msec);
  score();
  wait(500, msec);
  outtake();
  wait(100, msec);
  score();
  wait(650, msec);
  loader.set(false);
  wing.set(false);
  driveTo(3, 800, false, 6);
  turnToAngle(-150, 800, false, 6);
  correct_angle = normalizeTarget(-110);
  driveTo(-4, 1500, false, 4);
  turnToAngle(-177, 500, true);
  driveTo(-23, 2000, true, 8);
  stopChassis(hold);
}

void loader_async_63L() {
  wait(400, msec);
  loader.set(true);
  wait(500, msec);
  loader.set(false);
  wait(500, msec);
  loader.set(true);
}

void sixThreeLeft() {
  resetPositionBack();
  resetPositionLeft();
  intake();
  thread la1 = thread(loader_async_63L);
  wing.set(true);

  moveToPoint(-24.5, -22, 1, 2000, false);

  boomerang(-45, -12.5, 1, -70, 0.2, 2000, true);
  correct_angle = normalizeTarget(-80);
  driveTo(-10, 1000, false);
  correct_angle = normalizeTarget(-170);
  driveTo(9, 1000, true);

  moveToPoint(-48, -48, 1, 2000, true);
  turnToAngle(-180, 1000, true);
  intake();
  driveTo(1000, 1000, false, 6);
  correct_angle = normalizeTarget(-179);

  driveTo(-100, 1000, false);
  turnToAngle(-180, 1000, true);
  outtake();
  wait(100, msec);
  score();
  wait(500, msec);
  outtake();
  wait(100, msec);
  score();
  wait(650, msec);

  driveTo(4, 1000, false);
  correct_angle = normalizeTarget(-90);
  driveTo(-4, 1000, false);
  correct_angle = normalizeTarget(-135);
  driveTo(-48, 2000, true);
  score_mid();
  intake3.spin(reverse, 4, volt);

  wait(1000, msec);

  driveTo(40, 2000, true);
  wing.set(false);
  turnToAngle(-180, 1000, true);
  driveTo(-26, 2000, true);
  stopChassis(hold);
}

void loader_async_63R() {
  wait(575, msec);
  loader.set(true);
  wait(250, msec);
  loader.set(false);
  wait(400, msec);
  loader.set(true);
}

void sixThreeRight() {
  resetPositionBack();
  resetPositionRight();
  intake();
  thread la1 = thread(loader_async_63R);
  wing.set(true);

  moveToPoint(29.5, -17, 1, 2000, false);

  boomerang(52, -4, 1, 70, 0.2, 2000, true);
  correct_angle = normalizeTarget(80);
  driveTo(-8, 1000, false);
  correct_angle = normalizeTarget(170);
  driveTo(9, 1000, true);

  moveToPoint(68, -46, 1, 2000, true);
  turnToAngle(180, 1000, true);
  intake();
  driveTo(1000, 1000, false, 6);
  correct_angle = normalizeTarget(179);

  driveTo(-100, 1000, false);
  turnToAngle(180, 1000, true);
  outtake();
  wait(100, msec);
  score();
  wait(500, msec);
  outtake();
  wait(100, msec);
  score();
  wait(650, msec);

  driveTo(4, 1000, false);
  correct_angle = normalizeTarget(-90);
  driveTo(4, 1000, false);
  correct_angle = normalizeTarget(-45);
  driveTo(48, 2000, true);
  outtake();

  wait(1000, msec);

  driveTo(-40, 2000, true);
  wing.set(false);
  turnToAngle(0, 1000, true);
  driveTo(14, 2000, true);
  stopChassis(hold);
}

void loader_async_skills() {
  wait(550, msec);
  loader.set(true);
}

void intakeAfterScoring() {
  score();
  intake3.spin(reverse, 1, volt);
  wait(150, msec);
  intake();
}

void loader_async_skills2() {
  wait(200, msec);
  loader.set(true);
}


void skills() {
  resetPositionBack();
  resetPositionLeft();
  thread la1 = thread(loader_async_skills);
  intake();
  intake3.spin(forward, 0, volt);
  moveToPoint(-22, -27, 1, 2000, false);
  loader.set(true);
  turnToAngle(-90, 1000, false);
  correct_angle = -120;
  driveTo(-20, 1000, true);
  hood.set(true);
  outtake();
  intake3.spin(reverse, 4, volt);
  wait(200, msec);
  score_mid();
  hood.set(true);
  intake3.spin(reverse, 4, volt);
  wait(500, msec);
  turnToAngle(-135, 1000, true);
  outtake();
  wait(200, msec);
  score_mid();
  intake3.spin(reverse, 3.5, volt);
  wait(500, msec);

  intake();
  driveTo(56, 2000, true);
  turnToAngle(-180, 1000, true);
  driveTo(20, 2000, false);
  driveTo(-6, 1000, true);
  turnToAngle(-215, 1000, true);
  driveTo(-24, 1000, true);
  turnToAngle(-180, 1000, true);
  stop(coast);
  driveTo(-64, 2000, true);
  loader.set(false);
  turnToAngle(-90, 1000, false);
  driveTo(10, 1000, true);
  stopChassis(hold);
  resetPositionFront();
  resetPositionRight();
  stopChassis(coast);

  driveTo(-15.25, 1000, true);
  turnToAngle(0, 1000, true);
  driveTo(-12, 1000, false);
  hood.set(true);
  loader.set(true);
  outtake();
  wait(100, msec);
  score();
  wait(750, msec);
  outtake();
  wait(100, msec);
  score();
  wait(750, msec);

  thread ias = thread(intakeAfterScoring);
  correct_angle = 2;
  driveTo(36, 3000, true, 4);
  driveTo(-32, 2000, false, 6);
  hood.set(true);
  ias.interrupt();
  outtake();
  wait(100, msec);
  score();
  wait(750, msec);
  outtake();
  wait(100, msec);
  score();
  wait(750, msec);
  loader.set(false);

  turnToAngle(95, 1000, true);
  loader.set(false);
  intake1.spin(forward, 12, volt);
  intake2.spin(reverse, 3, volt);
  intake3.spin(reverse, 3, volt);
  driveTo(10, 1000, true);
  loader.set(true);
  wait(200, msec);
  loader.set(false);
  correct_angle = 135;
  driveTo(20, 2000, true);
  outtake();
  intake1.spin(reverse, 6, volt);
  wait(50, msec);
  intake3.spin(reverse, 12, volt);
  wait(1000, msec);

  intake();
  driveTo(-14, 2000, true);
  turnToAngle(90, 1000, true);
  // driveTo(48, 3000, true);

  // turnToAngle(60, 1000, false);
  // correct_angle = 40;
  // driveTo(-24, 1000, true);
  // hood.set(true);
  // outtake();
  // intake3.spin(reverse, 4, volt);
  // wait(200, msec);
  // score_mid();
  // hood.set(true);
  // intake3.spin(reverse, 4, volt);
  // wait(500, msec);
  // turnToAngle(45, 1000, true);
  // outtake();
  // wait(200, msec);
  // score_mid();
  // intake3.spin(reverse, 3.5, volt);
  // wait(500, msec);


}