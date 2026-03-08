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

// void loader_async_SAWP() {
//   wait(600, msec);
//   loader.set(true);
//   wait(300, msec);
//   loader.set(false);
//   wait(600, msec);
//   loader.set(true);
// }

void loader_async_SAWP() {
  wait(500, msec);
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
  moveToPoint(-48, -45.5, -1, 2000, true, 9);
  loader.set(true);
  turnToAngle(-90, 500, true);
  intake();
  driveTo(30, 1000, true, 5.5);
  correct_angle = -89;
  driveTo(-32, 1000, true);
  outtake();
  hood.set(false);
  wait(50, msec);
  score();
  wait(250, msec);
  outtake();
  wait(50, msec);
  score();
  wait(650, msec);
  loader.set(false);
  stop(coast);
  // 5/10 in og
  turnToAngle(12.5, 1500, true);
  intake();


  /*thread la1 = thread(loader_async_SAWP);
  moveToPoint(-25, -23, 1, 1000, false, 12);

  correct_angle = 0;
  driveTo(48, 2000, true);
  correct_angle = -50;
  driveTo(-24, 1000, true);
  */

  driveTo(30, 2000, true);
  //turnToAngle(0.0, 500, false);
  correct_angle = -14.0;
  // -1 og
  thread la1 = thread(loader_async_SAWP);
  driveTo(32, 2000, true);
  correct_angle = -48.5;
  driveTo(-18.5, 1000, false);
  turnToAngle(-45, 1000, true); 
  hood.set(true);
  score();
  wait(15, msec);
  score_mid();
  intake3.spin(reverse, 12.0, volt);
  wait(300, msec);

  intake();
  driveTo(50.0, 2500, true);
  turnToAngle(-90, 1000, true);
  driveTo(21, 1000, false, 6);
  correct_angle = -91;
  driveTo(-36, 1000, true);
  outtake();
  hood.set(false);
  wait(50, msec);
  score();
  wait(250, msec);
  outtake();
  wait(50, msec);
  score();
}

void soloAwp2() {
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
  driveTo(30, 1000, true, 6);
  correct_angle = -89;
  driveTo(-32, 1000, true);
  outtake();
  wait(75, msec);
  score();
  wait(400, msec);
  outtake();
  wait(75, msec);
  score();
  wait(400, msec);
  loader.set(false);
  stop(coast);

  turnToAngle(4.5, 1000, true);
  intake();

  driveTo(30, 2000, true);
  correct_angle = -1.0;
  driveTo(32, 2000, true, 9);
  loader.set(true);
  
  turnToAngle(-30, 1000, true);
  driveTo(16, 1000, false);
  turnToAngle(-90, 1000, true);
  driveTo(-12, 1000, true);
  outtake();
  wait(75, msec);
  score();
  wait(450, msec);
  outtake();
  wait(75, msec);
  score();
  wait(450, msec);

  driveTo(32, 1000, true, 6);
  driveTo(-6, 1000, true);
  turnToAngle(-45, 1000, true);
  driveTo(-60, 1000, true);
  outtake();
  intake1.spin(forward, 1, volt); 
  intake3.spin(reverse, 12, volt);
  wait(100,msec);
  score_mid();
  wait(400, msec);
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
  moveToPoint(-37.65, -56, 1, 2000, false);
  loader.set(true);
  //correct_angle = normalizeTarget(180);
  turnToAngle(normalizeTarget(180), 500, true);
  driveTo(36, 500, false, 7);
  driveTo(36, 550, false, 4);
  correct_angle = normalizeTarget(179);
  driveTo(-36, 1000, false, 9);
  hood.set(false);
  outtake();
  wait(75, msec);
  score();
  wait(250, msec);
  outtake();
  wait(75, msec);
  score();
  wait(1250, msec);
  loader.set(false);
  wing.set(false);
  correct_angle = normalizeTarget(180);
  driveTo(3, 800, false, 6);
  intake();
  turnToAngle(-150, 800, false, 6);
  correct_angle = normalizeTarget(-110);
  driveTo(-5, 1500, false, 6);
  turnToAngle(-177, 500, true);
  driveTo(-20, 2000, true, 8);
  stopChassis(hold);
}

void left43() {
  resetPositionBack();
  resetPositionLeft();
  intake();
  wing.set(true);
  thread la1 = thread(loader_async_l7);
  moveToPoint(-21.75, -27.25, 1, 2000, false, 12);
  loader.set(true);
  driveChassis(6, 6);
  wait(150, msec);
  moveToPoint(-37.65, -56, 1, 2000, false);
  loader.set(true);
  //correct_angle = normalizeTarget(180);
  turnToAngle(normalizeTarget(180), 500, true);
  driveTo(36, 500, false, 7);
  driveTo(36, 550, false, 4);
  correct_angle = normalizeTarget(179);
  driveTo(-36, 1000, false, 9);
  hood.set(false);
  outtake();
  wait(75, msec);
  score();
  wait(250, msec);
  outtake();
  wait(75, msec);
  score();
  wait(600, msec);
  intake();
  loader.set(false);
  wing.set(false);
  turnToAngle(90, 500, true);
  driveTo(12, 1000, true);
  turnToAngle(225, 1000, true);
  driveTo(-24, 1000, true);
  score_mid();
  intake3.spin(reverse, 6, volt);
  wait(1000, msec);
  driveTo(12, 1000, true, 8);
  descore.set(true);
  driveTo(-12, 1000, true, 6);
  stopChassis(hold);
  //wait(5000, msec);
  stopChassis(coast);
  driveTo(36, 2000, true);
  wing.set(false);
  turnToAngle(-180, 1000, true);
  driveTo(-24, 2000, true);
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
  loader.set(true);
  moveToPoint(56.5, -54, 1, 2000, false, 12);
  //correct_angle = normalizeTarget(180);
  turnToAngle(180, 1000, true);
  driveTo(1000, 500, false, 9);
  driveTo(1000, 450, false, 6);
  correct_angle = normalizeTarget(179);
  driveTo(-1000, 1300, false, 11);
  outtake();
  wait(75, msec);
  score();
  wait(700, msec);
  outtake();
  hood.set(false);
  wait(75, msec);
  score();
  wait(750, msec);
  loader.set(false);
  wing.set(false);
  driveTo(3, 800, false, 6);
  turnToAngle(-150, 800, false, 6);
  correct_angle = normalizeTarget(-110);
  driveTo(-4, 1500, false, 6);
  turnToAngle(-182, 500, true);
  driveTo(-22, 2000, true, 12);
  stopChassis(hold);
}

void right4() {
  resetPositionBack();
  resetPositionRight();
  intake();
  wing.set(true);
  thread la1 = thread(loader_async_r7);
  moveToPoint(21.75, -27.25, 1, 2000, false, 12);
  loader.set(true);
  driveChassis(6, 6);
  wait(150, msec);
  moveToPoint(56.5, -54, 1, 2000, false, 12);

  driveTo(-1000, 1000, false, 11);
  outtake();
  wait(75, msec);
  score();
  wait(450, msec);
  outtake();
  hood.set(false);
  wait(75, msec);
  score();
  wait(650, msec);
  loader.set(false);
  wing.set(false);
  driveTo(3, 800, false, 6);
  turnToAngle(-150, 800, false, 6);
  correct_angle = normalizeTarget(-110);
  driveTo(-4, 1500, false, 6);
  turnToAngle(-182, 500, true);
  driveTo(-22, 2000, true, 12);
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

void left9() {
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

  driveTo(3, 800, false, 6);
  turnToAngle(-150, 800, false, 6);
  correct_angle = normalizeTarget(-110);
  driveTo(-4, 1500, false, 4);
  turnToAngle(-177, 500, true);
  driveTo(-23, 2000, true, 8);
  stopChassis(hold);
}

void sixThreeLeft() {
  resetPositionBack();
  resetPositionLeft();
  intake();
  thread la1 = thread(loader_async_63L);
  wing.set(true);

  moveToPoint(-24.5, -22, 1, 2000, false);

  boomerang(-45, -13, 1, -75, 0.2, 2000, true);
  correct_angle = normalizeTarget(-80);
  driveTo(-10, 1000, false);
  correct_angle = normalizeTarget(-170);
  driveTo(9, 1000, true);

  moveToPoint(-47.5, -48, 1, 2000, true);
  turnToAngle(-180, 1000, true);
  intake();
  driveTo(1000, 1000, false, 6);
  correct_angle = normalizeTarget(-179);

  driveTo(-36, 1000, false);
  turnToAngle(-180, 1000, true);
  outtake();
  wait(75, msec);
  score();
  wait(450, msec);
  outtake();
  wait(75, msec);
  score();
  wait(450, msec);

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
  wait(275, msec);
  loader.set(false);
  wait(600, msec);
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
  driveTo(-8, 1000, true);
  turnToAngle(170, 1000, false);
  driveTo(9, 1000, false);
  
  moveToPoint(64, -45, 1, 2000, true);
  turnToAngle(180, 1000, true);
  intake();
  driveTo(1000, 1000, false, 7);
  correct_angle = normalizeTarget(179);

  driveTo(-100, 1000, false);
  turnToAngle(180, 1000, true);
  outtake();
  wait(50, msec);
  score();
  wait(300, msec);
  outtake();
  wait(50, msec);
  score();
  wait(300, msec);

  intake();
  driveTo(4, 1000, true);
  loader.set(false);
  turnToAngle(-90, 1000, true);
  driveTo(17, 1000, true);
  turnToAngle(-44, 1000, true);
  driveTo(33, 2000, true);
  outtake();
  intake1.spin(reverse, 7, volt);
  wait(1500, msec);

  driveTo(-38, 2000, true);
  wing.set(false);
  turnToAngle(0, 1000, true, 8);
  driveTo(30, 2000, true);
  stopChassis(hold);
  turnToAngle(-30, 500, true);
}

void park() {
  inertial_sensor.setHeading(180, degrees);
  curveCircle(260, 29, 2800, true, 12);
  score();
  driveTo(50, 2000, true);
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
  driveTo(53.5, 2000, true);
  turnToAngle(-180, 600, true);
  driveTo(20, 2000, false);
  driveTo(-6, 1000, true);
  turnToAngle(-215, 600, true);
  driveTo(-24, 1000, true);
  turnToAngle(-180, 600, true);
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
  turnToAngle(0, 600, true);
  driveTo(-12, 1000, false);
  hood.set(true);
  loader.set(true);
  outtake();
  wait(75, msec);
  score();
  wait(500, msec);
  outtake();
  wait(75, msec);
  score();
  wait(1000, msec);

  thread ias = thread(intakeAfterScoring);
  correct_angle = 2;
  driveTo(36, 3000, true, 4);
  driveTo(-32, 2000, false, 6);
  hood.set(true);
  ias.interrupt();
  outtake();
  wait(75, msec);
  score();
  wait(500, msec);
  outtake();
  wait(75, msec);
  score();
  wait(1000, msec);
  loader.set(false);

  turnToAngle(95, 1000, true);
  loader.set(false);
  intake1.spin(forward, 12, volt);
  intake2.spin(reverse, 1, volt);
  intake3.spin(reverse, 3, volt);
  driveTo(10, 1000, true, 6);
  loader.set(true);
  wait(200, msec);
  loader.set(false);
  correct_angle = 135;
  driveTo(20, 2000, true);
  outtake();
  intake1.spin(reverse, 6, volt);
  wait(50, msec);
  intake3.spin(reverse, 12, volt);
  wait(1250, msec);

  intake();
  driveTo(-14, 2000, true);
  turnToAngle(90, 600, true);


  driveTo(50, 4000, true);

  turnToAngle(45, 600, true);

  driveTo(24.75, 1200, true);

  turnToAngle(0, 600, true);
  driveTo(-12, 1000, false);
  hood.set(true);
  loader.set(true);
  outtake();
  wait(75, msec);
  score();
  wait(500, msec);
  outtake();
  wait(75, msec);
  score();
  wait(1000, msec);

  driveTo(7, 3000, true, 9);
  hood.set(false);
  driveTo(36, 3000, true, 4);
   driveTo(-7, 1000, true);
  turnToAngle(-45, 600, true);
  driveTo(-24, 1000, true);
  turnToAngle(0, 600, true);
  stop(coast);
    loader.set(false);
  driveTo(-69, 2000, true);

  turnToAngle(90, 650, false);
  driveTo(10, 1000, true);
  stopChassis(hold);
  resetPositionFront();
  resetPositionRight();
  stopChassis(coast);

  driveTo(-15.25, 1000, true);
  turnToAngle(180, 600, true);
  driveTo(-12, 1000, false);
  hood.set(true);
  loader.set(true);
  outtake();
  wait(75, msec);
  score();
  wait(500, msec);
  outtake();
  wait(75, msec);
  score();
  wait(1000, msec);

  driveTo(6, 3000, true, 4);
  hood.set(false);
  
  driveTo(36, 3000, true, 4);
  driveTo(-32, 2000, false, 6);
  hood.set(true);
  ias.interrupt();
  outtake();
  wait(75, msec);
  score();
  wait(500, msec);
  outtake();
  wait(75, msec);
  score();
  wait(1000, msec);
  loader.set(false);

  curveCircle(260, 29, 2800, true, 12);
  score();
  driveTo(50, 2000, true);
}
