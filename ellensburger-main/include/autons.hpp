#include "lib/intake.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/rtos.hpp"
#include "robotconfig.h"



inline void redRingSide() {
  teamColor = team::red;
  chassis.setPose(-56, 14, -90);
  chassis.moveToPoint(-56, 14, 100);

  // score on alliance stake
  chassis.turnToHeading(-140, 1000);

  lift.setTarget(215);
  pros::delay(450);
  
  // clamp goal
  chassis.moveToPoint(-48, 24, 1000, {.forwards = false, .minSpeed = 90, .earlyExitRange = 6});
  chassis.moveToPoint(-26, 19.5, 1000, {.forwards = false, .minSpeed = 60, .earlyExitRange = 4});
  lift.setTarget(75);

  chassis.waitUntilDone();
  clamp.extend();

  // intake quad stack
  chassis.moveToPoint(-13.7, 32, 1200, {.maxSpeed = 90});
  intake.setDirection(lib::Dir::In);

  chassis.turnToPoint(-12.5, 49, 1000, {.maxSpeed = 100});
  chassis.moveToPoint(-12.5, 49, 1500, {.minSpeed = 10, .earlyExitRange = 1});

  chassis.waitUntilDone();

  // back out
  chassis.moveToPoint(-24, 40, 1000, {.maxSpeed = 90, .minSpeed = 30, .earlyExitRange = 6});
  
  // go for corner
  chassis.turnToPoint(-70, 65.5, 1000);
  chassis.moveToPoint(-70, 65.5, 1000, {.maxSpeed = 95});

  intake.setState(lib::Dir::In, lib::Jam::None, true);

  chassis.waitUntilDone();

  chassis.moveToPoint(-51, 51, 800, {.forwards = false});
  chassis.moveToPoint(-69.5, 71, 900);

  chassis.moveToPoint(-48, 48, 2000, {.forwards = false});
  chassis.turnToPoint(-51.6, 20.7, 1000);
  chassis.moveToPoint(-52.6, 22, 1550, {.maxSpeed = 80});
  chassis.waitUntil(28.5);
  
  intake.setState(lib::Dir::In, lib::Jam::Reverse, true);
  leftDoinker.extend();
  pros::delay(280);

  chassis.moveToPoint(-49, 32, 1000, {.forwards = false, .maxSpeed = 90, .minSpeed = 20, .earlyExitRange = 3});
  chassis.turnToPoint(0, -17, 800);
  pros::delay(250);
  leftDoinker.retract();
  lift.setTarget(85);
  chassis.moveToPoint(-6, 6, 1000, {.maxSpeed = 65});
}



inline void blueRingSide() {
  teamColor = team::blue;
  chassis.setPose(56, 14, 90);
  chassis.moveToPoint(56, 14, 100);

  // score on alliance stake
  chassis.turnToHeading(140, 1000);

  lift.setTarget(215);
  pros::delay(450);
  
  // clamp goal
  chassis.moveToPoint(48, 24, 1000, {.forwards = false, .minSpeed = 90, .earlyExitRange = 6});
  chassis.moveToPoint(26, 19.5, 1000, {.forwards = false, .minSpeed = 60, .earlyExitRange = 4});
  lift.setTarget(75);

  chassis.waitUntilDone();
  clamp.extend();

  // intake quad stack
  chassis.moveToPoint(13.7, 32, 1200, {.maxSpeed = 90});
  intake.setDirection(lib::Dir::In);

  chassis.turnToPoint(12.5, 49, 1000, {.maxSpeed = 100});
  chassis.moveToPoint(12.5, 49, 1500, {.minSpeed = 10, .earlyExitRange = 1});

  chassis.waitUntilDone();

  // back out
  chassis.moveToPoint(24, 40, 1000, {.maxSpeed = 90, .minSpeed = 30, .earlyExitRange = 6});
  
  // go for corner
  chassis.turnToPoint(70, 65.5, 1000);
  chassis.moveToPoint(70, 65.5, 1000, {.maxSpeed = 95});

  intake.setState(lib::Dir::In, lib::Jam::None, true);

  chassis.waitUntilDone();

  chassis.moveToPoint(51, 51, 800, {.forwards = false});
  chassis.moveToPoint(69.5, 71, 900);

  chassis.moveToPoint(48, 48, 2000, {.forwards = false});
  chassis.turnToPoint(51.6, 20.7, 1000);
  chassis.moveToPoint(52.6, 22, 1550, {.maxSpeed = 80});
  chassis.waitUntil(28.5);
  
  intake.setState(lib::Dir::In, lib::Jam::Reverse, true);
  rightDoinker.extend();
  pros::delay(280);

  chassis.moveToPoint(49, 32, 1000, {.forwards = false, .maxSpeed = 90, .minSpeed = 20, .earlyExitRange = 3});
  chassis.turnToPoint(0, -17, 800);
  pros::delay(250);
  rightDoinker.retract();
  lift.setTarget(85);
  chassis.moveToPoint(6, 6, 1000, {.maxSpeed = 65});
}



inline void redMogo() {
  teamColor = team::red;
  chassis.setPose(-55.5, -24, -90);
  chassis.setPose(-55.5, -24, -90);

  chassis.turnToPoint(-25, -24, 1000, {.forwards = false});
  chassis.moveToPoint(-25, -24, 1000, {.forwards = false, .maxSpeed = 80});
  chassis.waitUntil(21.25);
  intake.setDirection(lib::Dir::In);
  chassis.waitUntil(21.75);
  clamp.extend();
  chassis.waitUntilDone();

  chassis.turnToPoint(-21, -16, 1000);
  chassis.moveToPoint(-21, -16, 1000);
  pros::delay(180);
  intake.setDirection(lib::Dir::Idle);
  chassis.waitUntilDone();

  rightDoinker.extend();
  pros::delay(200);

  chassis.swingToPoint(0, -6, lemlib::DriveSide::RIGHT, 750);
  chassis.waitUntilDone();
  leftDoinker.extend();
  pros::delay(200);

  chassis.moveToPoint(-39, -38, 1200, {.forwards = false});
  pros::delay(800);
  leftDoinker.retract();
  rightDoinker.retract();
  intake.setDirection(lib::Dir::In);

  chassis.moveToPoint(-22, -28, 1000);
  chassis.moveToPoint(-31, -19, 1000);

  chassis.turnToPoint(-28, -60, 1000);
  intake.setDirection(lib::Dir::Idle);
  chassis.moveToPoint(-28, -60, 1000);
  pros::delay(100);
  intake.setDirection(lib::Dir::In);

  chassis.turnToPoint(-58, -62, 1000);
  chassis.moveToPoint(-58, -62, 1000, {.minSpeed = 20, .earlyExitRange = 4});
  chassis.moveToPoint(-73, -70.5, 1300, {.maxSpeed = 90});
  lift.setTarget(100);
  intake.setState(lib::Dir::In, lib::Jam::None, true);

  chassis.moveToPoint(-53, -50, 1000, {.forwards = false});
  chassis.moveToPoint(-73, -68, 1000);
  chassis.moveToPoint(-22, -22, 2000);
  lift.setTarget(70);
}



inline void blueMogo() {
  teamColor = team::blue;
  chassis.setPose(55.5, -24, 90);
  chassis.setPose(55.5, -24, 90);

  chassis.turnToPoint(25, -24, 1000, {.forwards = false});
  chassis.moveToPoint(25, -24, 1000, {.forwards = false, .maxSpeed = 70});
  chassis.waitUntil(21);
  intake.setDirection(lib::Dir::In);
  chassis.waitUntil(21.75);
  clamp.extend();
  chassis.waitUntilDone();

  chassis.turnToPoint(20.75, -16.25, 1000);
  chassis.moveToPoint(20.75, -16.25, 1000, {.maxSpeed = 100});
  pros::delay(180);
  intake.setDirection(lib::Dir::Idle);
  chassis.waitUntilDone();

  leftDoinker.extend();
  pros::delay(200);

  chassis.swingToPoint(0, -5.7, lemlib::DriveSide::LEFT, 750);
  chassis.waitUntilDone();
  rightDoinker.extend();
  pros::delay(200);

  chassis.moveToPoint(39, -38, 1200, {.forwards = false, .maxSpeed = 100});
  pros::delay(800);
  leftDoinker.retract();
  rightDoinker.retract();
  intake.setDirection(lib::Dir::In);

  chassis.moveToPoint(22, -28, 1000);
  chassis.moveToPoint(30, -18, 1000, {.maxSpeed = 100});

  chassis.turnToPoint(30, -48, 1000);
  intake.setDirection(lib::Dir::Idle);
  chassis.moveToPoint(30, -48, 1000);
  pros::delay(100);
  intake.setDirection(lib::Dir::In);

  chassis.turnToPoint(48, -48, 1000);
  chassis.moveToPoint(50, -48, 1000, {.minSpeed = 20, .earlyExitRange = 10});
  chassis.moveToPoint(73, -70.5, 1300, {.maxSpeed = 90});
  lift.setTarget(100);

  chassis.moveToPoint(55, -52, 1000, {.forwards = false});
  chassis.moveToPoint(73, -68, 1000);
  chassis.moveToPoint(22, -22, 2000);
  lift.setTarget(70);
}



inline void redMogoStake() {
  teamColor = team::red;
  chassis.setPose(-56, -14, -90);

  chassis.turnToHeading(-40, 1000);
  lift.setTarget(210);
  while (armrot.get_angle() < 19500) { pros::delay(10); }

  // clamp mogo
  chassis.cancelMotion();
  chassis.moveToPoint(-24.25, -25, 2000, {.forwards = false, .maxSpeed = 80, .minSpeed = 10, .earlyExitRange = 2});
  chassis.waitUntilDone();
  clamp.extend();
  lift.setTarget(0);

  // doinker alliance stake ring
  chassis.moveToPoint(-36, -18.667, 1000);
  chassis.waitUntilDone();
  rightDoinker.extend();

  // pull ring back
  pros::delay(300);
  chassis.moveToPoint(-24, -30, 1000, {.forwards = false, .maxSpeed = 90, .minSpeed = 30, .earlyExitRange = 9});
  chassis.moveToPoint(-35, -42, 1500, {.forwards = false});

  chassis.waitUntilDone();
  rightDoinker.retract();
  pros::delay(100);

  // eat ring and move to center stacks
  chassis.moveToPoint(-18, -18.5, 1500);
  intake.setDirection(lib::Dir::In);
  pros::delay(700);
  intake.setDirection(lib::Dir::Idle);
  chassis.waitUntilDone();

  rightDoinker.extend(); // doink
  pros::delay(300);

  // back out
  chassis.moveToPoint(-50, -34, 1000, {.forwards = false});
  chassis.waitUntilDone();
  rightDoinker.retract();
  pros::delay(200);

  // eat ring
  chassis.moveToPoint(-27.5, -32, 1000, {.maxSpeed = 90});
  intake.setDirection(lib::Dir::In);

  // move to stack
  chassis.turnToPoint(-27.5, -53, 1000, {.maxSpeed = 90});
  chassis.moveToPoint(-27.5, -53, 2000);
  lift.setTarget(100);
  
  chassis.turnToPoint(-48, -48, 1000);
  chassis.moveToPoint(-48, -48, 1000, {.minSpeed = 20, .earlyExitRange = 2});
  chassis.moveToPoint(-70, -70, 1000);
  chassis.moveToPoint(-60, -60, 500, {.forwards = false});

  //chassis.turnToPoint(0, 0, 1000);
  chassis.moveToPoint(-8, -8, 1500);
  lift.setTarget(65);
}



inline void blueMogoStake() {
  teamColor = team::blue;
  chassis.setPose(56, -14, 90);

  chassis.turnToHeading(40, 1000);
  lift.setTarget(210);
  while (armrot.get_angle() < 19500) { pros::delay(10); }

  // clamp mogo
  chassis.cancelMotion();
  chassis.moveToPoint(24.25, -25, 2000, {.forwards = false, .maxSpeed = 80, .minSpeed = 10, .earlyExitRange = 2});
  chassis.waitUntilDone();
  clamp.extend();
  lift.setTarget(0);

  // doinker alliance stake ring
  chassis.moveToPoint(36, -18.667, 1000);
  chassis.waitUntilDone();
  leftDoinker.extend();

  // pull ring back
  pros::delay(300);
  chassis.moveToPoint(24, -30, 1000, {.forwards = false, .maxSpeed = 90, .minSpeed = 30, .earlyExitRange = 9});
  chassis.moveToPoint(35, -42, 1500, {.forwards = false});

  chassis.waitUntilDone();
  leftDoinker.retract();
  pros::delay(100);

  // eat ring and move to center stacks
  chassis.moveToPoint(18, -18.5, 1500);
  intake.setDirection(lib::Dir::In);
  pros::delay(700);
  intake.setDirection(lib::Dir::Idle);
  chassis.waitUntilDone();

  leftDoinker.extend(); // doink
  pros::delay(300);

  // back out
  chassis.moveToPoint(50, -34, 1000, {.forwards = false});
  chassis.waitUntilDone();
  leftDoinker.retract();
  pros::delay(200);

  // eat ring
  chassis.moveToPoint(27.5, -32, 1000, {.maxSpeed = 90});
  intake.setDirection(lib::Dir::In);

  // move to stack
  chassis.turnToPoint(27.5, -53, 1000, {.maxSpeed = 90});
  chassis.moveToPoint(27.5, -53, 2000);
  lift.setTarget(100);
  
  chassis.turnToPoint(48, -48, 1000);
  chassis.moveToPoint(48, -48, 1000, {.minSpeed = 20, .earlyExitRange = 2});
  chassis.moveToPoint(70, -70, 1000);
  chassis.moveToPoint(60, -60, 500, {.forwards = false});

  //chassis.turnToPoint(0, 0, 1000);
  chassis.moveToPoint(8, -8, 1500);
  lift.setTarget(65);
}