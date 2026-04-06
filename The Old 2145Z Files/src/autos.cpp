#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "screen.hpp"
#include "subsystems/chassis.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/misc.hpp"

bool check_equal(float val1, float val2, int residual)
{
	if (fabs(val2-val1) < residual) // if the difference is small enough
	{
		return true;
	}
	else
	{
		return false;
	}
}

void doNothing() {
    set_position(0, 0, 0);
    set_drive(4.0, 10000.0, {.minSpeed = 100});
}

void test1() {
    set_position(0, 0, 0);
    set_turn(90.0, 3000);
    set_turn(0.0, 3000);
}

void sawp() {
    // start
    set_position(-50, -20, 0);
    resetLeft();
    resetBack();
    intake();
    //set_drive(4.0, 500);
    // resetLeft();
    // resetBack();
    // get matchloader and score
    set_point(-50.0, -47.5, 1400, {.forwards = false});
    resetBack();
    resetLeft();
    setLoader(true);

    set_turn(270.0, 750, {.earlyExitRange = 10.0});
    set_point(-66.0, chassis.getPose().y, 850);
    set_point(-28.0, chassis.getPose().y-0.0, 800, {.forwards = false, .minSpeed = 127});
    set_raw(-64.0);
    outtake();
    wait(150);
    score();
    wait(1200);
    setLoader(false);

    // turn & intake 3 stacks
    set_turn(0.0, 800, {.earlyExitRange = 5.0});
    resetLeft();
    resetBack();
    intake();
    set_point(-23.0, -24.0, 3000, {.minSpeed = 127, .earlyExitRange = 6.0});
    set_point(-23.5, 22.0, 3000, {.minSpeed = 80}, false);
    wait(750);
    setLoader(true);
    wait();
    set_turn(315.0, 800, {.minSpeed = 80, .earlyExitRange = 5.0});

    //score 2nd long goal
    set_point(-48.0, 43.5, 2000);
    set_turn(270.0, 550, {.earlyExitRange = 0.5});
    //resetFront();
    set_point(-28.0, 43.70, 600, {.forwards = false, .minSpeed = 127});
    set_raw(-64.0);
    outtake();
    wait(150);
    score();
    wait(1000);
    
    // load and score middle goal
    intake();
    set_point(-64.0, chassis.getPose().y-1, 1750, {.maxSpeed = 55, .minSpeed = 30});
    set_point(-3.0, 1.0, 1700, {.forwards = false,.maxSpeed = 100, .minSpeed = 40}, false);
    wait(1250);
    setLoader(false);
    midSlow = false;
    scoreMiddle();
    wait();
    scoreMiddle();
}

void left43midMID() {
   // start
   set_position(-48, 18, 90.0);
   resetLeft();
   resetBack();
   intake();

   // intake 3 stack 
   set_point(-25, 22, 1000, {.minSpeed = 40}, false);
   wait_until(10.0);
   setLoader(true);
   wait();

   // score long goal
   set_point(-26, 48.0, 2000, {.forwards = false});

}

void left43mid() {
    // start
    set_position(-48, 18, 90.0);
    resetLeft();
    resetBack();
    intake();

    // intake 3 stack 
    set_point(-23, 22, 1000, {.minSpeed = 40}, false);
    wait(500);
    setLoader(true);
    wait();

    // get matchloader
    set_turn(315.0, 1000, {.minSpeed = 40, .earlyExitRange = 20.0});
    set_point(-54, 46.0, 3000);
    set_turn(270.0, 1000, {.minSpeed = 20});
    set_point(-64.0, 46.0, 3000);


    set_point(-24, chassis.getPose().y, 1400, {.forwards = false, .maxSpeed = 60});
    set_raw(-127);
    outtake();
    wait(200);
    score();
    wait(700);
    outtake();
    wait(150);
    stop();
    // setLoader(false);
    // set_drive(6.0, 1000, {.minSpeed = 50, .earlyExitRange = 1.0});
    // set_turn(0.0, 1000);

    set_turn(180, 2000);
    set_drive(10, 2000, {.minSpeed = 40});
    set_turn(315.0, 2000);
    setLoader(false);
    set_drive(-28, 1000, {.forwards = false, .minSpeed = 60});
    scoreMiddle();
    // intake2.move_velocity(300);
    wait(1500);
    set_drive(8.0, 1000, {.minSpeed = 80});
    setDescore(true);
    set_drive(-9.0, 1000, {.forwards = false, .maxSpeed = 127});
}

void left43long() {

}

void left7() {
    // start
    set_position(-48, 18, 90.0);
    resetLeft();
    resetBack();
    intake();

    // intake 3 stack 
    set_point(-23, 22, 1000, {.minSpeed = 30}, false);
    wait(500);
    setLoader(true);
    wait();

    // get matchloader
    set_turn(315.0, 1000, {.minSpeed = 20, .earlyExitRange = 20.0});
    set_point(-54, 48.5, 3000);
    set_turn(270.0, 1000, {.minSpeed = 20});
    // resetRight();
    // resetFront();
    set_drive(22.0, 1250, {.maxSpeed = 40});

    set_point(-24, chassis.getPose().y + 0.5, 1000, {.forwards = false, .minSpeed = 127});
    set_raw(-127);
    outtake();
    wait(200);
    score();
    wait(1750);

    set_position(0, 0, chassis.getPose().theta);
    set_point(-6, 0, 1000, {.minSpeed = 75});
    set_turn(330, 1000, {.minSpeed = 75});
    set_drive(-5.0, 1000, {.forwards = false, .minSpeed = 75});
    set_pose(24, -12, 270, 2000, {.forwards = false, .lead = 0.5, .minSpeed = 75});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

}

void right43() {
    // start
    set_position(-48, -18, 90.0);
    resetRight();
    resetBack();
    intake();

    // intake 3 stack 
    set_point(-23, -22, 1000, {.minSpeed = 30}, false);
    wait(500);
    setLoader(true);
    wait();

    // get matchloader
    set_turn(225.0, 1000, {.minSpeed = 20, .earlyExitRange = 20.0});
    set_point(-54, -47, 3000);
    set_turn(270.0, 1000, {.minSpeed = 20});
    resetRight();
    resetFront();
    set_drive(14.0, 2150);

    set_point(-24, -76.0, 1000, {.forwards = false, .minSpeed = 40});
    outtake();
    wait(150);
    score();
    wait(650);

    setLoader(false);
    intake();
    set_drive(4.0, 1000, {.minSpeed = 50, .earlyExitRange = 2.0});
    set_turn(180.0, 1000);

    set_pose(0, 0, 225, 2000, {.forwards = false});
    scoreMiddle();
    intake2.move_velocity(100);
    wait(1000);
}

void right7() {

}

void driveUntilDistanceFront(float distance, int speed, int timeout) {
    double time = pros::millis();
    set_raw(speed);
    while(pros::millis() - time < timeout) {
        if(distanceFront.get_distance() /25.4 <= distance) {
            break;
        }
    }
}

void skills() {

    float loadSpeed = 65.0;
    // start
    set_position(-48, 18, 90.0);
    resetLeft();
    resetBack();
    intake();
    setWingBack(true);

    // intake 3 stack and score middle
    set_point(-23, 22, 1000, {.minSpeed = 30}, false);
    wait(500);
    setLoader(true);
    wait();
    set_pose(-12, 7, 315, 2000, {.forwards = false});
    scoreMiddle();
    intake2.move_velocity(200);
    wait(1500);

    // get matchloader
    intake();
    set_point(-54, 47, 3000);
    set_turn(270.0, 1000, {.minSpeed = 20});
    resetRight();
    resetFront();
    set_drive(12.0, 2500, {.maxSpeed = loadSpeed});
    resetFront();
    resetRight();
    wait(1000);

    // drive through alley and score
    set_drive(-4.0, 2000, {.forwards = false, .minSpeed = 40, .earlyExitRange = 2.0});
    set_pose(-48, 83, 270, 3000, {.forwards = false, .lead = 0.3, .minSpeed = 40});
    set_turn(270.0, 1000);
    set_drive(-72.0, 2000, {.forwards = false, .minSpeed = 40}, false);
    wait_until(50.0);
    setLoader(false);
    wait();
    set_turn(0.0, 1000);
    resetFront();
    resetRight();
    set_drive(-12.0, 1000, {.forwards = false});
    set_turn(90.0, 1000);
    set_point(24, 47, 1000, {.forwards = false});
    outtake();
    set_raw(-120);
    wait(200);
    score();
    setLoader(true);
    wait(2000);
    // matchload and score long goal again
    set_point(62.0, 46, 3000, {.maxSpeed = loadSpeed}, false);
    wait_until(6.0);
    intake();
    wait();
    set_point(24, chassis.getPose().y+1.0, 1500, {.forwards = false});
    outtake();
    set_raw(-127);
    wait(200);
    score();
    wait(2000);
    setLoader(false);

    // get 4 stack and score in low goal
    set_turn(180, 1000, {.earlyExitRange = 5.0});
    intake();
    intake2.move(16);
    resetLeft();
    resetBack();
    set_point(26, 24, 2000, {}, false);
    wait_until(1.0);
    setLoader(true);
    wait();
    set_pose(9.5, 6.0, 225.0, 1750, {}, false);
    wait_until(2.0);
    setLoader(false);
    wait();
    outtake();
    intake1.move_velocity(-175);
    wait(2000);

    // get next 4 stack
    set_drive(-12.0, 1000, {.forwards = false, .minSpeed = 75, .earlyExitRange = 2.0});
    intake();
    set_point(23.5, -22.0, 3000, {.minSpeed = 80});
    set_turn(135.0, 800, {.minSpeed = 80, .earlyExitRange = 5.0});

    //get 3rd matchloader
    set_point(50.0, -46.0, 2000);
    set_position(50.0, -46.0, chassis.getPose().theta);
    intake();
    setLoader(true);
    set_turn(90.0, 550, {.earlyExitRange = 0.5});
    resetFront();
    resetRight();
    set_point(64.0, -47.0, 2000, {.maxSpeed = loadSpeed});

    // go through alley
    set_drive(-4.0, 2000, {.forwards = false, .minSpeed = 40, .earlyExitRange = 2.0});
    set_pose(32.0, -60.0, 90, 3000, {.forwards = false, .lead = 0.3, .minSpeed = 40});
    set_turn(90.0, 1000);
    set_drive(-72.0, 2000, {.forwards = false, .minSpeed = 40}, false);
    wait_until(-60.0);
    setLoader(false);
    wait();
    set_turn(180.0, 1000);
    resetFront();
    resetRight();
    set_drive(-11.0, 1000, {.forwards = false});
    set_turn(270.0, 1000);
    set_point(-24, -47.5, 1000, {.forwards = false});
    outtake();
    set_raw(-127);
    wait(200);
    score();
    setLoader(true);
    wait(1500);

    // matchload and score long goal again
    set_point(-63.0, -46.5, 3000, {.maxSpeed = loadSpeed}, false);
    wait_until(6.0);
    intake();
    wait();
    set_point(-24, chassis.getPose().y-1.0, 1750, {.forwards = false});
    outtake();
    set_raw(-127);
    wait(200);
    score();
    wait(2000);
    setLoader(false);

    // park
    resetFront();
    resetLeft();
    score();
    set_turn(300.0, 1000);
    set_drive(34.0, 1000, {.minSpeed = 10});
    set_turn(350.0, 1000, {.minSpeed = 10});
    set_drive(14.0, 1000, {.minSpeed = 10});
    setLoader(true);
    wait(200);
    setLoader(false);
    set_drive(32.0, 1000, {.minSpeed = 110});
}

void skills2() {
    set_position(50.0, -46.0, 90.0);
    intake();
    setLoader(true);
    wait(500);
    resetFront();
    resetRight();
    set_point(64.0, -46.5, 1500, {});

    // go through alley
    set_drive(-4.0, 2000, {.forwards = false, .minSpeed = 40, .earlyExitRange = 2.0});
    set_pose(32.0, -60.0, 90, 3000, {.forwards = false, .lead = 0.3, .minSpeed = 40});
    set_turn(90.0, 1000);
    set_drive(-72.0, 2000, {.forwards = false, .minSpeed = 40}, false);
    wait_until(-60.0);
    setLoader(false);
    wait();
    set_turn(180.0, 1000);
    resetFront();
    resetRight();
    set_drive(-12.0, 1000, {.forwards = false});
    set_turn(270.0, 1000);
    set_point(-24, -47.5, 1000, {.forwards = false});
    outtake();
    set_raw(-127);
    wait(200);
    score();
    setLoader(true);
    wait(2000);

    // matchload and score long goal again
    set_point(-62.0, -46.5, 2750, {.maxSpeed = 75}, false);
    wait_until(6.0);
    intake();
    wait();
    set_point(-24, chassis.getPose().y-1.0, 1750, {.forwards = false});
    outtake();
    set_raw(-127);
    wait(200);
    score();
    wait(1750);
    setLoader(false);

}

void skills3() {
    set_position(-24, -47, 270.0);
    resetFront();
    resetLeft();
    score();
    set_turn(300.0, 1000);
    set_drive(34.0, 1000, {.minSpeed = 10});
    set_turn(350.0, 1000);
    set_drive(10.0, 1000, {.minSpeed = 10});
    setLoader(true);
    wait(200);
    setLoader(false);
    set_drive(28.0, 1000, {.minSpeed = 100});
}