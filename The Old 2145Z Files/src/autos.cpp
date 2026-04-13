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

    set_turn(270.0, 750);
    set_point(-66.0, chassis.getPose().y, 850);
    set_point(-28.0, chassis.getPose().y-0.0, 800, {.forwards = false, .minSpeed = 127});
    set_raw(-64.0);
    outtake();
    wait(200);
    score();
    wait(1000);
    setLoader(false);

    // turn & intake 3 stacks
    set_turn(0.0, 800);
    wait(50);
    resetLeft();
    resetBack();
    intake();
    set_point(-23.0, -24.0, 3000);
    set_point(-23.5, 22.0, 3000, {}, false);
    wait(750);
    setLoader(true);
    wait();
    set_turn(315.0, 800, {.minSpeed = 80});

    //score 2nd long goal
    set_point(-48.0, 43.5, 2000);
    set_turn(270.0, 550);
    //resetFront();
    set_point(-28.0, 43.70, 600, {.forwards = false, .minSpeed = 127});
    set_raw(-64.0);
    outtake();
    wait(200);
    score();
    wait(800);
    
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

void left4() {

}

void left7() {
    // start
    set_position(-48, 12, 90.0);
    resetLeft();
    resetBack();
    intake();

    pros::Task driveAtEnd([] {
        if (curMatchState == AUTO) {
            pros::delay(14750);
            set_drive(2.0, 1000, {.minSpeed = 90});
        }
    });

    // intake 3 stack 
    //set_point(-28, 19, 1000, {.minSpeed = 110, .earlyExitRange = 1.0}, false);
    set_point(-24, 23, 1000, {.minSpeed = 90}, false);
    wait_until(9.0);
    setLoader(true);
    wait();

    // get matchloader
    set_turn(300.0, 600);
    setWingFront(true);
    setWingBack(true);
    set_drive(35.0, 1500, {.minSpeed = 110});
    set_turn(270.0, 300);
    wait(50);
    resetFront();
    resetRight();
    set_point(-66.0, 45.5, 1100, {.maxSpeed = 60});
    wait(50);
    resetRight();

    // score long goal
    set_point(-30, 47.0, 1000, {.forwards = false, .minSpeed = 127}, false);
    wait(400);
    outtake();
    wait(200);
    score();
    wait();
    set_raw(-127);
    wait(150);
    set_raw(0);
    wait(50);
    resetFront();
    resetRight();
    wait(1100);
    intake();

    // wing
    setWingFront(false);
    setWingBack(false);
    set_turn(220.0, 500, {.minSpeed = 127});
    set_swing(270.0, lemlib::DriveSide::RIGHT, 500);
    wait(50);
    resetFront();
    resetRight();
    set_point(-11, 36, 1000, {.forwards = false});
}

void left43mid() {
    // start
    set_position(-48, 12, 90.0);
    resetLeft();
    resetBack();
    intake();

    pros::Task driveAtEnd([] {
        if (curMatchState == AUTO) {
            pros::delay(14850);
            set_drive(1.0, 200, {.minSpeed = 127});
        }
    });

    // intake 3 stack 
    //set_point(-28, 19, 1000, {.minSpeed = 110, .earlyExitRange = 1.0}, false);
    set_point(-24, 22, 900, {.minSpeed = 90}, false);
    wait_until(9.0);
    setLoader(true);
    wait();

    // get matchloader
    set_turn(315.0, 600);
    setWingFront(true);
    setWingBack(true);
    set_drive(29.0, 1500, {.minSpeed = 90});
    set_turn(270.0, 300);
    wait(50);
    resetFront();
    resetRight();

    // score long goal
    set_point(-30, 47.0, 1000, {.forwards = false, .minSpeed = 127});
    score();
    set_raw(-127);
    wait(150);
    set_raw(0);
    wait(50);
    resetFront();
    resetRight();
    wait(600);
    intake();

    //matchload and score middle
    set_point(-66.0, 45.5, 1500, {.maxSpeed = 60});
    wait(50);
    resetFront();
    resetRight();
    set_pose(-12, 11, 315.0, 2000, {.forwards = false, .minSpeed = 90});
    scoreMiddle();
    wait(1000);
    setLoader(false);
    setDescore(true);
    set_drive(8.0, 1000, {.minSpeed = 127});
    set_drive(-10.0, 1000, {.forwards = false, .minSpeed = 127});
}

void left43long() {

    int waittime = 0;
        // start
        set_position(-48, 12, 90.0);
        resetLeft();
        resetBack();
        intake();
    
        pros::Task driveAtEnd([] {
            if (curMatchState == AUTO) {
                pros::delay(14850);
                set_drive(1.0, 200, {.minSpeed = 127});
            }
        });
    
        // intake 3 stack 
        //set_point(-28, 19, 1000, {.minSpeed = 110, .earlyExitRange = 1.0}, false);
        set_point(-24, 22, 900, {.minSpeed = 90}, false);
        wait_until(9.0);
        setLoader(true);
        wait();
    
        // get matchloader
        set_turn(315.0, 600);
        setWingFront(true);
        setWingBack(true);
        set_drive(29.0, 1500, {.minSpeed = 90});
        set_turn(270.0, 300);
        wait(50);
        resetFront();
        resetRight();
    
        // score long goal
        set_point(-30, 47.0, 1000, {.forwards = false, .minSpeed = 127});
        score();
        set_raw(-127);
        wait(150);
        set_raw(0);
        wait(50);
        resetFront();
        resetRight();
        wait(600);
        intake();
    
        //matchload and score middle
        set_point(-66.0, 45.5, 1500, {.maxSpeed = 60});
        wait(50);
        resetFront();
        resetRight();
        set_pose(-12, 11, 315.0, 2000, {.forwards = false, .minSpeed = 110});
        scoreMiddle();
        wait(1000);
        setLoader(false);
        setDescore(true);
        set_drive(8.0, 1000, {.minSpeed = 127});
        set_drive(-2.0, 500, {.forwards = false, .minSpeed = 127});
        wait(waittime);
        set_drive(-7.0, 1000, {.forwards = false, .minSpeed = 127});

        //wing long goal
        set_drive(10.0, 200, {.minSpeed = 127});
        set_pose(-36, 34, 270, 2000, {.forwards = true});
        setWingBack(false);
        setWingFront(false);
        wait(50);
        resetFront();
        resetRight();
        set_pose(-11, 36, 270, 2000, {.forwards = false, .minSpeed = 127});
}

void left63() {

}

void right7() {
    // start
    set_position(-48, -12, 90.0);
    resetRight();
    resetBack();
    intake();

    pros::Task driveAtEnd([] {
        if (curMatchState == AUTO) {
            pros::delay(14750);
            set_drive(-2.0, 1000, {.forwards = false, .minSpeed = 90});
        }
    });

    // intake 3 stack 
    //set_point(-28, 19, 1000, {.minSpeed = 110, .earlyExitRange = 1.0}, false);
    set_point(-24, -23, 1000, {.minSpeed = 90}, false);
    wait_until(9.0);
    setLoader(true);
    wait();

    // get matchloader
    set_turn(235, 600);
    setWingFront(true);
    setWingBack(true);
    set_drive(36.0, 2000, {.minSpeed = 60});
    set_turn(270.0, 300);
    wait(50);
    resetFront();
    resetLeft();
    set_point(-66.0, -45.5, 1100, {.maxSpeed = 60});

    // score long goal
    set_point(-30, -47.0, 1000, {.forwards = false, .minSpeed = 127}, false);
    wait(400);
    outtake();
    wait(200);
    score();
    wait();
    set_raw(-127);
    wait(150);
    set_raw(0);
    wait(50);
    resetFront();
    resetLeft();
    wait(1100);
    intake();

    // wing
    setWingFront(false);
    setWingBack(false);
    set_turn(220.0, 500, {.minSpeed = 127});
    set_swing(270.0, lemlib::DriveSide::RIGHT, 500);
    wait(50);
    resetFront();
    resetLeft();
    set_point(-11, -60, 1000, {.forwards = false});
}

void right43mid() {
    
}

void right43long() {
    
}

void doubleMid() {
    set_position(-48, 12, 90);
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