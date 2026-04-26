#include "autos.hpp"
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
    set_position(-49, -20, 0);
    resetLeft();
    resetBack();
    intake();
    //set_drive(4.0, 500);
    // resetLeft();
    // resetBack();
    // get matchloader and score
    set_point(chassis.getPose().x, 4.5, 200, {.minSpeed = 40});
    set_point(chassis.getPose().x, -46.6, 350, {.forwards = false, .minSpeed = 127});
    set_point(chassis.getPose().x, -46.6, 1250, {.forwards = false});
    resetBack();
    resetLeft();
    setLoader(true);

    set_turn(-66.0, -47.0, 650);
    resetLeft();
    set_point(-68.0, -47, 1000, {.maxSpeed = 55, .minSpeed = 50});
    resetLeft();
    resetFront();
    set_point(-28.0, -48.0, 800, {.forwards = false, .minSpeed = 127});
    score();
    set_raw(-127.0);
    wait(1150);
    setLoader(false);
    intake();

    // turn & intake 3 stacks
    set_turn(0.0, 1000, {.minSpeed = 70});
    resetLeft();
    resetBack();
    set_point(-23.5, -24.0, 2000, {.minSpeed = 90});
    set_point(-23.5, 23.0, 2000, {.minSpeed = 60}, false);
    wait_until(33.0);
    setLoader(true);
    wait();

    //score 2nd long goal
    set_point(-45.0, 47.0, 500, {.minSpeed = 127});
    set_point(-45.0, 47.0, 500);
    set_turn(270.0, 700);
    resetFront();
    resetRight();
    // score long goal
    set_point(-30, 46.5, 800, {.forwards = false, .minSpeed = 127});
    score();
    set_raw(-127);
    wait(200);
    set_raw(0);
    wait(50);
    resetFront();
    resetRight();
    wait(800);
    intake();

    //matchload and score middle
    set_point(-66.0, 45.5, 450, {.minSpeed = 127});
    set_point(-66.0, 45.5, 1000, {.maxSpeed = 55, .minSpeed = 50});
    resetFront();
    resetRight();
    set_pose(-9.5, 7.5, 315.0, 1100, {.forwards = false, .minSpeed = 127});
    outtake();
    set_pose(-9.5, 7.5, 315.0, 100, {.forwards = false});
    scoreMiddle();
    wait(100);
    intake2.move(76);
    set_turn(0,0, 500, {.forwards = false});
}

void left4() {
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
    set_point(-24, 22, 400, {.minSpeed = 127});
    set_point(-24, 22, 600, {}, false);
    wait_until(9.0);
    setLoader(true);
    wait();

    // get matchloader
    set_turn(320.0, 600);
    setWingFront(true);
    setWingBack(true);
    set_drive(22.0, 400, {.minSpeed = 127});
    set_drive(22.5, 600);
    set_turn(270.0, 350);
    resetFront();
    resetRight();

    // score long goal
    set_point(-30, 47.0, 1000, {.forwards = false, .minSpeed = 127});
    score();
    set_raw(-127);
    wait(150);
    resetFront();
    resetRight();
    wait(600);
    intake();

    // wing
    setWingFront(false);
    setWingBack(false);
    set_turn(220.0, 500, {.minSpeed = 127});
    set_swing(270.0, lemlib::DriveSide::RIGHT, 500, {.minSpeed = 127});
    wait(50);
    resetFront();
    resetRight();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    set_point(-12.5, 36.5, 1000, {.forwards = false, .minSpeed = 80});
}

void left7() {
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
    set_point(-24, 22, 400, {.minSpeed = 127});
    set_point(-24, 22, 600, {}, false);
    wait_until(9.0);
    setLoader(true);
    wait();

    // get matchloader
    set_turn(315.0, 600);
    setWingFront(true);
    setWingBack(true);
    set_drive(26.0, 400, {.minSpeed = 127});
    set_drive(26.5, 500);
    set_turn(270.0, 350);
    resetFront();
    resetRight();

    set_point(-66.0, 45.5, 1100, {.maxSpeed = 60, .minSpeed = 55});

    // score long goal
    set_point(-30, 47.0, 1000, {.forwards = false, .minSpeed = 127}, false);
    wait(400);
    outtake();
    wait(200);
    score();
    wait();
    set_raw(-127);
    wait(150);
    resetFront();
    resetRight();
    wait(1250);
    intake();

    // wing
    setWingFront(false);
    setWingBack(false);
    set_turn(217.5, 500, {.minSpeed = 127});
    set_swing(270.0, lemlib::DriveSide::RIGHT, 500, {.minSpeed = 127});
    wait(50);
    resetFront();
    resetRight();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    set_point(-13.5, 36.5, 1000, {.forwards = false, .minSpeed = 80});
}

void left43mid() {
    int waittime = 875;
    // start
    set_position(-48, 12, 90.0);
    resetLeft();
    resetBack();
    intake();

    pros::Task driveAtEnd([] {
        if (curMatchState == AUTO) {
            pros::delay(14800);
            set_drive(-1.0, 150, {.forwards = false,.minSpeed = 127});
            set_drive(1.0, 100, {.minSpeed = 127});
        }
    });

    // intake 3 stack 
    //set_point(-28, 19, 1000, {.minSpeed = 110, .earlyExitRange = 1.0}, false);
    set_point(-24, 22, 400, {.minSpeed = 127});
    set_point(-24, 22, 600, {}, false);
    wait_until(9.0);
    setLoader(true);
    wait();

    // get matchloader
    set_turn(320.0, 600);
    setWingFront(true);
    setWingBack(true);
    set_drive(22.0, 400, {.minSpeed = 127});
    set_drive(22.5, 600);
    set_turn(270.0, 400);
    resetFront();
    resetRight();

    // score long goal
    set_point(-30, 47.0, 1000, {.forwards = false, .minSpeed = 127});
    score();
    set_raw(-127);
    wait(150);
    resetFront();
    resetRight();
    wait(700);
    intake();

    //matchload and score middle
    set_point(-68.0, 45.5, 400, {.minSpeed = 127});
    set_point(-68.0, 45.5, 950, {.maxSpeed = 55, .minSpeed = 50});
    resetFront();
    resetRight();
    set_pose(-7.5, 6.5, 315.0, 1100, {.forwards = false, .minSpeed = 127});
    set_pose(-7.5, 6.5, 315.0, 400, {.forwards = false});
    scoreMiddle();
    wait(100);
    intake2.move(69);
    set_turn(0,0, 500, {.forwards = false});
    wait(waittime);
    setLoader(false);
    //setDescore(true);
    intake();
    set_drive(8.0, 1000, {.minSpeed = 127});
    setDescore(true);
    set_drive(-11.0, 1000, {.forwards = false, .minSpeed = 60});
}

void left43long() {
    int waittime = 2000;
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
    set_point(-24, 22, 400, {.minSpeed = 127});
    set_point(-24, 22, 600, {}, false);
    wait_until(9.0);
    setLoader(true);
    wait();

    // get matchloader
    set_turn(320.0, 600);
    setWingFront(true);
    setWingBack(true);
    set_drive(23.0, 400, {.minSpeed = 127});
    set_drive(23.0, 600);
    set_turn(270.0, 400);
    resetFront();
    resetRight();

    // score long goal
    set_point(-30, 47.0, 1000, {.forwards = false, .minSpeed = 127});
    score();
    set_raw(-127);
    wait(150);
    resetFront();
    resetRight();
    wait(700);
    intake();

    //matchload and score middle
    set_point(-68.0, 45.5, 400, {.minSpeed = 127});
    set_point(-68.0, 45.5, 975, {.maxSpeed = 55, .minSpeed = 50});
    resetFront();
    resetRight();
    set_pose(-7.5, 6.5, 315.0, 1100, {.forwards = false, .minSpeed = 127});
    set_pose(-7.5, 6.5, 315.0, 400, {.forwards = false});
    scoreMiddle();
    wait(100);
    intake2.move(69);
    set_turn(0,0, 500, {.forwards = false});
    wait(waittime);
    setLoader(false);
    //setDescore(true);
    intake();
    set_drive(8.0, 1000, {.minSpeed = 127});
    //set_drive(20.0, 2000, {.minSpeed = 127});
    setDescore(true);
    set_drive(-11.0, 1000, {.forwards = false, .minSpeed = 60});

    set_drive(23.5, 1500, {.minSpeed = 127});
    set_swing(270, lemlib::DriveSide::LEFT, 500);
    resetFront();
    resetRight();
    setWingFront(false);
    setWingBack(false);

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    set_point(-13.0, 37.5, 2000, {.forwards = false, .minSpeed = 80});
    //set_raw(-64);
}

void right4() {
    // start
    set_position(-48, -12, 90.0);
    resetRight();
    resetBack();
    intake();

    pros::Task driveAtEnd([] {
        if (curMatchState == AUTO) {
            pros::delay(14800);
            set_drive(-1.0, 150, {.forwards = false,.minSpeed = 127});
            set_drive(1.0, 100, {.minSpeed = 127});
        }
    });

    // intake 3 stack 
    //set_point(-28, 19, 1000, {.minSpeed = 110, .earlyExitRange = 1.0}, false);
    set_point(-23.75, -22.5, 300, {.minSpeed = 127});
    set_point(-23.75, -22.5, 700, {}, false);
    wait_until(9.0);
    setLoader(true);
    wait();

    // get matchloader
    set_turn(225.0, 650);
    set_drive(23.0, 300, {.minSpeed = 127});
    set_drive(23.5, 500);
    resetFront();
    resetLeft();
    set_turn(270.0, 500);

    // score long goal
    set_point(-30, -48.0, 1000, {.forwards = false, .minSpeed = 127}, false);
    wait(350);
    score();
    outtake();
    wait(200);
    score();
    wait();
    set_raw(-127);
    wait(100);
    resetFront();
    resetLeft();
    wait(750);
    intake();

    // wing
    setWingFront(false);
    setWingBack(false);
    set_turn(217.5, 500, {.minSpeed = 127});
    set_swing(270.0, lemlib::DriveSide::RIGHT, 500, {.minSpeed = 127});
    resetFront();
    resetLeft();
    set_point(-14.5, -56, 1000, {.forwards = false, .minSpeed = 60});
}

void right7() {
    // start
    set_position(-48, -12, 90.0);
    resetRight();
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
    set_point(-23.75, -22.5, 300, {.minSpeed = 127});
    set_point(-23.75, -22.5, 700, {}, false);
    wait_until(9.0);
    setLoader(true);
    wait();

    // get matchloader
    set_turn(225.0, 650);
    set_drive(28.0, 300, {.minSpeed = 127});
    set_drive(28.5, 500);
    resetFront();
    resetLeft();
    set_turn(270.0, 500);
    resetFront();
    resetLeft();
    set_point(-66.0, -47.0, 1050, {.maxSpeed = 60});
    resetLeft();
    resetFront();

    // score long goal
    set_point(-30, -48.0, 1000, {.forwards = false, .minSpeed = 127}, false);
    wait(350);
    score();
    outtake();
    wait(200);
    score();
    wait();
    set_raw(-127);
    wait(100);
    resetFront();
    resetLeft();
    wait(1250);
    intake();
    
    // wing
    setWingFront(false);
    setWingBack(false);
    set_turn(217.5, 500, {.minSpeed = 127});
    set_swing(270.0, lemlib::DriveSide::RIGHT, 500, {.minSpeed = 127});
    resetFront();
    resetLeft();
    set_point(-12.5, -56, 1000, {.forwards = false, .minSpeed = 60});
}

void right43() {
    // start
    set_position(-48, -12, 90.0);
    resetRight();
    resetBack();
    intake();

    pros::Task driveAtEnd([] {
        if (curMatchState == AUTO) {
            pros::delay(14850);
            set_raw(127);
        }
    });

    // intake 3 stack 
    //set_point(-28, 19, 1000, {.minSpeed = 110, .earlyExitRange = 1.0}, false);
    set_point(-23.75, -22.5, 300, {.minSpeed = 127});
    set_point(-23.75, -22.5, 700, {}, false);
    wait_until(9.0);
    setLoader(true);
    wait();

    // get matchloader
    set_turn(225.0, 650);
    set_drive(28.0, 300, {.minSpeed = 127});
    set_drive(28.5, 500);
    resetFront();
    resetLeft();
    set_turn(270.0, 500);
    resetFront();
    resetLeft();
    set_point(-66.0, -47.0, 1050, {.maxSpeed = 55, .minSpeed = 55});
    resetLeft();
    resetFront();

    // score long goal
    set_point(-30, -48.0, 1000, {.forwards = false, .minSpeed = 127}, false);
    wait(350);
    score();
    outtake();
    wait(200);
    score();
    wait();
    set_raw(-127);
    wait(100);
    resetFront();
    resetLeft();
    wait(750);
    intake();

    setTech(true);
    set_turn(0.0, 1000, {.minSpeed = 90});
    resetLeft();
    resetBack();
    setLoader(false);
    //set_pose(-9, -8, 45.0, 500, {.minSpeed = 127});
    set_pose(-9, -8.5, 45.0, 1500);
    outtake();
    //intake1.move_velocity(-225);
    intake1.move(-60);
    set_turn(45.0, 300);
    wait(2000);
    outtake();

}

void doubleMid() {
    set_position(-48, 12, 90);
    intake();
    set_point(-24, 22, 1000, {.minSpeed = 90}, false);
    wait_until(9.0);
    setLoader(true);
    wait();
    setLoader(false);

    set_pose(-12, 50, 5.0, 2000);
    setLoader(true);
    set_turn(0.0, 600);
    resetFront();
    resetLeft();
    set_turn(5.0, 600);

    set_swing(90.0, lemlib::DriveSide::LEFT, 600);

    set_swing(315.0, lemlib::DriveSide::RIGHT, 1000);

    set_point(0, 0, 1000, {.forwards = false, .minSpeed = 30});
    scoreMiddle();
    intake2.move(-127 * 0.6);
    wait(1000);
    intake();
    
    set_point(-44, 44, 1000, {.forwards = false, .minSpeed = 127});
    set_turn(270.0, 600);
    resetRight();
    resetFront();
    set_point(-66, 46, 1000, {.forwards = false, .minSpeed = 127});




}

void techRush() {
    set_position(-48, -12, 90.0);
    resetRight();
    resetBack();
    intake();


    pros::Task scoreAtEnd([] {
        if (curMatchState == AUTO) {
            pros::delay(14500);
            midSlow = false;
            outtake();
        }
    });

    // intake 3 stack
    set_point(-24, -21, 1000, {}, false);
    wait_until(10.0);
    setLoader(true);
    wait();
    setTech(true);
    set_pose(0, 0, 43.0, 2000, {.minSpeed = 127}, false);
    wait(350);
    setLoader(false);
    wait();
    set_raw(127);
    midSlow = true;
    outtake();
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

void skillsZoneLowGoal() {
    set_position(-50, 0, 270.0);
    intake();
    intake2.move(127 * 0.3);
    resetFront();
    set_point(-70, 0, 300, {.maxSpeed = 40}, false);

    for(int i = 0; i < 2; i++)
	{
		set_turn(290.0, 100);
		set_turn(250.0, 100);
	}

    set_point(-90, 0, 400, {.maxSpeed = 110});
    resetFront();

    for(int i = 0; i < 2; i++)
	{
		set_turn(290.0, 100);
		set_turn(250.0, 100);
	}

    set_turn(270.0, 300, {}, false);
    set_drive(-100, 300, {.forwards = false, .maxSpeed = 40});

    set_point(72, chassis.getPose().y, 200, {.forwards = false, .maxSpeed = 40});
	set_point(-26, chassis.getPose().y, 1500, {.forwards = false});

    set_turn(180.0, 600);
    resetFront();
    resetRight();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    set_point(-28, -19, 400, {}, false);
    set_point(-28, -19, 800, {.maxSpeed = 80, .minSpeed = 1, .earlyExitRange = 1}, false);
    wait(50);
	set_point(-28, -21, 250, {.maxSpeed = 90, .minSpeed = 70});
		
	//move and score on lower goal
    set_turn(90.0, 450);
    resetRight();
    resetBack();
	set_turn(0, 0, 300, {.minSpeed = 1, .earlyExitRange = 3});

	//chassis.moveToPose(9, 11, 225, 1000, {.lead = 0.7});
	set_point(0, 0, 300, {});
    setTech(true);
	set_point(0, 0, 300, {.maxSpeed = 40});

	set_point(0, 0, 600, {.maxSpeed = 70});
    wait(300);
    // outtake();
    // wait(50);
    // intake();
    // wait(50);
		
	set_turn(45.0, 1000, {}, false);
    outtake();
	wait(200);
    intake1.move(-127*0.7);
    wait(100);
	intake1.move(-127*0.6);
	wait(300);
	intake1.move(-127*0.5); 
	wait(1000);
	intake1.move(-127*0.4);
	wait(1500);
    outtake();

	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    set_drive(-17.0, 1000, {.forwards = false, .minSpeed = 60});
    setTech(false);
    set_turn(0.0, 500);
    intake();
    resetLeft();
}

void skillsFirstHalf() {
    //set_position(chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
    resetLeft();

    set_point(-27, 23.0, 2000, {.minSpeed = 30}, false);
    wait_until(32.0);
    setLoader(true);
    wait();
    resetLeft();

    //matchloader 1
    set_point(-50.0, 45.0, 1000);
    set_turn(270.0, 600);
    resetFront();
    resetRight();
    set_pose(-67, 46.0, 270.0, 2500, {.maxSpeed = 60, .minSpeed = 50});
    resetFront();
    resetRight();
    set_drive(-4.0, 2000, {.forwards = false, .minSpeed = 40, .earlyExitRange = 2.0});
    set_point(-24, 60, 1000, {.forwards = false});
    set_turn(270.0, 300);

    set_point(22.75, 60, 1500, {.forwards = false}, false);
    wait(1000);
    setLoader(false);
    wait();
    resetRight();
    resetBack();
    set_point(36, 48.5, 1000, {.forwards = false, .minSpeed = 25});
    set_turn(90.0, 600);
    resetLeft();
    resetFront();

    set_point(26, 48, 1000, {.forwards = false, .minSpeed = 127});
    set_raw(-127);
    score();
    outtake();
    wait(100);
    score();
    wait(1650);

    setLoader(true);
    set_pose(66.0, 45.5, 90.0, 350, {.minSpeed = 127});
    intake();
    set_pose(66.0, 45.5, 90.0, 1500, {.maxSpeed = 55, .minSpeed = 50});
    //resetFront();
    resetLeft();

    set_point(26, 47.25, 1000, {.forwards = false, .minSpeed = 100});
    score();
    set_raw(-127);
    outtake();
    wait(100);
    score();
    wait(1650);
    resetLeft();
    resetFront();

}

void skillsHighGoal3Stacks() {
    //set_position(30, 48, 90.0);    
    intake();
    setLoader(false);
    set_turn(180.0, 1000, {.minSpeed = 80});
    intake();
    resetLeft();
    resetBack();
    set_point(23.5, 24.0, 1500, {.minSpeed = 90});
    set_point(23.5, -23.5, 1000, {.maxSpeed = 70}, false);
    wait_until(33.0);
    setLoader(true);
    wait();

    set_turn(180.0, 600);
    resetLeft();

    set_turn(0.0, 3.0, 600, {.forwards = false});
    set_drive(-18.0, 900, {.forwards = false, .minSpeed = 100});

    scoreMiddle();
    intake2.move(127*1.0);
	wait(200);
	intake2.move(127*0.5);
	wait(2500);
    intake();
}

void skillsSecondHalf() {
    set_point(44.0, -44.0, 500, {.minSpeed = 127});
    set_point(44.0, -44.0, 1000);
    set_turn(90.0, 600);
    resetFront();
    resetRight();
    set_point(67, -45.5, 2500, {.maxSpeed = 60, .minSpeed = 50});
    resetFront();
    resetRight();
    set_drive(-4.0, 2000, {.forwards = false, .minSpeed = 40, .earlyExitRange = 2.0});
    set_point(24, -60, 1000, {.forwards = false});
    set_turn(90.0, 500);
    resetRight();
    resetFront();

    set_point(-24.0, -60, 500, {.forwards = false, .minSpeed = 127});
    setLoader(false);
    set_point(-24.0, -60, 1000, {.forwards = false});
    wait();
    resetRight();
    resetFront();
    set_point(-36, -48, 1000, {.forwards = false, .minSpeed = 30});
    set_turn(270.0, 600);
    resetLeft();
    resetFront();

    set_point(-26, -48, 1000, {.forwards = false, .minSpeed = 127});
    set_raw(-127);
    score();
    outtake();
    wait(100);
    score();
    wait(1650);

    setLoader(true);
    intake();
    set_point(-66.0, -47.0, 3000, {.maxSpeed = 55, .minSpeed = 50});
    resetLeft();
    resetFront();

    set_point(-26, -48, 1000, {.forwards = false, .minSpeed = 127});
    score();
    set_raw(-127);
    outtake();
    wait(100);
    score();
    wait(1650);
    resetLeft();
    resetFront();
    intake();

}

void park() {
    //set_position(-30.0, -48.0, 270.0);
    score();
    setLoader(false);
    resetLeft();
    resetFront();
    set_pose(-63, -22, 350, 3000);
    intake();
    set_drive(32.0, 2000, {.minSpeed = 110});
    
}

void skills() {
    skillsZoneLowGoal();
    skillsFirstHalf();
    skillsHighGoal3Stacks();
    skillsSecondHalf();
    park();
}