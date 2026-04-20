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
    set_point(chassis.getPose().x, -46.6, 2000, {.forwards = false});
    resetBack();
    resetLeft();
    setLoader(true);

    set_turn(-66.0, -47.0, 650);
    resetLeft();
    set_point(-68.0, -47, 1000, {.maxSpeed = 65, .minSpeed = 60});
    resetLeft();
    resetFront();
    set_point(-28.0, -48.0, 800, {.forwards = false, .minSpeed = 127});
    score();
    set_raw(-127.0);
    wait(1150);
    setLoader(false);
    intake();

    // turn & intake 3 stacks
    set_turn(0.0, 1000, {.minSpeed = 60});
    resetLeft();
    resetBack();
    set_point(-23.5, -24.0, 3000, {.minSpeed = 90});
    set_point(-23.5, 23.0, 3000, {.minSpeed = 30}, false);
    wait_until(33.0);
    setLoader(true);
    wait();

    //score 2nd long goal
    set_point(-44.0, 44.0, 3000, {.minSpeed = 90});
    wait(50);
    set_turn(270.0, 650);
    resetFront();
    resetRight();
    // score long goal
    set_point(-30, 46.5, 1000, {.forwards = false, .minSpeed = 127});
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
    set_point(-66.0, 45.5, 1550, {.maxSpeed = 65, .minSpeed = 60});
    wait(50);
    resetFront();
    resetRight();
    set_pose(-9.5, 8, 315.0, 2000, {.forwards = false, .minSpeed = 120});
    scoreMiddle();
    intake2.move_velocity(225);
    set_turn(0,0, 500, {.forwards = false});
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
    set_drive(36.0, 1500, {.minSpeed = 100, .earlyExitRange = 1.0});
    set_turn(270.0, 400);
    resetFront();
    resetRight();
    set_point(-66.0, 45.5, 1000, {.maxSpeed = 60});
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
    set_point(-11, 36, 1000, {.forwards = false, .minSpeed = 60});
}

void left43mid() {

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
    set_point(-24, 22, 900, {.minSpeed = 90}, false);
    wait_until(9.0);
    setLoader(true);
    wait();

    // get matchloader
    set_turn(315.0, 600);
    setWingFront(true);
    setWingBack(true);
    set_drive(29.0, 1500, {.minSpeed = 90});
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

    //matchload and score middle
    set_point(-66.0, 45.5, 1450, {.maxSpeed = 60});
    resetFront();
    resetRight();
    set_pose(-9.5, 8, 315.0, 2000, {.forwards = false, .minSpeed = 120});
    scoreMiddle();
    intake2.move_velocity(215);
    set_turn(0,0, 500, {.forwards = false});
    wait(waittime);
    setLoader(false);
    setDescore(true);
    set_drive(8.0, 1000, {.minSpeed = 127});
    set_drive(-12.0, 1000, {.forwards = false, .minSpeed = 60});
}

void left43long() {

    int waittime = 2000;
    int waittime2 = 500;
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

    //matchload and score middle
    set_point(-66.0, 45.5, 1450, {.maxSpeed = 60});
    resetFront();
    resetRight();
    set_pose(-9.5, 8, 315.0, 2000, {.forwards = false, .minSpeed = 120});
    scoreMiddle();
    intake2.move_velocity(215);
    set_turn(0,0, 500, {.forwards = false});
    wait(waittime);
    setLoader(false);
    setDescore(true);
    set_drive(8.0, 1000, {.minSpeed = 127});
    set_drive(-12.0, 1000, {.forwards = false, .minSpeed = 60});

    //wing long goal
    wait(waittime2);
    set_drive(10.0, 200, {.minSpeed = 127});
    set_pose(-36, 34, 270, 2000, {.forwards = true});
    setWingBack(false);
    setWingFront(false);
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
                pros::delay(14900);
                set_drive(2.0, 1000, {.minSpeed = 90});
            }
        });
    
        // intake 3 stack 
        //set_point(-28, 19, 1000, {.minSpeed = 110, .earlyExitRange = 1.0}, false);
        set_point(-24, -22.5, 1000, {.minSpeed = 90}, false);
        wait_until(9.0);
        setLoader(true);
        wait();
    
        // get matchloader
        set_turn(234.0, 600);
        set_drive(30.0, 2000, {.minSpeed = 80});
        resetFront();
        resetLeft();
        set_turn(270.0, 500);
        resetFront();
        resetLeft();
        set_point(-66.0, -47.0, 1100, {.maxSpeed = 60});
        resetLeft();
        resetFront();
    
        // score long goal
        set_point(-30, -48.0, 1000, {.forwards = false, .minSpeed = 100}, false);
        wait(300);
        outtake();
        wait(200);
        score();
        wait();
        set_raw(-127);
        wait(100);
        resetFront();
        resetLeft();
        wait(1150);
        intake();
    
        // wing
        setWingFront(false);
        setWingBack(false);
        set_turn(220.0, 500, {.minSpeed = 127});
        set_swing(270.0, lemlib::DriveSide::RIGHT, 500);
        wait(50);
        resetFront();
        resetLeft();
        set_point(-11, -56, 1000, {.forwards = false});
}

void right43mid() {
    // start
    set_position(-48, -12, 90.0);
    resetRight();
    resetBack();
    intake();
    // intake 3 stack 
    //set_point(-28, 19, 1000, {.minSpeed = 110, .earlyExitRange = 1.0}, false);
    set_point(-24, -22.5, 1000, {.minSpeed = 90}, false);
    wait_until(9.0);
    setLoader(true);
    wait();

    // get matchloader
    set_turn(234.0, 600);
    set_drive(30.0, 2000, {.minSpeed = 100});
    resetFront();
    resetLeft();
    set_turn(270.0, 500);
    resetFront();
    resetLeft();
    set_point(-66.0, -47.0, 1100, {.maxSpeed = 60});
    resetLeft();
    resetFront();

    // score long goal
    set_point(-30, -48.0, 1000, {.forwards = false, .minSpeed = 100}, false);
    wait(300);
    outtake();
    wait(200);
    score();
    wait();
    set_raw(-127);
    wait(100);
    resetFront();
    resetLeft();
    wait(1150);
    intake();

    setTech(true);
    set_turn(0.0, 700);
    resetLeft();
    resetBack();
    setLoader(false);
    set_pose(-9, -8, 45.0, 2000, {.forwards = true, .minSpeed = 70});
    outtake();
    intake1.move_velocity(-400);
    set_turn(45.0, 1000);
    set_raw(127);
}

void right43long() {
    
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
    wait(50);
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
    outtake();
    wait(50);
    intake();
    wait(50);
		
	intake1.move(127*0.7);
		
	set_turn(45.0, 1000, {}, false);

	wait(500);
    intake2.move(-127);
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
    intake();
    set_turn(0.0, 500);
    resetLeft();
}

void skillsFirstHalf() {
    set_position(chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
    resetLeft();

    set_point(-27, 23.0, 2000, {.minSpeed = 30}, false);
    wait_until(32.0);
    setLoader(true);
    wait();

    //matchloader 1
    set_point(-44.0, 44.0, 2000, {.minSpeed = 90});
    set_turn(270.0, 600);
    resetFront();
    resetRight();
    set_pose(-67, 45.0, 270.0, 2500, {.maxSpeed = 60, .minSpeed = 50});
    resetFront();
    resetRight();
    set_drive(-4.0, 2000, {.forwards = false, .minSpeed = 40, .earlyExitRange = 2.0});
    set_point(-24, 60, 1000, {.forwards = false});
    set_turn(270.0, 300);
    resetRight();
    resetFront();

    set_point(22.75, 60, 1500, {.forwards = false}, false);
    wait(1000);
    setLoader(false);
    wait();
    set_point(36, 48, 1000, {.forwards = false, .minSpeed = 25});
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
    intake();
    set_pose(66.0, 46.0, 90.0, 3000, {.maxSpeed = 55, .minSpeed = 50});
    //resetFront();
    resetLeft();

    set_point(26, 48.5, 1000, {.forwards = false, .minSpeed = 100});
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

void skillsHighGoal3Stacks() {
    set_position(30, 48, 90.0);    
    intake();
    setLoader(false);
    set_turn(180.0, 1000, {.minSpeed = 80});
    resetLeft();
    resetBack();
    set_point(23.5, 24.0, 1500, {.minSpeed = 90});
    set_point(23.5, -23.5, 1500, {.maxSpeed = 70}, false);
    wait_until(33.0);
    setLoader(true);
    wait();

    set_turn(180.0, 600);
    resetLeft();

    set_turn(0.0, 0.0, 600, {.forwards = false});
    set_drive(-20.0, 1000, {.forwards = false, .minSpeed = 100});


    scoreMiddle();
    intake2.move(127*0.7);

	wait(700);
	intake2.move(127*0.6);
	wait(300);
	intake2.move(127*0.5); 
	wait(1000);
	intake2.move(127*0.4);
	wait(1500);
    intake();
}

void skillsSecondHalf() {
    set_point(44.0, -44.0, 3000, {.minSpeed = 90});
    set_turn(90.0, 600);
    resetFront();
    resetRight();
    set_point(67, -45.0, 2500, {.maxSpeed = 60, .minSpeed = 50});
    resetFront();
    resetRight();
    set_drive(-4.0, 2000, {.forwards = false, .minSpeed = 40, .earlyExitRange = 2.0});
    set_point(24, -60, 1000, {.forwards = false});
    set_turn(90.0, 500);
    resetRight();
    resetFront();

    set_point(-24.0, -60, 1500, {.forwards = false}, false);
    wait(1000);
    setLoader(false);
    wait();
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
    set_point(-66.0, -46.0, 3000, {.maxSpeed = 55, .minSpeed = 50});
    resetLeft();
    resetFront();

    set_point(-26, -48.25, 1000, {.forwards = false, .minSpeed = 127});
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
    set_position(-30.0, -48.0, 270.0);
    score();
    setLoader(false);
    resetLeft();
    resetFront();
    set_point(-63.0, -20.0, 2000, {.minSpeed = 90});
    set_swing(350.0, lemlib::DriveSide::RIGHT, 600, {.minSpeed = 30, .earlyExitRange = 1.0});
    set_drive(40.0, 2000, {.minSpeed = 110});
    
}

void skills() {
    skillsZoneLowGoal();
    skillsFirstHalf();
    skillsHighGoal3Stacks();
    skillsSecondHalf();
    park();
}