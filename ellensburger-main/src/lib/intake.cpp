#include "lib/intake.hpp"
#include "pros/rtos.hpp"
#include "robotconfig.h"



using namespace lib;



void Intake::loop() { while (true) {


  // Color Sort Handling
  if (sortEnabled && direction == Dir::In && ((teamColor == team::blue && color->seesRed()) || (teamColor != team::blue && color->seesBlue()))) {
    sortTickCount++;
  } else {
    if (sortTickCount >= requiredSortTicks) {
      motors->move(-127);
      pros::delay(250);
    }
    sortTickCount = 0;
  }
  

  // Jam Handling
  if (direction == Dir::In && std::abs(motors->get_actual_velocity()) < 5) { switch (jamMode) {
    case Jam::None:
      break;
    
    case Jam::Reverse:
      if (jamStartTime == 0) { jamStartTime = pros::millis(); }
      else if (pros::millis() - jamStartTime > 75) {
        jamStartTime = 0;
        motors->move(-127);
        pros::delay(250);
      }
      break;
    
    case Jam::Tap:
      if (jamStartTime == 0 && !ringSeated) { jamStartTime = pros::millis(); }
      else if (jamStartTime != 0 && pros::millis() - jamStartTime > 20) {
        jamStartTime = 0;
        ringSeated = true;
        motors->move(0);
        pros::delay(150);
        motors->move(127);
        pros::delay(150);
      }
      break;

  } } else { jamStartTime = 0; }
  
  
  // Manual Control
  if (ringSeated) { motors->move(0); }
  else {
    switch (direction) {
    case Dir::Idle:
      motors->move(0);
      break;
    case Dir::In:
      motors->move(127);
      break;
    case Dir::Out:
      motors->move(-127);
      break;
    }
  }
  pros::delay(10);
} }