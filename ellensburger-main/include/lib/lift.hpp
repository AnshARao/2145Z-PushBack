#include "lib/TaskWrapper.hpp"
#include "pid.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"


namespace lib {


enum class LiftStates { Position, Voltage, Hold };


class Lift : public ryan::TaskWrapper {

private:

  std::shared_ptr<pros::MotorGroup> motors;
  std::shared_ptr<pros::Rotation> rotation;

  LiftStates state = LiftStates::Hold;
  PID pid = PID(2.25, 0, 8);

  float target;


public:
  Lift(pros::MotorGroup* motors, pros::Rotation* rotation) : motors(motors), rotation(rotation) {}

  void loop() override {
    motors->set_brake_mode_all(pros::MotorBrake::hold);

    while (true) {
      float position = rotation->get_angle() / 100.0;
      if (position > 330) { position -= 360; }

      switch (state) {
      case LiftStates::Voltage:
        if (target < -0.1 and position < 10) {
          motors->move(0);
        } else {
          motors->move(target);
        }
        break;

      case LiftStates::Position:
        motors->move(pid.compute(target - position));
        break;

      case LiftStates::Hold:
        motors->brake();
        break;
      }

      pros::delay(10);
    }
  };

  void hold() { state = LiftStates::Hold; }

  void setTarget(float newTarget) { target = newTarget; state = LiftStates::Position; }

  void setVoltage(float newVoltage) { target = newVoltage; state = LiftStates::Voltage; }

  float getAngle() { return rotation->get_angle() / 100.0; };
};
} // namespace lib