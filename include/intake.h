#pragma once

#include "pros/adi.hpp"
#include "pros/motor_group.hpp"

namespace Intake {
enum class State {
  IDLE,
  INTAKING_EXTENDED,
  INTAKING_RETRACTED,
};

class Intake {
 public:
  Intake(pros::adi::Pneumatics* liftPneumatics, pros::MotorGroup* rollerMotors);

  void periodic();

  void setState(State newState);

  State getDesiredState();
  State getCurrState();

 private:
  const int32_t intakeVoltage = 127;

  pros::adi::Pneumatics* liftPneumatics;
  pros::MotorGroup* rollerMotors;

  State desiredState{State::IDLE};
  State currState{State::IDLE};
};

}  // namespace Intake