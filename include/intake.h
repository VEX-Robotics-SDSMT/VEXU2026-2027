#pragma once

#include "pros/adi.hpp"
#include "pros/motor_group.hpp"

namespace Intake {
enum class State {
  IDLE_EXTENDED,
  IDLE_RETRACTED,
  INTAKING_EXTENDED,
  INTAKING_RETRACTED,
  OUTTAKING_EXTENDED,
  OUTTAKING_RETRACTED,
};

class Intake {
 public:
  Intake(pros::adi::Pneumatics* liftPneumatics, pros::MotorGroup* rollerMotors);

  void periodic();

  void setState(State newState);

  State getDesiredState();
  State getCurrState();
  void setIdle();
  void setIntaking();
  void setOuttaking();
  void setExtended();
  void setRetracted();

 private:
  const int32_t intakeVoltage = 127;
  const int32_t outtakeVoltage = -127;

  pros::adi::Pneumatics* liftPneumatics;
  pros::MotorGroup* rollerMotors;

  State desiredState{State::IDLE_RETRACTED};
  State currState{State::IDLE_RETRACTED};
};

}  // namespace Intake