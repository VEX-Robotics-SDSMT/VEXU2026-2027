#pragma once

#include "pros/adi.hpp"
#include "pros/motor_group.hpp"

namespace Intake {
enum class State {
  // if state & 0b01 == 1, then the wheels are spinning
  // if state & 0b10 >> 1 == 1, then the intake is extended
  IDLE_RETRACTED = 0b00,
  IDLE_EXTENDED = 0b01,
  INTAKING_RETRACTED = 0b10,
  INTAKING_EXTENDED = 0b11,
};

class Intake {
 public:
  Intake(pros::adi::Pneumatics* liftPneumatics, pros::MotorGroup* rollerMotors);

  void periodic();

  void setState(State newState);

  void stopWheels();
  void startWheels();
  void extend();
  void retract();

  void toggleWheels();
  void toggleExtended();

  State getDesiredState();
  State getCurrState();

 private:
  const int32_t intakeVoltage = 127;

  pros::adi::Pneumatics* liftPneumatics;
  pros::MotorGroup* rollerMotors;

  State desiredState{State::IDLE_RETRACTED};
  State currState{State::IDLE_RETRACTED};
};

}  // namespace Intake