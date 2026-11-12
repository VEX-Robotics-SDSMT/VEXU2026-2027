#pragma once

#include "pros/adi.hpp"
#include "pros/motor_group.hpp"

namespace Intake {
enum class State {
  // if state & 0b01 == 1, then the intake is extended
  // if state & 0b10 >> 1 == 1, then the wheels are spinning
  IDLE_RETRACTED = 0b000,
  IDLE_EXTENDED = 0b001,
  INTAKING_RETRACTED = 0b010,
  INTAKING_EXTENDED = 0b011,
  OUTTAKING_RETRACTED = 0b100,
  OUTTAKING_EXTENDED = 0b101,
};

class Intake {
 public:
  Intake(pros::adi::Pneumatics* liftPneumatics, pros::MotorGroup* rollerMotors);

  void periodic();

  void setState(State newState);

  void stopWheels();
  void startWheels();
  void outtake();
  void extend();
  void retract();

  void toggleWheels();
  void toggleExtended();
  void toggleOuttake();

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