#include "intake.h"

#include "pros/adi.hpp"
#include "pros/motor_group.hpp"

namespace Intake {
Intake::Intake(pros::adi::Pneumatics* liftPneumatics,
               pros::MotorGroup* rollerMotors)
    : liftPneumatics(liftPneumatics), rollerMotors(rollerMotors) {}

void Intake::periodic() {
  if (desiredState != currState) {
    if (desiredState == State::IDLE) {
      rollerMotors->move(0);
    } else if (desiredState == State::INTAKING_RETRACTED) {
      rollerMotors->move(intakeVoltage);
      liftPneumatics->retract();
    } else if (desiredState == State::INTAKING_EXTENDED) {
      rollerMotors->move(intakeVoltage);
      liftPneumatics->extend();
    }
  }
}

void Intake::setState(State newState) { desiredState = newState; }

State Intake::getDesiredState() { return desiredState; }

State Intake::getCurrState() { return currState; }
}  // namespace Intake