#include "intake.h"

#include <iostream>

#include "pros/adi.hpp"
#include "pros/motor_group.hpp"

namespace Intake {
Intake::Intake(pros::adi::Pneumatics* liftPneumatics,
               pros::MotorGroup* rollerMotors)
    : liftPneumatics(liftPneumatics), rollerMotors(rollerMotors) {}

void Intake::periodic() {
  if (desiredState != currState) {
    std::cout << static_cast<int>(desiredState) << std::endl;
    if (desiredState == State::IDLE) {
      rollerMotors->move(0);
      currState = State::IDLE;
    } else if (desiredState == State::INTAKING_RETRACTED) {
      rollerMotors->move(intakeVoltage);
      liftPneumatics->retract();
      currState = State::INTAKING_RETRACTED;
    } else if (desiredState == State::INTAKING_EXTENDED) {
      rollerMotors->move(intakeVoltage);
      liftPneumatics->extend();
      currState = State::INTAKING_EXTENDED;
    }
  }
}

void Intake::setState(State newState) { desiredState = newState; }

State Intake::getDesiredState() { return desiredState; }

State Intake::getCurrState() { return currState; }
}  // namespace Intake