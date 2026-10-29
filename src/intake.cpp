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
    if (desiredState == State::IDLE_RETRACTED) {
      rollerMotors->move(0);
      liftPneumatics->retract();
      currState = State::IDLE_RETRACTED;
    } else if (desiredState == State::IDLE_EXTENDED) {
      rollerMotors->move(0);
      liftPneumatics->extend();
      currState = State::IDLE_EXTENDED;
    } else if (desiredState == State::INTAKING_RETRACTED) {
      rollerMotors->move(intakeVoltage);
      liftPneumatics->retract();
      currState = State::INTAKING_RETRACTED;
    } else if (desiredState == State::INTAKING_EXTENDED) {
      rollerMotors->move(intakeVoltage);
      liftPneumatics->extend();
      currState = State::INTAKING_EXTENDED;
    } else if (desiredState == State::OUTTAKING_EXTENDED) {
      rollerMotors->move(-intakeVoltage);
      liftPneumatics->extend();
      currState = State::OUTTAKING_EXTENDED;
    } else if (desiredState == State::OUTTAKING_RETRACTED) {
      rollerMotors->move(-intakeVoltage);
      liftPneumatics->retract();
      currState = State::OUTTAKING_EXTENDED;
    }
  }
}

void Intake::setState(State newState) { desiredState = newState; }

State Intake::getDesiredState() { return desiredState; }

State Intake::getCurrState() { return currState; }

void Intake::stopWheels() {
  if (static_cast<int>(currState) & 0b01 == 0b01) {
    desiredState = State::IDLE_EXTENDED;
  } else {
    desiredState = State::IDLE_RETRACTED;
  }
}

void Intake::startWheels() {
  if (static_cast<int>(currState) & 0b001 == 0b001) {
    desiredState = State::INTAKING_EXTENDED;
  } else {
    desiredState = State::INTAKING_RETRACTED;
  }
}

void Intake::outtake() {
  if (static_cast<int>(currState) & 0b001) {
    desiredState = State::OUTTAKING_EXTENDED;
  } else {
    desiredState = State::OUTTAKING_RETRACTED;
  }
}

void Intake::extend() {
  desiredState = static_cast<State>(static_cast<int>(currState) | 0b1);
}

void Intake::retract() {
  desiredState = static_cast<State>(static_cast<int>(currState) & 0b110);
}

void Intake::toggleWheels() {
  if (static_cast<int>(currState) & 0b10 == 0b10) {
    stopWheels();
  } else {
    startWheels();
  }
}

void Intake::toggleExtended() {
  desiredState = static_cast<State>(static_cast<int>(currState) ^ 0b1);
}

}  // namespace Intake