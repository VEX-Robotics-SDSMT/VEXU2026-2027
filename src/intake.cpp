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
    if (desiredState == State::IDLE_EXTENDED) {
      rollerMotors->move(0);
      liftPneumatics->extend();
      currState = State::IDLE_EXTENDED;
    } else if (desiredState == State::IDLE_RETRACTED) {
      rollerMotors->move(0);
      liftPneumatics->retract();
      currState = State::IDLE_EXTENDED;
    } else if (desiredState == State::INTAKING_RETRACTED) {
      rollerMotors->move(intakeVoltage);
      liftPneumatics->retract();
      currState = State::INTAKING_RETRACTED;
    } else if (desiredState == State::INTAKING_EXTENDED) {
      rollerMotors->move(intakeVoltage);
      liftPneumatics->extend();
      currState = State::INTAKING_EXTENDED;
    } else if (desiredState == State::OUTTAKING_RETRACTED) {
      rollerMotors->move(outtakeVoltage);
      liftPneumatics->retract();
      currState = State::OUTTAKING_RETRACTED;
    } else if (desiredState == State::OUTTAKING_EXTENDED) {
      rollerMotors->move(outtakeVoltage);
      liftPneumatics->extend();
      currState = State::OUTTAKING_EXTENDED;
    }
  }
}

void Intake::setIdle() {
  if (liftPneumatics->is_extended()) {
    Intake::setState(State::IDLE_EXTENDED);
  } else {
    Intake::setState(State::IDLE_RETRACTED);
  }
}

void Intake::setIntaking() {
  if (liftPneumatics->is_extended()) {
    Intake::setState(State::INTAKING_EXTENDED);
  } else if (!liftPneumatics->is_extended()) {
    Intake::setState(State::INTAKING_RETRACTED);
  }
}

void Intake::setOuttaking() {
  if (liftPneumatics->is_extended()) {
    Intake::setState(State::OUTTAKING_EXTENDED);
  } else if (!liftPneumatics->is_extended()) {
    Intake::setState(State::OUTTAKING_RETRACTED);
  }
}

void Intake::setExtended() {
  if (rollerMotors->get_voltage() == intakeVoltage) {
    Intake::setState(State::INTAKING_EXTENDED);
  } else if (rollerMotors->get_voltage() == outtakeVoltage) {
    Intake::setState(State::OUTTAKING_EXTENDED);
  } else {
    Intake::setState(State::IDLE_EXTENDED);
  }
}

void Intake::setRetracted() {
  if (rollerMotors->get_voltage() == intakeVoltage) {
    Intake::setState(State::INTAKING_RETRACTED);
  } else if (rollerMotors->get_voltage() == outtakeVoltage) {
    Intake::setState(State::OUTTAKING_RETRACTED);
  } else {
    Intake::setState(State::IDLE_RETRACTED);
  }
}

void Intake::setState(State newState) { desiredState = newState; }

State Intake::getDesiredState() { return desiredState; }

State Intake::getCurrState() { return currState; }

}  // namespace Intake