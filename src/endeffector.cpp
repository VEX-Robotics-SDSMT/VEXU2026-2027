#include "endeffector.h"

namespace EndEffector {
EndEffector::EndEffector(pros::adi::Pneumatics* flipper,
                         pros::MotorGroup* scoreMotors)
    : flipper(flipper), scoreMotors(scoreMotors) {}

void EndEffector::periodic() {
  if (desiredState == State::RETRACTED) {
    scoreMotors->move(0);
    flipper->retract();
    currState = State::RETRACTED;
  } else if (desiredState == State::INTAKING_RETRACTED) {
    flipper->retract();
    scoreMotors->move(-rollerIntakingSpeed);
    currState = State::INTAKING_RETRACTED;
  } else if (desiredState == State::INTAKING_EXTENDED) {
    flipper->extend();
    scoreMotors->move(-rollerIntakingSpeed);
  } else if (desiredState == State::READY) {
    scoreMotors->move(-rollerIntakingSpeed);
    flipper->extend();
    currState = State::READY;
  } else if (desiredState == State::SCORING) {
    scoreMotors->move(rollerScoringSpeed);
    flipper->extend();
    currState = State::SCORING;
  } else if (desiredState == State::DEJAM) {
    flipper->retract();
    scoreMotors->move(rollerScoringSpeed);
  }
}

void EndEffector::setState(State newState) { desiredState = newState; }

State EndEffector::getCurrState() { return currState; }

State EndEffector::getDesiredState() { return desiredState; }

void EndEffector::toggleExtended() {
  if (currState == State::RETRACTED) {
    setState(State::READY);
  } else if (currState == State::READY) {
    setState(State::RETRACTED);
  } else if (currState == State::INTAKING_RETRACTED) {
    setState(State::INTAKING_EXTENDED);
  } else if (currState == State::INTAKING_EXTENDED ||
             currState == State::SCORING) {
    setState(State::INTAKING_RETRACTED);
  } else if (currState == State::DEJAM) {
    setState(State::SCORING);
  }
}

void EndEffector::toggleDirection() {
  if (currState == State::RETRACTED || currState == State::DEJAM) {
    setState(State::INTAKING_RETRACTED);
  } else if (currState == State::INTAKING_RETRACTED) {
    setState(State::RETRACTED);
  } else if (currState == State::INTAKING_EXTENDED) {
    setState(State::SCORING);
  } else if (currState == State::READY) {
    setState(State::SCORING);
  } else if (currState == State::SCORING) {
    setState(State::READY);
  }
}
}  // namespace EndEffector