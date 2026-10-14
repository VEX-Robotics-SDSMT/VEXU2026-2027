#include "endeffector.h"

namespace EndEffector {
EndEffector::EndEffector(pros::adi::Pneumatics* flipper,
                         pros::MotorGroup* scoreMotors)
    : flipper(flipper), scoreMotors(scoreMotors) {}

void EndEffector::periodic() {
  if (desiredState != currState) {
    if (desiredState == State::RETRACTED) {
      scoreMotors->move(0);
      flipper->retract();
      currState = State::RETRACTED;
    } else if (desiredState == State::INTAKING_RETRACTED) {
      flipper->retract();
      scoreMotors->move(-rollerIntakingSpeed);
    } else if (desiredState == State::READY) {
      scoreMotors->move(0);
      flipper->extend();
      currState = State::READY;
    } else if (desiredState == State::SCORING) {
      // TODO: Should the class guarantee that we are ready before scoring?
      // impl probably involves a timer and third state variable
      scoreMotors->move(rollerScoringSpeed);
      flipper->extend();
      currState = State::SCORING;
    }
  }
}

void EndEffector::setState(State newState) { currState = newState; }

State EndEffector::getCurrState() { return currState; }

State EndEffector::getDesiredState() { return desiredState; }
}  // namespace EndEffector