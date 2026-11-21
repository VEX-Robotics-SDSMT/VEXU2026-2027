#include "claw.hpp"

#include <iostream>

namespace Claw {
Claw::Claw(pros::adi::Pneumatics& clawPneumatics)
    : m_pneumatics(clawPneumatics) {}

void Claw::periodic() {
  if (currState != desiredState) {
    if (desiredState == State::OPEN) {
      m_pneumatics.extend();
      currState = State::OPEN;
    } else if (desiredState == State::CLOSED) {
      m_pneumatics.retract();
      currState = State::CLOSED;
    } else {
      std::cout << "Claw reached unreachable state\n";
    }
  }
}

void Claw::setState(State newState) { desiredState = newState; }

void Claw::open() { setState(State::OPEN); }

void Claw::close() { setState(State::CLOSED); }

void Claw::toggle() {
  if (isOpen()) {
    close();
  } else {
    open();
  }
}

bool Claw::isOpen() { return currState == State::OPEN; }
}  // namespace Claw
