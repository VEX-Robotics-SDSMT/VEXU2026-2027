#pragma once

#include "pros/adi.hpp"

namespace Claw {

enum class State {
  CLOSED = false,
  OPEN = true,
};

class Claw {
 public:
  Claw(pros::adi::Pneumatics& clawPneumatics);

  void periodic();

  void setState(State newState);

  void open();
  void close();
  void toggle();

  bool isOpen();

 private:
  pros::adi::Pneumatics& m_pneumatics;

  State currState;
  State desiredState;
};
};  // namespace Claw