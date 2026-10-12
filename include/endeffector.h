#pragma once

#include "pros/adi.hpp"
#include "pros/motor_group.hpp"

namespace EndEffector {
enum class State {
  RETRACTED,
  READY,
  SCORING,
};

class EndEffector {
 public:
  EndEffector(pros::adi::Pneumatics* flipper, pros::MotorGroup* scoreMotors);

  void periodic();

  void setState(State newState);

  State getDesiredState();
  State getCurrState();

 private:
  const int32_t rollerSpeed = 75;

  State currState{State::RETRACTED};
  State desiredState{State::RETRACTED};

  pros::adi::Pneumatics* flipper;
  pros::MotorGroup* scoreMotors;
};
}  // namespace EndEffector