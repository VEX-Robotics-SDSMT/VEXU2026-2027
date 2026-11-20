#pragma once

#include <math.h>

#include <functional>
#include <variant>

#include "PID.h"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

namespace Lift {

typedef double inch;

namespace States {
struct Idle {};

struct VoltageCallable {
  std::function<int32_t(void)> voltageCallable;
};

struct Voltage {
  int32_t voltage;
};

struct Position {
  inch height;
};

using State = std::variant<Idle, VoltageCallable, Voltage, Position>;

}  // namespace States

class Lift {
 public:
  Lift(pros::MotorGroup* motors, pros::Rotation* rotationSensor);

  void setState(States::State newState);

  States::State getState();

  void periodic();

  inch getHeight();

 private:
  const inch minHeight = 0.0;
  const inch maxHeight = 48.0;
  const inch maxVelInchesPerSecond = 48.0;

  const int numStages = 3;

  // diameter = 0.65 in
  const inch inchesHeightPerRotation =
      0.65 * M_PI * numStages * 114.653385739 * 1.25294276689 * 0.76065634799;

  pros::MotorGroup* motors;
  pros::Rotation* rotationSensor;

  bool setpointValid;
  inch position;

  States::State currState;

  Mines::PID positionPID;

  inch rotationToHeight(double rotations);
  double heightToRotation(inch height);
};
}  // namespace Lift