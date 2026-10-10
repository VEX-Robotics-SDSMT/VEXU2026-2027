#pragma once

#include <math.h>

#include "devices.h"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

namespace Mines::Lift {

typedef double inch;

enum class ControlMode {
  POSITION = 0,
  VELOCITY = 1,
}

class Lift {
 public:
  Lift(pros::MotorGroup* motors, pros::Rotation* rotationSensor);

  void setSepointPosition(inch setpoint);
  void setSepointVelocity(double rotationsPerMinute);

  void setControlMode(ControlMode newControlMode);

  inch getSetpoint();

  void periodic();

 private:
  const inch minHeight = 0.0;
  const inch maxHeight = 48.0;
  const inch maxVelInchesPerSecond = 48.0;

  const int numStages = 3;

  ControlMode currControlMode;

  // diameter = 0.65 in
  const inch inchesHeightPerRotation = 0.65 * M_PI * numStages;

  pros::MotorGroup* motors;
  pros::Rotation* rotationSensor;

  bool setpointValid;
  inch setpoint;
  inch position;

  inch rotationToHeight(double rotations);
  double heightToRotation(inch height);
};
}  // namespace Mines::Lift
