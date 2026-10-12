#pragma once

#include <math.h>

#include "PID.h"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

namespace Lift {

typedef double inch;

class Lift {
 public:
  Lift(pros::MotorGroup* motors, pros::Rotation* rotationSensor);

  void setSetpointPosition(inch setpoint);

  inch getSetpoint();

  void periodic();

 private:
  const inch minHeight = 0.0;
  const inch maxHeight = 48.0;
  const inch maxVelInchesPerSecond = 48.0;

  const int numStages = 3;

  // diameter = 0.65 in
  const inch inchesHeightPerRotation = 0.65 * M_PI * numStages;

  pros::MotorGroup* motors;
  pros::Rotation* rotationSensor;

  bool setpointValid;
  inch setpoint;
  inch position;

  Mines::PID positionPID;

  inch rotationToHeight(double rotations);
  double heightToRotation(inch height);
};
}  // namespace Lift
