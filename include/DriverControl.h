#pragma once

#include "Drivetrain.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"

namespace Driver {
class Driver {
 public:
  Driver(Mines::Drivetrain* driveTrain, pros::Controller* master);

  void periodic();

  void AlexDrive();

  void AlfredoDrive();

  void setDrive(int32_t x, int32_t y);

 private:
  Mines::Drivetrain* driveTrain;
  pros::MotorGroup* rightMotors;
  pros::Controller* master;

  int32_t xInput, yInput, leftVoltage, rightVoltage = 0;
};

}  // Namespace Driver