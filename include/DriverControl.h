#pragma once

#include "Drivetrain.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"

namespace Driver {
class Driver {
 public:
  Driver(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors,
         pros::Controller* master);

  void periodic();

  void AlexDrive();

  void AlfredoDrive();

  void setDrive(int32_t x, int32_t y);

 private:
  pros::MotorGroup* leftMotors;
  pros::MotorGroup* rightMotors;
  pros::Controller* master;
  int32_t xInput, yInput = 0;
};

}  // Namespace Driver