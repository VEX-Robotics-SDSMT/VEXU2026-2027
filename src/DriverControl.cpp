#pragma once
#include "DriverControl.h"

#include "misc.hpp"
#include "pros/motor_group.hpp"

namespace Driver {
Driver::Driver(Mines::Drivetrain* driveTrain, pros::Controller* master)
    : driveTrain(driveTrain), master(master) {}

void Driver::periodic() { driveTrain->driveVoltage(leftVoltage, rightVoltage); }

void Driver::AlexDrive() {
  leftVoltage = xInput + yInput;
  rightVoltage = xInput - yInput;

  return;
}

void Driver::AlfredoDrive() { return; }

void Driver::setDrive(int32_t x, int32_t y) {
  yInput = y;
  xInput = x;
}

}  // Namespace Driver
