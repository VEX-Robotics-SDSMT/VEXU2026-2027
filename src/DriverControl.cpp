#pragma once
#include "DriverControl.h"

#include "misc.hpp"
#include "pros/motor_group.hpp"

namespace Driver {
Driver::Driver(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors,
               pros::Controller* master)
    : leftMotors(leftMotors), rightMotors(rightMotors), master(master) {}

void Driver::periodic() { return; }

void AlexDrive() { return; }

void AlfredoDrive() { return; }

void Driver::setDrive(int32_t x, int32_t y) {
  yInput = y;
  xInput = x;
}

}  // Namespace Driver
