#include "lift.h"

namespace Mines::Lift {
Lift::Lift(pros::MotorGroup* motorGroup, pros::Rotation* rotationSensor)
    : motors(motorGroup),
      rotationSensor(rotationSensor),
      currControlMode(ControlMode::POSITION),
      setpointValid(false) {};

void Lift::periodic() {
  if (setpointValid) {
    if (currControlMode == ControlMode::POSITION) {
      double angleRots = rotationSensor->get_angle() / 100.0 / 360.0;

      motors->move_absolute(
          (int32_t)rotationToHeight(angleRots),
          (int32_t)heightToRotation(maxVelInchesPerSecond) / 60.0);
    } else if (currControlMode == ControlMode::VELOCITY) {
      motors->move(setpoint / heightToRotation(maxVelInchesPerSecond) * 127);
    }
  } else {
    motors->move_voltage(0);
  }
}

void Lift::setControlMode(ControlMode newControlMode) {
  currControlMode = newControlMode;
  setpointValid = false;
}

void Lift::setSepointPosition(inch setpoint) {
  this->setpoint = setpoint;
  setpointValid = true;
}

void Lift::setSepointVelocity(double rotationsPerMinute) {
  this->setpoint = setpoint;
  setpointValid = true;
}

inch Lift::getSetpoint() { return setpoint; }

inch Lift::rotationToHeight(double rotations) {
  return rotations * inchesHeightPerRotation;
}

double Lift::heightToRotation(inch height) {
  return height / inchesHeightPerRotation;
}

}  // namespace Mines::Lift
