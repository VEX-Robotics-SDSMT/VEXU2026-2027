#include "lift.h"

#include <iostream>

namespace Lift {
Lift::Lift(pros::MotorGroup* motorGroup, pros::Rotation* rotationSensor)
    : motors(motorGroup),
      rotationSensor(rotationSensor),
      setpointValid(false),
      positionPID(30.0, 0.0, 0.0) {
  motors->set_encoder_units(pros::MotorEncoderUnits::degrees);
  motors->tare_position();
};

void Lift::periodic() {
  double angleRots = motors->get_position();

  motors->move_absolute((int32_t)(heightToRotation(setpoint) * 360), 100);
}

void Lift::setSetpointPosition(inch setpoint) {
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

}  // namespace Lift
