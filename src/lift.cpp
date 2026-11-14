#include "lift.h"

#include <iostream>

namespace Lift {
Lift::Lift(pros::MotorGroup* motorGroup, pros::Rotation* rotationSensor)
    : motors(motorGroup),
      rotationSensor(rotationSensor),
      setpointValid(false),
      positionPID(25.0, 0.0, 0.0) {
motors->set_brake_mode_all(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD);
};

void Lift::periodic(int32_t volts) {
  double angleRots = motors->get_position();

  positionPID.setTarget(heightToRotation(this->setpoint));
  double out = positionPID.calculate(angleRots);

  if (std::abs(volts) < 10) {
    volts = 0;
  }
  motors->move(volts);
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