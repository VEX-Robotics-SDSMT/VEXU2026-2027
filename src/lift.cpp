#include "lift.h"

#include <iostream>

#include "main.h"

namespace Lift {
Lift::Lift(pros::MotorGroup* motorGroup, pros::Rotation* rotationSensor)
    : motors(motorGroup),
      rotationSensor(rotationSensor),
      setpointValid(false),
      positionPID(150.0, 0.0, 0.0) {
  motors->set_brake_mode_all(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
  rotationSensor->set_reversed(true);
  rotationSensor->reset();
};

void Lift::periodic() {
  position += (rotationSensor->get_velocity() / 36000.0) *
              inchesHeightPerRotation * DT_S;

  std::visit(
      [&](const auto& s) {
        using T = typename std::decay<decltype(s)>::type;
        if constexpr (std::is_same_v<T, States::Idle>) {
          motors->move(0);
        } else if constexpr (std::is_same_v<T, States::Voltage>) {
          motors->move(s.voltage);
        } else if constexpr (std::is_same_v<T, States::VoltageCallable>) {
          motors->move(s.voltageCallable());
        } else if constexpr (std::is_same_v<T, States::Position>) {
          positionPID.setTarget(s.height);
          double out = positionPID.calculate(position);
          motors->move(out);
        }
      },

      currState);
}

void Lift::setState(States::State newState) { currState = std::move(newState); }

States::State Lift::getState() { return currState; }

inch Lift::getHeight() { return position; }

inch Lift::rotationToHeight(double rotations) {
  return rotations * inchesHeightPerRotation;
}

double Lift::heightToRotation(inch height) {
  return height / inchesHeightPerRotation;
}

}  // namespace Lift