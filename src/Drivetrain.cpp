#include "Drivetrain.h"

#include <math.h>

#include <numbers>

namespace Drivetrain {
//=======================================
// Member Variables
//=======================================

// pros::MotorGroup m_leftMotors;
// pros::MotorGroup m_rightMotors;
// Mines::PID m_drivePID;
// Mines::PID m_turnPID;

Drivetrain::Drivetrain(pros::MotorGroup* leftMotors,
                       pros::MotorGroup* rightMotors,
                       pros::Rotation* xOdomRotationSensor, pros::Imu* imu,
                       double gearRatio, double wheelDiameter,
                       double odomWheelDiameterInches)
    : m_drivePID(9, .001, .05, 20),
      m_turnPID(5, .001, .5, 20),
      m_leftMotors(leftMotors),
      m_rightMotors(rightMotors),
      m_xOdomRotationSensor(xOdomRotationSensor),
      m_imu(imu),
      m_gearRatio(gearRatio),
      m_wheelDiameter(wheelDiameter),
      odomWheelDiameterInches(odomWheelDiameterInches) {}

void Drivetrain::periodic() {
  headingDegs = m_imu->get_yaw();
  std::chrono::duration dt =
      std::chrono::high_resolution_clock::now() - prevPeriodicCall;

  double dt_s = dt.count() * powf(10, -9);

  double change_in = m_xOdomRotationSensor->get_velocity() * dt_s;

  poseEstXIn += change_in * cosf(headingDegs * M_PI / 180.0);
  poseEstYIn += change_in * sinf(headingDegs * M_PI / 180.0);

  // handle the current state
  std::visit(
      [](const auto& s) {
        using T = std::decay_t<decltype(currState)>;
        if constexpr (std::is_same_v<T, States::Idle>) {
          driveVoltage(0, 0);
        } else if constexpr (std::is_same_v<T, States::DriveVoltage>) {
          driveVoltage(s.leftVoltage(), s.rightVoltage());
        } else if constexpr (std::is_same_v<T, States::DrivePose>) {
          // TODO: not implemented
        } else if constexpr (std::is_same_v<T, States::DriveDistance>) {
        } else if constexpr (std::is_same_v<T, States::TurnHeading>) {
          m_turnPID.setTarget(s.desiredHeadingDegs);
          double calc = m_turnPID.calculate(headingDegs);
          driveVoltage(-calc, calc);
        }
      },
      currState);

  prevPeriodicCall = std::chrono::high_resolution_clock::now();
}

void Drivetrain::setState(States::State s) {
  using newT = std::decay_t<decltype(s)>;
  using curT = std::decay_t<decltype(currState)>;

  if (std::is_same_v<newT, States::DrivePose> && s != currState) {
    m_drivePID.reset();
    m_turnPID.reset();
  } else if (std::is_same_v<newT, States::TurnHeading> && s != currState) {
    m_turnPID.reset();
  }
  currState = std::move(s);
}

void Drivetrain::driveVoltage(int32_t leftVoltage, int32_t rightVoltage) {
  m_leftMotors->move(leftVoltage);
  m_rightMotors->move(rightVoltage);
}

}  // namespace Drivetrain