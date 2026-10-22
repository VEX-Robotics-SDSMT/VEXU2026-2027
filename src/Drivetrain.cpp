#include "Drivetrain.h"

#include <math.h>

#include <numbers>

#include "main.h"

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
                       pros::v5::GPS* reverseGPS, pros::v5::GPS* leftGPS,
                       double gearRatio, double wheelDiameter,
                       double odomWheelDiameterInches)
    : m_drivePID(9, .001, .05, 20),
      m_turnPID(5, .001, .5, 20),
      m_leftMotors(leftMotors),
      m_rightMotors(rightMotors),
      m_xOdomRotationSensor(xOdomRotationSensor),
      m_imu(imu),
      m_reverseGPS(reverseGPS),
      m_leftGPS(leftGPS),
      m_gearRatio(gearRatio),
      m_wheelDiameter(wheelDiameter),
      odomWheelDiameterInches(odomWheelDiameterInches) {
  poseFilter = KalmanFilter(DT_S, )
}

void Drivetrain::periodic() {
  headingDegs = m_imu->get_yaw();
  std::chrono::duration dt =
      std::chrono::high_resolution_clock::now() - prevPeriodicCall;

  double dt_s = dt.count() * powf(10, -9);

  double change_in =
      (m_xOdomRotationSensor->get_velocity() / 360.0 * m_wheelDiameter * M_PI) *
      dt_s;
  poseAcc += change_in;

  poseEstXIn += change_in * cosf(degsToRads(headingDegs));
  poseEstYIn += change_in * sinf(degsToRads(headingDegs));

  std::cout << "Drive Periodic " << dt_s << ' ' << change_in << ' '
            << headingDegs << "degs " << poseAcc << ' ' << poseEstXIn << ' '
            << poseEstYIn << '\n';

  // handle the current state
  std::visit(
      [this](const auto& s) {
        using T = std::decay_t<decltype(s)>;
        if constexpr (std::is_same_v<T, States::Idle>) {
          driveVoltage(0, 0);
        } else if constexpr (std::is_same_v<T, States::DriveVoltage>) {
          driveVoltage(s.leftVoltage(), s.rightVoltage());
        } else if constexpr (std::is_same_v<T, States::DrivePose>) {
          std::cout << "drive pose\n";
          // TODO: not implemented
        } else if constexpr (std::is_same_v<T, States::DriveDistance>) {
          std::cout << "drive distance\n";
        } else if constexpr (std::is_same_v<T, States::TurnHeading>) {
          m_turnPID.setTarget(s.desiredHeadingDegs);
          double calc = m_turnPID.calculate(headingDegs);
          driveVoltage(-calc, calc);
        } else {
          std::cout << "Bad\n";
        }
      },
      currState);

  prevPeriodicCall = std::chrono::high_resolution_clock::now();
}

void Drivetrain::setState(States::State s) {
  using newT = std::decay_t<decltype(s)>;
  using curT = std::decay_t<decltype(currState)>;

  if (std::is_same_v<newT, States::DrivePose>) {
    m_drivePID.reset();
    m_turnPID.reset();
  } else if (std::is_same_v<newT, States::TurnHeading>) {
    m_turnPID.reset();
  }
  currState = std::move(s);
}

void Drivetrain::driveVoltage(int32_t leftVoltage, int32_t rightVoltage) {
  m_leftMotors->move(leftVoltage);
  m_rightMotors->move(rightVoltage);
}

double Drivetrain::degsToRads(double degs) { return degs * M_PI / 180; }

}  // namespace Drivetrain