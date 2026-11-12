#include "Drivetrain.h"

#include <math.h>

#include <numbers>
#include <type_traits>

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
                       pros::Rotation* xOdomRotationSensor, pros::IMU* Imu,
                       double gearRatio, double wheelDiameter,
                       double odomWheelDiameterInches)
    : m_drivePID(145, 0.000, 0.00, 20),
      m_turnPID(63, 0.0, 0.00, 20),
      m_leftMotors(leftMotors),
      m_rightMotors(rightMotors),
      m_xOdomRotationSensor(xOdomRotationSensor),
      m_imu(Imu),
      m_gearRatio(gearRatio),
      m_wheelDiameter(wheelDiameter),
      odomWheelDiameterInches(odomWheelDiameterInches) {
  leftMotors->set_brake_mode_all(
      pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
  rightMotors->set_brake_mode_all(
      pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
}

void Drivetrain::periodic() {
  // Eigen::VectorXd z = Eigen::VectorXd::Zero(NZ);
  // getNewStateForFilter(z);
  // poseFilter.update(z);

  // Eigen::VectorXd state = poseFilter.state();
  double rate = m_imu->get_gyro_rate().z;
  if (!isinf(rate)) {
    headingRads -= degsToRads(rate) * DT_S;
  }

  double vF = m_xOdomRotationSensor->get_velocity() / 3600.0 * M_PI *
              inchesToMeters(odomWheelDiameterInches);

  poseEstXM += vF * std::cos(headingRads);
  poseEstYM += vF * std::sin(headingRads);

  const double x_m = poseEstXM;
  const double y_m = poseEstYM;

  std::cout << "x=" << x_m << " y=" << y_m
            << " th=" << wrapPi(headingRads) * 180.0 / M_PI << std::endl;

  // handle the current state
  std::visit(
      [&](const auto& s) {
        using T = typename std::decay<decltype(s)>::type;
        if constexpr (std::is_same_v<T, States::Idle>) {
          std::cout << "Idle\n";
          driveVoltage(0, 0);
        } else if constexpr (std::is_same_v<T, States::DriveVoltage>) {
          std::cout << "Drive Voltage\n";
          driveVoltage(s.leftVoltage(), s.rightVoltage());
        } else if constexpr (std::is_same_v<T, States::DriveDirection>) {
          // we use cos compensation to make sure that we don't drive much when
          // pointed in the wrong direction.
          // see
          // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#cosine-compensation
          // for an example of what this means
          std::cout << "drive direction\n";

          double err = wrapPi(degsToRads(s.headingDegs) - headingRads);
          double distTraveled = std::hypot(x_m, y_m);

          // this solves some issues of what we do at something like being at
          // 5pi/6 and target heading of -5pi/6. This lets us not flip 2pi, but
          // just the pi/3 we need to spin
          m_turnPID.setTarget(0);

          double turn = m_turnPID.calculate(err);

          m_drivePID.setTarget(inchesToMeters(s.distIn));

          double drive = m_drivePID.calculate(distTraveled);

          std::cout << distTraveled << ' ' << inchesToMeters(s.distIn) << '\n';

          drive *= std::cos(err);

          driveVoltage(drive - turn, drive + turn);

          if (std::abs(distTraveled - inchesToMeters(s.distIn)) <= 0.1 &&
              std::abs(err) <= M_PI / 4.0) {
            setState(States::Idle{});
          }
        } else if constexpr (std::is_same_v<T, States::TurnHeading>) {
          double err = wrapPi(degsToRads(s.desiredHeadingDegs) - headingRads);
          std::cout << "Turn Heading " << err << ' ' << M_PI / 16 << '\n';
          m_turnPID.setTarget(0);
          double calc = m_turnPID.calculate(err);
          driveVoltage(-calc, calc);
          if (std::abs(err) <= M_PI / 4) {
            setState(States::Idle{});
          }
        } else {
          // unreachable
          std::cout << "Bad\n";
        }
      },
      currState);
}

void Drivetrain::setState(States::State s) {
  // if (std::holds_alternative<States::DriveDirection>(s) &&
  //     !std::holds_alternative<States::DriveDirection>(currState)) {
  //   m_drivePID.reset();
  //   m_turnPID.reset();
  //   poseReset();
  // } else if (std::holds_alternative<States::TurnHeading>(s) &&
  //            !std::holds_alternative<States::DriveDirection>(currState)) {
  //   m_turnPID.reset();
  //   poseReset();
  // }
  currState = std::move(s);
}

void Drivetrain::initPose(double x_meters, double y_meters, double theta_degs) {
  // Eigen::VectorXd x0 = Eigen::VectorXd::Zero(NX);
  // x0 << x_meters, y_meters, degsToRads(theta_degs), 0.0, 0.0, 0.0;
  // poseFilter.init(0.0, x0);
  poseEstXM = x_meters;
  poseEstYM = y_meters;
  headingRads = degsToRads(theta_degs);
}

void Drivetrain::driveVoltage(int32_t leftVoltage, int32_t rightVoltage) {
  m_leftMotors->move(leftVoltage);
  m_rightMotors->move(rightVoltage);
}

void Drivetrain::poseReset() {
  initPose(0, 0, 0);
  // Eigen::MatrixXd x0 = Eigen::VectorXd::Zero(NX);
  // auto currPose = poseFilter.state();
  // x0 << 0.0, 0.0, degsToRads(m_imu->get_heading());

  // poseFilter.init();
}

void Drivetrain::resetIMU() { m_imu->reset(); }

bool Drivetrain::isIdle() {
  return std::holds_alternative<States::Idle>(currState);
}

double Drivetrain::degsToRads(double degs) { return degs * M_PI / 180; }

double Drivetrain::wrapPi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

double Drivetrain::dist(double x1, double y1, double x2, double y2) {
  return hypot(x2 - x1, y2 - y1);
}

double Drivetrain::inchesToMeters(double in) { return in * 0.0254; }

}  // namespace Drivetrain