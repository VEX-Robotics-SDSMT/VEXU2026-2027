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
                       pros::Rotation* xOdomRotationSensor, pros::Imu* imu,
                       double gearRatio, double wheelDiameter,
                       double odomWheelDiameterInches)
    : m_drivePID(120, 0.000, 0.00, 20),
      m_turnPID(75, 0.0, 0.00, 20),
      m_leftMotors(leftMotors),
      m_rightMotors(rightMotors),
      m_xOdomRotationSensor(xOdomRotationSensor),
      m_imu(imu),
      m_gearRatio(gearRatio),
      m_wheelDiameter(wheelDiameter),
      odomWheelDiameterInches(odomWheelDiameterInches) {
  leftMotors->set_brake_mode_all(
      pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
  rightMotors->set_brake_mode_all(
      pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
  return;
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(NX, NX);
  A(0, 3) = DT_S;  // x += vx * dt
  A(1, 4) = DT_S;  // y += vy * dt
  A(2, 5) = DT_S;  // theta += omega * dt
  /*
  A =
  1 0 0 DT0 0
  0 1 0 0 DT0
  0 0 1 0 0 DT
  0 0 0 1 0 0
  0 0 0 0 1 0
  0 0 0 0 0 1
  */
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(NZ, NX);
  C(0, 3) = 1;
  C(1, 4) = 1;
  C(2, 5) = 1;
  /*
  C=
  1 0 0 0 0 0
  0 1 0 0 0 0
  0 0 1 0 0 0
  */

  // ---- Q: process noise (discrete white-noise acceleration) ----
  const double SIG_A_LIN = 2.0;   // m/s^2, unmodeled linear accel (tune)
  const double SIG_A_ANG = 10.0;  // rad/s^2, unmodeled angular accel (tune)

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(NX, NX);
  const double dt2 = DT_S * DT_S;
  const double dt3 = dt2 * DT_S;
  const double dt4 = dt2 * dt2;
  auto qBlock = [&](int p, int v, double sig) {
    const double s2 = sig * sig;
    Q(p, p) = s2 * dt4 / 4.0;
    Q(p, v) = Q(v, p) = s2 * dt3 / 2.0;
    Q(v, v) = s2 * dt2;
  };
  qBlock(0, 3, SIG_A_LIN);  // x / vx
  qBlock(1, 4, SIG_A_LIN);  // y / vy
  qBlock(2, 5, SIG_A_ANG);  // theta / omega

  // ---- R: measurement noise (diagonal; TODO: replace with measured variances)
  const double odoV = 0.5;  // (m/s)^2
  const double odoW = 0.5;  //* 0.10;  // (rad/s)^2

  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(NZ, NZ);
  R.diagonal() << odoV, odoV, odoW;  // odometry

  // ---- P: initial estimate error covariance ----
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(NX, NX);
  P.diagonal() << 0.01, 0.01, 0.01, 0.01, 0.01,
      0.01;  // pose unknown at startup;
             // use ~0.01 for x,y,theta if you set the start pose precisely

  poseFilter = KalmanFilter(DT_S, A, C, Q, R, P);

  Eigen::MatrixXd x0 = Eigen::VectorXd::Zero(NX);

  poseFilter.init(0.0, x0);
}

void Drivetrain::periodic() {
  // Eigen::VectorXd z = Eigen::VectorXd::Zero(NZ);
  // getNewStateForFilter(z);
  // poseFilter.update(z);

  // Eigen::VectorXd state = poseFilter.state();
  const double headingRads = degsToRads(m_imu->get_yaw()) + headingOffRads;

  const double vF = m_xOdomRotationSensor->get_velocity() / 36000.0 * M_PI *
                    inchesToMeters(odomWheelDiameterInches);

  poseEstXM += vF * std::cos(headingRads);
  poseEstYM += vF * std::sin(headingRads);

  const double x_m = poseEstXM;
  const double y_m = poseEstYM;

  // std::cout << '\n'
  //           << "Drive Periodic.\n\tx: " << x_m << "\n\ty: " <<
  //           y_m
  //           << "\n\tt: " << headingRads * 180.0 / M_PI;
  std::cout << "x=" << x_m << " y=" << y_m
            << " th=" << wrapPi(headingRads) * 180.0 / M_PI << std::endl;

  // handle the current state
  std::visit(
      [&](const auto& s) {
        using T = typename std::decay<decltype(s)>::type;
        if constexpr (std::is_same_v<T, States::Idle>) {
          std::cout << "Idle\n";
          driveVoltage(0, 0);
        } else if constexpr (std::is_same<T, States::DriveVoltage>::value) {
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
          m_turnPID.setTarget(headingRads + err);

          double turn = m_turnPID.calculate(headingRads);

          m_drivePID.setTarget(inchesToMeters(s.distIn));

          double drive = m_drivePID.calculate(distTraveled);

          drive *= std::cos(err);

          driveVoltage(drive - turn, drive + turn);

          if (distTraveled - inchesToMeters(s.distIn) <= 0.02 &&
              std::abs(err) <= M_PI / 16.0) {
            setState(States::Idle{});
          }
        } else if constexpr (std::is_same_v<T, States::DriveDistance>) {
          std::cout << "drive distance\n";
        } else if constexpr (std::is_same_v<T, States::TurnHeading>) {
          std::cout << "Turn Heading\n";
          double target = wrapPi(degsToRads(s.desiredHeadingDegs));
          m_turnPID.setTarget(target);
          double calc = m_turnPID.calculate(headingRads);
          driveVoltage(-calc, calc);
          if (std::abs(target - headingRads) <= M_PI / 16) {
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
  using newT = typename std::decay<decltype(s)>::type;
  using curT = typename std::decay<decltype(currState)>::type;

  if (std::is_same<newT, States::DriveDirection>::value) {
    m_drivePID.reset();
    m_turnPID.reset();
    poseReset();
  } else if (std::is_same<newT, States::TurnHeading>::value) {
    m_turnPID.reset();
  }
  currState = std::move(s);
}

void Drivetrain::initPose(double x_meters, double y_meters, double theta_degs) {
  // Eigen::VectorXd x0 = Eigen::VectorXd::Zero(NX);
  // x0 << x_meters, y_meters, degsToRads(theta_degs), 0.0, 0.0, 0.0;
  // poseFilter.init(0.0, x0);
  poseEstXM = x_meters;
  poseEstYM = y_meters;
  headingOffRads += degsToRads(theta_degs);
}

void Drivetrain::driveVoltage(int32_t leftVoltage, int32_t rightVoltage) {
  m_leftMotors->move(leftVoltage);
  m_rightMotors->move(rightVoltage);
}

void Drivetrain::getNewStateForFilter(Eigen::VectorXd& z) {
  return;
  Eigen::VectorXd xhat = poseFilter.state();
  if (xhat.size() != NX) {
    std::cout << "filter state size " << xhat.size() << '\n';
    return;
  }
  double thetaPrev = xhat(2);

  double vF = degsToRads(m_xOdomRotationSensor->get_velocity()) *
              inchesToMeters(odomWheelDiameterInches / 2.0);
  double vL = 0.0;  // we don't move laterally
  double w = -m_imu->get_gyro_rate().z * M_PI /
             180.0;  // same sign convention as theta
  if (isnan(w) || isinf(w)) {
    return;
  }

  std::cout << w << '\n';

  // Eigen::VectorXd z(NZ);
  z << vF * std::cos(thetaPrev) - vL * std::sin(thetaPrev),
      vF * std::sin(thetaPrev) + vL * std::cos(thetaPrev), w;

  /*old - uses gps sensors
  Eigen::VectorXd xHat = poseFilter.state();
  double thetaPrev = xHat(2);

  double s1_x = m_leftGPS->get_position_x();
  double s1_y = m_leftGPS->get_position_y();

  double s2_x = m_reverseGPS->get_position_x();
  double s2_y = m_reverseGPS->get_position_y();

  std::cout << "gps done\n";
  double th1, th2;
  // if (m_leftGPS->is_installed()) {
  //   th1 = degsToRads(m_leftGPS->get_heading() - 180.0);  // wrap to [-pi, pi]
  //   th2 = degsToRads(m_reverseGPS->get_heading() - 180.0);
  // } else {
  th1 = 0.0;
  th2 = 0.0;
  // }

  th1 = thetaPrev + wrapPi(th1 - thetaPrev);
  th2 = thetaPrev + wrapPi(th2 - thetaPrev);

  std::cout << "th12 done\n";
  double change_in =
      (m_xOdomRotationSensor->get_velocity() / 360.0 * m_wheelDiameter * M_PI) *
      DT_S;
  std::cout << "change_in done\n";

  double vx = change_in * cosf(thetaPrev);
  double vy = change_in * sinf(thetaPrev);

  std::cout << "trig done\n";

  double w = m_imu->get_gyro_rate().x;
  std::cout << "omega done\n";

  z << s1_x, s1_y, th1, s2_x, s2_y, th2, vx, vy, w;
  */
}

void Drivetrain::poseReset() {
  poseEstXIn = 0.0;
  poseEstYIn = 0.0;
  m_imu->tare();
  // Eigen::MatrixXd x0 = Eigen::VectorXd::Zero(NX);
  // auto currPose = poseFilter.state();
  // x0 << 0.0, 0.0, degsToRads(m_imu->get_heading());

  // poseFilter.init();
}

bool Drivetrain::isIdle() {
  return std::is_same_v<std::decay<decltype(currState)>, States::Idle>;
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