/**
 * @file Drivetrain.h
 * @brief Drivetrain control system for VEX robot movement and turning
 *
 * This module provides closed-loop control for robot drivetrain movement,
 * including distance driving and heading-based turning using PID controllers.
 */

#pragma once

// #define DEBUG
#include <math.h>

#include <chrono>
#include <functional>
#include <variant>

#include "PID.h"
#include "TaskBase.h"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

/// Debug print macro - outputs to console if DEBUG is defined
#ifdef DEBUG
#define PRINT_D(out) std::cout << out << "\n"
#else
#define PRINT_D(out)
#endif

namespace Drivetrain {

namespace States {
struct Idle {};

struct DriveVoltage {
  std::function<int32_t()> leftVoltage;
  std::function<int32_t()> rightVoltage;
};

struct DrivePose {
  double xPoseIn;
  double yPoseIn;
  double headingDegs;
};

struct DriveDistance {
  double distancIn;
  double accumulatedDistIn;
};

// struct TurnHeading {
//   double desiredHeadingDegs;
// };

using State = std::variant<Idle, DriveVoltage, DrivePose,
                           DriveDistance>;  //, TurnHeading>;
}  // namespace States

/**
 * @class Drivetrain
 * @brief Manages differential drive movement with PID-based position and
 * heading control
 *
 * This class encapsulates drivetrain functionality including:
 * - Closed-loop distance driving with tolerance and settling time
 * - Closed-loop rotation/heading control via IMU
 * - Voltage capping for safety and control
 * - Configurable PID tuning for both drive and turn operations
 */
class Drivetrain {
 public:
  /**
   * @brief Constructs a Drivetrain instance with motor and sensor configuration
   * @param leftMotors Motor group controlling the left side of the drivetrain
   * @param rightMotors Motor group controlling the right side of the drivetrain
   * @param rotationSensor Rotation sensor for tracking relative wheel rotation
   * @param imu Inertial Measurement Unit for heading/gyro tracking
   * @param gearRatio Mechanical gear ratio (motor revolutions per wheel
   * revolution), default 1.0
   * @param wheelDiameter Wheel diameter in inches, default 2.25 inches
   */
  Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors,
             pros::Rotation* xOdomRotationSensor, pros::Imu* imu,
             double gearRatio = 1.0, double wheelDiameter = 2.25,
             double odomWheelDiameterInches = 2.0);

  void periodic();

  void setState(States::State s);

  /**
   * @brief Initializes drivetrain components (call once in init function)
   *
   * Resets the IMU to establish the initial heading reference.
   */
  void init() { m_imu->reset(true); };

  /**
   * @brief Applies a voltage cap to limit maximum motor output
   *
   * @param voltageCap Maximum voltage magnitude for motor commands, range
   * [-127, 127] where 127 is full forward and -127 is full reverse
   */
  void capVoltage(int32_t voltageCap) { m_voltageCap = voltageCap; };

  /**
   * @brief Configures the PID tuning parameters for distance driving control
   *
   * @param kp Proportional gain coefficient
   * @param ki Integral gain coefficient
   * @param kd Derivative gain coefficient
   * @param integralLim Integral accumulation limit for anti-windup protection
   */
  void setDrivePID(double kp, double ki, double kd, double integralLim) {
    m_drivePID.setTunings(kp, ki, kd);
    m_drivePID.setIntegralLimit(integralLim);
  };

  /**
   * @brief Configures the PID tuning parameters for heading/turn control
   *
   * @param kp Proportional gain coefficient
   * @param ki Integral gain coefficient
   * @param kd Derivative gain coefficient
   * @param integralLim Integral accumulation limit for anti-windup protection
   */
  void setTurnPID(double kp, double ki, double kd, double integralLim) {
    m_turnPID.setTunings(kp, ki, kd);
    m_turnPID.setIntegralLimit(integralLim);
  };

 private:
  void driveVoltage(int32_t leftVoltage, int32_t rightVoltage);

  // ==================== Configuration ====================

  States::State currState = States::Idle();

  /** Maximum voltage magnitude for motor output, range [-127, 127] */
  int32_t m_voltageCap = 127;

  // ==================== Motor and Sensor Pointers ====================

  /** Pointer to left motor group controlling left side of drivetrain */
  pros::MotorGroup* m_leftMotors;

  /** Pointer to right motor group controlling right side of drivetrain */
  pros::MotorGroup* m_rightMotors;

  /** Pointer to rotation sensor for tracking relative wheel position */
  double odomWheelDiameterInches = 2.0;
  pros::Rotation* m_xOdomRotationSensor;

  /** Pointer to inertial measurement unit (IMU) for heading/gyro feedback */
  pros::Imu* m_imu;

  // ==================== PID Controllers ====================

  /** PID controller for closed-loop distance driving control */
  Mines::PID m_drivePID;

  /** PID controller for closed-loop heading/turn control */
  Mines::PID m_turnPID;

  // ==================== Mechanical Parameters ====================

  /** Wheel diameter in inches - used to convert rotations to linear distance */
  double m_wheelDiameter;

  /** Gear ratio (motor revolutions per wheel revolution) - accounts for gearing
   * between motor and wheel */
  double m_gearRatio;

  // odometry
  double poseEstXIn = 0.0;
  double poseEstYIn = 0.0;
  double poseAcc = 0.0;
  double headingDegs = 0.0;
  std::chrono::time_point<std::chrono::high_resolution_clock> prevPeriodicCall =
      std::chrono::high_resolution_clock::now();

  static double degsToRads(double degs) { return degs / 180.0 * M_PI; }
};
}  // namespace Drivetrain