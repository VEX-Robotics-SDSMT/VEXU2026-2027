/**
 * @file Drivetrain.h
 * @brief Drivetrain control system for VEX robot movement and turning
 * 
 * This module provides closed-loop control for robot drivetrain movement,
 * including distance driving and heading-based turning using PID controllers.
 */

#pragma once

//#define DEBUG

#include "pros/motor_group.hpp"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "PID.h"
#include "TaskBase.h"

/// Debug print macro - outputs to console if DEBUG is defined
#ifdef DEBUG
#define PRINT_D(out) std::cout << out << "\n"
#else
#define PRINT_D(out)
#endif

namespace Mines
{
/**
 * @class Drivetrain
 * @brief Manages differential drive movement with PID-based position and heading control
 * 
 * This class encapsulates drivetrain functionality including:
 * - Closed-loop distance driving with tolerance and settling time
 * - Closed-loop rotation/heading control via IMU
 * - Voltage capping for safety and control
 * - Configurable PID tuning for both drive and turn operations
 */
class Drivetrain
{
    public:

        /**
         * @brief Constructs a Drivetrain instance with motor and sensor configuration
         * @param leftMotors Motor group controlling the left side of the drivetrain
         * @param rightMotors Motor group controlling the right side of the drivetrain
         * @param rotationSensor Rotation sensor for tracking relative wheel rotation
         * @param imu Inertial Measurement Unit for heading/gyro tracking
         * @param gearRatio Mechanical gear ratio (motor revolutions per wheel revolution), default 1.0
         * @param wheelDiameter Wheel diameter in inches, default 2.25 inches
         */
        Drivetrain(pros::AbstractMotor* leftMotors,
                  pros::AbstractMotor* rightMotors,
                  pros::Rotation* rotationSensor,
                  pros::Imu* imu,
                  double gearRatio = 1.0, double wheelDiameter = 2.25);

        /**
         * @brief Initializes drivetrain components (call once in init function)
         * 
         * Resets the IMU to establish the initial heading reference.
         */
        void init() { m_imu->reset(true);};

        /**
         * @brief Drives the robot forward/backward a specified distance using closed-loop PID control
         * 
         * Uses rotation sensor feedback to drive the robot a relative distance while maintaining
         * proper heading via IMU feedback.
         * 
         * @param distance Distance to travel in inches (positive = forward, negative = reverse)
         * @param tol Tolerance threshold for settling - motor stops when error is below this value (default 5000)
         * @param settleTime Minimum time in milliseconds that the robot must maintain
         *                    the target before movement is considered complete (default 200ms)
         */
        void driveDistance(double distance, int32_t tol = 5000, uint32_t settleTime = 200);

        /**
         * @brief Applies a voltage cap to limit maximum motor output
         * 
         * @param voltageCap Maximum voltage magnitude for motor commands, range [-127, 127]
         *                    where 127 is full forward and -127 is full reverse
         */
        void capVoltage(int32_t voltageCap) {m_voltageCap = voltageCap;};


        /**
         * @brief Turns the robot relative to its current heading using closed-loop PID control
         * 
         * Performs an in-place rotation using IMU feedback to maintain the specified heading.
         * 
         * @param angle Amount of rotation in degrees (positive = counterclockwise, negative = clockwise)
         * @param tol Tolerance threshold in degrees for settling (default 1.0 degree)
         * @param settleTime Minimum time in milliseconds that the robot must maintain
         *                    the target heading before rotation is considered complete (default 200ms)
         */
        void turnFor(double angle, double tol = 1.0, uint32_t settleTime = 200);

        /**
         * @brief Turns the robot to an absolute heading using closed-loop PID control
         * 
         * Rotates the robot to face a specific absolute heading (0-360 degrees) using IMU feedback.
         * This differs from turnFor in that it targets an absolute heading rather than a relative rotation.
         * 
         * @param angle Target absolute heading in degrees (0-360)
         * @param tol Tolerance threshold in degrees for settling (default 1.0 degree)
         * @param settleTime Minimum time in milliseconds that the robot must maintain
         *                    the target heading before rotation is considered complete (default 200ms)
         */
        void turnTo(double angle, double tol = 1.0, uint32_t settleTime = 200);

        /**
         * @brief Configures the PID tuning parameters for distance driving control
         * 
         * @param kp Proportional gain coefficient
         * @param ki Integral gain coefficient
         * @param kd Derivative gain coefficient
         * @param integralLim Integral accumulation limit for anti-windup protection
         */
        void setDrivePID(double kp, double ki, double kd, double integralLim) { m_drivePID.setTunings(kp,ki,kd); m_drivePID.setIntegralLimit(integralLim);};

        /**
         * @brief Configures the PID tuning parameters for heading/turn control
         * 
         * @param kp Proportional gain coefficient
         * @param ki Integral gain coefficient
         * @param kd Derivative gain coefficient
         * @param integralLim Integral accumulation limit for anti-windup protection
         */
        void setTurnPID(double kp, double ki, double kd, double integralLim) { m_turnPID.setTunings(kp,ki,kd); m_turnPID.setIntegralLimit(integralLim); };

    private:
        // ==================== Configuration ====================
        
        /** Maximum voltage magnitude for motor output, range [-127, 127] */
        int32_t m_voltageCap = 127;

        // ==================== Motor and Sensor Pointers ====================
        
        /** Pointer to left motor group controlling left side of drivetrain */
        pros::AbstractMotor* m_leftMotors;
        
        /** Pointer to right motor group controlling right side of drivetrain */
        pros::AbstractMotor* m_rightMotors;
        
        /** Pointer to rotation sensor for tracking relative wheel position */
        pros::Rotation* m_rotation;
        
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
        
        /** Gear ratio (motor revolutions per wheel revolution) - accounts for gearing between motor and wheel */
        double m_gearRatio;

};
} // namespace Mines