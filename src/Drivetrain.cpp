#include "Drivetrain.h"

#include <numbers>
#include <math.h>

namespace Mines{
//=======================================
// Member Variables
//=======================================

//pros::MotorGroup m_leftMotors;
//pros::MotorGroup m_rightMotors;
//Mines::PID m_drivePID;
//Mines::PID m_turnPID;


Drivetrain::Drivetrain(pros::AbstractMotor* leftMotors,
                  pros::AbstractMotor* rightMotors,
                  pros::Rotation* rotationSensor,
                  pros::Imu* imu,
                  double gearRatio, double wheelDiameter)
:
  m_drivePID(9, .001, .05, 20),
  m_turnPID(5, .001, .5, 20),
  m_gearRatio(gearRatio),
  m_wheelDiameter(wheelDiameter)
{
    m_leftMotors = leftMotors;
    m_rightMotors = rightMotors;
    m_rotation = rotationSensor;
    m_imu = imu;
}

void Drivetrain::driveDistance(double distance, int32_t tol, uint32_t settleTime)
{
    double distancePerTurn = (m_wheelDiameter * std::numbers::pi) / m_gearRatio;
    double target = m_rotation->get_position() + (((distance / distancePerTurn) * 360 )* 100); 

    m_drivePID.setTarget(target);

    uint32_t time = 0;
    while(time <= settleTime)
    {
        int32_t motorPower = std::clamp((int32_t)m_drivePID.calculate(static_cast<double>(m_rotation->get_position())),-m_voltageCap,m_voltageCap);
        m_leftMotors->move(motorPower);
        m_rightMotors->move(motorPower);

        if(m_rotation->get_position() <= target + tol && m_rotation->get_position() >= target - tol)
        {
            time += 5;
        }
        else
        {
            time = 0;
        }

        pros::delay(5);
    }
    m_leftMotors->move(0);
    m_rightMotors->move(0);
}

void Drivetrain::turnFor(double angle, double tol, uint32_t settleTime)
{
    // Relative turning based on current angle being 0.
    double target = m_imu->get_rotation() + angle;

    m_turnPID.setTarget(target);

    uint32_t time = 0;

    while(time <= settleTime)
    {
        int32_t motorPower = std::clamp((int32_t)m_turnPID.calculate(m_imu->get_rotation()),-m_voltageCap,m_voltageCap);
        m_leftMotors->move(motorPower);
        m_rightMotors->move(-motorPower);

        if(m_imu->get_rotation() <= target + tol && m_imu->get_rotation() >= target - tol)
        {
            time += 5;
        }
        else
        {
            time = 0;
        }

        pros::delay(5);
    }
}

void Drivetrain::turnTo(double angle, double tol, uint32_t settleTime)
{
    // Absolute turning based on starting angle being 0.
    // Allows for (hopefully) more accurate autonomous routes.

    double target = angle > 180 ? -angle: angle;
    m_turnPID.setTarget(target);

    uint32_t time = 0;

    while(time <= settleTime)
    {
        double angle = fabs(m_imu->get_heading()) - 180;

        int32_t motorPower = std::clamp((int32_t)m_turnPID.calculate(angle), -m_voltageCap, m_voltageCap);
        m_leftMotors->move(motorPower);
        m_rightMotors->move(-motorPower);

        if(angle <= target + tol && angle >= target - tol)
        {
            time += 5;
        }
        else
        {
            time = 0;
        }

        pros::delay(5);
    }
}

}