#include "PID.h"
#include <algorithm>
#include <iostream>

namespace Mines
{

PID::PID(double kp, double ki, double kd, double integralLimit)
    : m_kp(kp), m_ki(ki), m_kd(kd), m_integralLimit(integralLimit), m_previousError(0.0), m_integral(0.0), m_target(0.0)
{

}

void PID::setTarget(double target)
{
    m_target = target;
    reset();
}

void PID::reset()
{
    m_previousError = 0.0;
    m_integral = 0.0;
}

double PID::calculate(double measurement)
{
    double error = m_target - measurement;

    m_integral += error;

    double derivative = error - m_previousError;

    if(m_integral > m_integralLimit)
    {
        m_integral = m_integralLimit;
    }
    if(m_integral < -m_integralLimit)
    {
        m_integral = -m_integralLimit;
    }
    
    m_previousError = error;
    
    return (m_kp * error) + (m_ki * m_integral) + (m_kd * derivative);
}

void PID::setTunings(double kp, double ki, double kd)
{
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}

void PID::setIntegralLimit(double limit)
{
    m_integralLimit = limit;
}

}