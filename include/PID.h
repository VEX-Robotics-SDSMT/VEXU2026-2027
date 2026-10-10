#pragma once

namespace Mines
{

class PID
{
public:
    /**
     * @brief Constructs a PID controller with specified gains and integral limit.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     * @param integralLimit Maximum absolute value for the integral term.
     */
    PID(double kp, double ki, double kd, double integralLimit = 100.0);

    /**
     * @brief Sets the target setpoint for the PID controller.
     * @param target The desired target value.
     */
    void setTarget(double target);
    /**
     * @brief Calculates the PID output based on the current measurement.
     * @param measurement The current measured value.
     * @return The PID controller output.
     */
    double calculate(double measurement);

    /**
     * @brief Resets the PID controller's internal state.
     */
    void reset();

    /**
     * @brief Sets the PID tunings.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     */
    void setTunings(double kp, double ki, double kd);
    
    /**
     * @brief Sets the integral limit.
     * @param limit Maximum absolute value for the integral term.
     */
    void setIntegralLimit(double limit);
private:
    double m_kp;
    double m_ki;
    double m_kd;
    double m_previousError = 0.0;
    double m_integral;
    double m_target;
    double m_integralLimit;
};

}