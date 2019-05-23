#include "PID.hpp"

PID::PID(float Kp, float Ki, float Kd) :
m_Kp(Kp), m_Ki(Ki), m_Kd(Kd) 
{
    m_error = 0;
    m_prevError = 0;
    m_cumError = 0;
}

void PID::setTarget(float target)
{
    m_target = target;
}

float PID::update(float signal)
{
    m_prevError = m_error;
    m_error = m_target - signal;
    m_cumError += m_error;
    return m_Kp * m_error + m_Ki * m_cumError + m_Kd * (m_error-m_prevError);
}

void PID::resetError()
{
    m_cumError = 0;
    m_error = 0;
    m_prevError = 0;
}