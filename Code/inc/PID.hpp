#ifndef __PID_H__
#define __PID_H__

class PID
{
public:    
    PID(float Kp, float Ki, float Kd);
    void setTarget(float target);
    float update(float signal);
private:
    float m_Kp, m_Ki, m_Kd;
    float m_error, m_prevError, m_cumError;
    float m_target;
};

#endif