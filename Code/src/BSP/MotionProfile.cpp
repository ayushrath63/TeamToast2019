#include "MotionProfile.hpp"

MotionProfile::MotionProfile(float maxAccel, float vCruise)
 : m_maxAccel(maxAccel), m_vCruise(vCruise), m_totalTime(0.0)
{}

void MotionProfile::generate(int32_t distance)
{ 
    m_totalTime = distance / m_vCruise;
    m_vMax = m_maxAccel * m_totalTime / 2;
    if(m_vCruise > m_vMax)
    {
        m_vMax = m_vCruise;
        m_currTime = 0.0;
        m_tCruise = 0.0;
        m_tAccel = m_totalTime / 2;
        m_tDecel = m_tAccel;
    } else {
        m_vMax = m_vCruise;
        m_tAccel = m_vMax / m_maxAccel;
        m_tDecel = m_tAccel;
        m_tCruise = m_totalTime - (m_tDecel + m_tAccel);
        m_currTime = 0.0;
    }

}

void MotionProfile::resetAll()
{ 
    m_tAccel = 0.0;
    m_tCruise = 0.0;
    m_tDecel = 0.0;
    m_totalTime = 0.0;
    m_currTime = 0.0;
}

void MotionProfile::resetTime()
{ 
    m_currTime = 0.0;
}

float MotionProfile::update(int dt)
{
    m_currTime += dt;
    if(m_currTime > m_totalTime)
    {
        return 0.0;
    } else if(m_currTime <= m_tAccel) {
        return m_maxAccel * m_currTime;
    } else if((m_currTime > m_tAccel) && (m_currTime < m_tAccel + m_tCruise)) {
        return m_vMax;
    } else {
        return m_vMax - (m_maxAccel*(m_currTime - (m_tAccel + m_tCruise)));
    }
}