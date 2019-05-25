#ifndef __MOTIONPROFILE_H__
#define __MOTIONPROFILE_H__
#include "main.h"
#include <cstdint>

class MotionProfile
{
public:
    MotionProfile(float maxAccel, float vCruise);
    void generate(int32_t distance);
    void resetTime();
    void resetAll();
    float update(int dt);
//private:
    float m_maxAccel, m_vCruise; // Constant
    float m_totalTime;
    float m_vMax;
    float m_tAccel, m_tCruise, m_tDecel;
    float m_currTime;
};

#endif //__MOTIONPROFILE_H__