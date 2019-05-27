#ifndef __BUZZER_HPP__
#define __BUZZER_HPP__

#include "main.h"
#include "tim.h"
#include <cmath>

class Buzzer
{
public:
    Buzzer(TIM_HandleTypeDef* timeHandle, uint32_t channel)
     : m_timeHandle(timeHandle), m_channel(channel)
    {
        HAL_TIM_Base_Start(m_timeHandle);

        //Initialize midi array
        //http://subsynth.sourceforge.net/midinote2freq.html
        int a = 440; // a is 440 hz
        for (int x = 0; x < 127; ++x)
        {
            m_midi[x] = (a) * pow(2.0,(x - 69.0) / 12.0);
        }
    }

    void playFreq(int32_t freq)
    {
        setPWM(m_timeHandle, m_channel, freq, freq/2);
        return;
    }

    void playMidiNote(int32_t note)
    {
        playFreq((int)m_midi[note]);
        return;
    }
private:
    TIM_HandleTypeDef* m_timeHandle;
    uint32_t m_channel;
    float m_midi[127];
};

#endif //__BUZZER_HPP__