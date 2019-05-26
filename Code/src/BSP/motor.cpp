#include "Motor.hpp"
#include "tim.h"

Motor::Motor(TIM_HandleTypeDef* timeHandle, bool flipped)
 : m_timeHandle(timeHandle), m_flipped(flipped) {
	HAL_TIM_Base_Start(m_timeHandle);
}


void Motor::setSpeed (int speed) {
	speed = m_flipped ? speed*-4096/1000 : speed*4096/1000;
	if (speed > 0 ) {
		setPWM(m_timeHandle, TIM_CHANNEL_2, 4095, 0);
		setPWM(m_timeHandle, TIM_CHANNEL_1, 4095, speed);		
	} else {
		setPWM(m_timeHandle, TIM_CHANNEL_1, 4095, 0);
		setPWM(m_timeHandle, TIM_CHANNEL_2, 4095, -1*speed);
	}
	
}