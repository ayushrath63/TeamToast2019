#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"
#include "tim.h"
#include "stm32f4xx_hal.h"

class Motor {

public: 
	void setSpeed(int speed);
	Motor(TIM_HandleTypeDef timeHandle, bool flipped = false);


private:
	TIM_HandleTypeDef m_timeHandle;
	bool m_flipped;
};


#endif