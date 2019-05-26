#ifndef __IRSENSOR_H__
#define __IRSENEOR_H__
#pragma once

#include "main.h"
#include "adc.h"
#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "usart.h"



// When there is no wall aroudn the mouse 
/*
NOwall: Right = 2000
Left = 3100 
Front = 1900
*/
// Values when the mouse is in the center of the cell 
constexpr float WALL_R = 3092; // 2864 - 3269 
constexpr float WALL_L = 3735; // 3600 - 3834 
constexpr float WALL_F = 2710;  // 2600 - 3000 


// The value when mouse is on the bottom  of the cell, 
// if value small than this then there is an opening in the front
constexpr float OPEN_F = 2100;
// The value when mouse is on the left most of the cell, 
// if value small than this then there is an opening on the right 
constexpr float OPEN_R = 2700;
// The value when mouse is on the right most of the cell, 
// if value small than this then there is an opening on the left 
constexpr float OPEN_L = 3400;



// no wall F= 2000

class IRSensor {

public: 
	uint32_t read();
	IRSensor(ADC_HandleTypeDef* adcHandle, uint32_t channel, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, bool invert = false);
	uint32_t value();

private:
	ADC_HandleTypeDef* m_adcHandle;
	uint32_t m_channel;
	GPIO_TypeDef* m_GPIOx;
	uint16_t m_GPIO_Pin;
	bool m_invert;
	int m_val; 
};

extern IRSensor IRLeft, IRTopLeft, IRTopRight;

void IRSensor_readAll();
bool ifdetectedFrontWall();
bool ifdetectedRightWall(); 
bool ifdetectedLeftWall();
bool ifcentered ();




#endif