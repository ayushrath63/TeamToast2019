#ifndef __IRSENSOR_H__
#define __IRSENSOR_H__
#pragma once

#include "main.h"
#include "adc.h"
#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "usart.h"



// When there is NO WALL aroudn the mouse 
/*
Front = 400
Left = 1206
Right = 120

*/

// Values when the mouse is in the center of the cell 


constexpr int WALL_F = 1700;  // 450 - 3050
constexpr int WALL_L = 2116; // 1546 - 3890
constexpr int WALL_R = 1066; // 528 - 3834 


// The value when mouse is on the bottom  of the cell, 
// if value small than this then there is an opening in the front
constexpr int OPEN_F = 1100;

// The value when mouse is on the right most of the cell, 
// if value small than this then there is an opening on the left 
constexpr int OPEN_L = 1400;

// The value when mouse is on the left most of the cell, 
// if value small than this then there is an opening on the right 
constexpr int OPEN_R = 400;


// no wall F= 2000

class IRSensor {

public: 
	int read() volatile;
	IRSensor(ADC_HandleTypeDef* adcHandle, uint32_t channel, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
	int value() volatile;

private:
	ADC_HandleTypeDef* m_adcHandle;
	uint32_t m_channel;
	GPIO_TypeDef* m_GPIOx;
	uint16_t m_GPIO_Pin;
	volatile int m_val; 
};

extern IRSensor IRLeft, IRTopLeft, IRTopRight;

void IRSensor_readAll();
bool ifdetectedFrontWall();
bool ifdetectedRightWall();
bool ifdetectedLeftWall();
bool ifcentered ();




#endif