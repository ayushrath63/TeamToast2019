#ifndef __IRSENSOR_H__
#define __IRSENEOR_H__

#include "main.h"
#include "adc.h"
#include "stm32f4xx_hal.h"
#include "gpio.h"

constexpr float WALL_R = 3820;
constexpr float WALL_L = 3890;
constexpr float WALL_F = 3237;
constexpr float WALL_2F = 2165;

// no wall F= 1960


class IRSensor {

public: 
	uint32_t read();
	IRSensor(ADC_HandleTypeDef* adcHandle, uint32_t channel, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, bool invert = false);


private:
	ADC_HandleTypeDef* m_adcHandle;
	uint32_t m_channel;
	GPIO_TypeDef* m_GPIOx;
	uint16_t m_GPIO_Pin;
	bool m_invert;
};


#endif