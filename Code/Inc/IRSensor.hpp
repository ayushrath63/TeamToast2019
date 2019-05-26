#ifndef __IRSENSOR_H__
#define __IRSENEOR_H__

#include "main.h"
#include "adc.h"
#include "stm32f4xx_hal.h"
#include "gpio.h"

constexpr float WALL_R = 3092; // 2864 - 3269
constexpr float WALL_L = 3735; // 3600 - 3834
constexpr float WALL_F = 2710;  // 2600 - 3000
constexpr float WALL_2F = 2165;

// no wall F= 2000

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


IRSensor IRLeft(&hadc1, ADC_CHANNEL_8, IR_L_GPIO_Port, IR_L_Pin);
IRSensor IRTopLeft(&hadc1, ADC_CHANNEL_14, IR_FL_GPIO_Port, IR_FL_Pin);
IRSensor IRTopRight(&hadc1, ADC_CHANNEL_7, IR_FR_GPIO_Port, IR_FR_Pin);
IRSensor IRRight(&hadc1, ADC_CHANNEL_5, IR_R_GPIO_Port, IR_R_Pin, true);

void IR_read() {
	char gzbuf[128];
	irF = IRLeft.read();
	irF_Bad = 0;///IRRight.read();
	irL = IRTopLeft.read();
	irR = IRTopRight.read();
	sprintf(gzbuf,"FL: %d, L: %d, R: %d, FR: %d\r\n", irF, irL, irR, irF_Bad);
	print((uint8_t*)gzbuf);
}
#endif