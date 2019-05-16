#include "IRSensor.hpp"

IRSensor::IRSensor (ADC_HandleTypeDef* adcHandle, uint32_t channel, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, bool invert) {
    m_adcHandle = adcHandle;
    m_channel = channel;
    m_GPIOx = GPIOx;
    m_GPIO_Pin = GPIO_Pin;
    m_invert = invert;
}

uint32_t IRSensor::read(){

    uint32_t ADC_VAL;
    if(!m_invert) HAL_GPIO_WritePin(m_GPIOx, m_GPIO_Pin, GPIO_PIN_SET);
    else HAL_GPIO_WritePin(m_GPIOx, m_GPIO_Pin, GPIO_PIN_RESET);
    ADC_VAL = readADC(m_adcHandle,m_channel, 500);
    HAL_Delay(1000);
    if(!m_invert) HAL_GPIO_WritePin(m_GPIOx, m_GPIO_Pin, GPIO_PIN_RESET);
    else HAL_GPIO_WritePin(m_GPIOx, m_GPIO_Pin, GPIO_PIN_SET);
    return ADC_VAL;
}

//    sprintf(gzbuf, "%ld, %ld, %ld, %ld\r\n", ADC_VAL1, ADC_VAL2, ADC_VAL3, ADC_VAL4);
//    print((uint8_t*)gzbuf);