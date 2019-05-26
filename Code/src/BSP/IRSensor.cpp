#include "IRSensor.hpp"

IRSensor IRLeft(&hadc1, ADC_CHANNEL_8, IR_L_GPIO_Port, IR_L_Pin);
IRSensor IRTopLeft(&hadc1, ADC_CHANNEL_14, IR_FL_GPIO_Port, IR_FL_Pin);
IRSensor IRTopRight(&hadc1, ADC_CHANNEL_7, IR_FR_GPIO_Port, IR_FR_Pin);

IRSensor::IRSensor (ADC_HandleTypeDef* adcHandle, uint32_t channel, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, bool invert) {
    m_adcHandle = adcHandle;
    m_channel = channel;
    m_GPIOx = GPIOx;
    m_GPIO_Pin = GPIO_Pin;
    m_invert = invert;
    m_val = 0; 
}

uint32_t IRSensor::read(){

    uint32_t ADC_VAL;
    HAL_GPIO_WritePin(m_GPIOx, m_GPIO_Pin, GPIO_PIN_SET);
    ADC_VAL = readADC(m_adcHandle,m_channel, 500);
    //HAL_Delay(1);
    HAL_GPIO_WritePin(m_GPIOx, m_GPIO_Pin, GPIO_PIN_RESET);
    m_val = ADC_VAL;
    return ADC_VAL;
}

uint32_t IRSensor::value(){
    return m_val;
}

void IRSensor_readAll() {
    char gzbuf[128];
    IRLeft.read();
    IRTopLeft.read();
    IRTopRight.read();
    sprintf(gzbuf,"FL: %d, TopL: %d, TopR: %d\r\n", IRLeft.value(),  IRTopLeft.value(), IRTopRight.value());
   print((uint8_t*)gzbuf);
}

bool ifdetectedFrontWall() {
    //char gzbuf[128];
    int res = IRLeft.value() > OPEN_F;
    //sprintf(gzbuf,"ifFront wall: return val: %d\n", res);
    //print((uint8_t*)gzbuf);
    return (res);
}
bool ifdetectedRightWall() {
    // char gzbuf[128];
    // sprintf(gzbuf,"IN IFRIGHT WALL");
    // print((uint8_t*)gzbuf);

    return (IRTopRight.value() > OPEN_R);} 
bool ifdetectedLeftWall() {return (IRTopLeft.value() > OPEN_L);} 
bool ifcentered () 
{
    return (IRTopRight.value() > 2900 &&
            IRTopRight.value() < 3200 &&
            IRTopLeft.value() > 3600 &&
            IRTopLeft.value() < 3830 );
}

//    sprintf(gzbuf, "%ld, %ld, %ld, %ld\r\n", ADC_VAL1, ADC_VAL2, ADC_VAL3, ADC_VAL4);
//    print((uint8_t*)gzbuf);