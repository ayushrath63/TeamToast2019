#include "Encoder.hpp"

volatile int EncL;
volatile int EncR;
volatile int prevEncL;
volatile int prevEncR;
volatile int diffL;
volatile int diffR;
volatile int EncAvg;
volatile int EncAngle;


void initEncoders()
{
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1 | TIM_CHANNEL_2);
  TIM2->EGR=TIM_EGR_UG;
  TIM2->CR1=TIM_CR1_CEN;
  HAL_TIM_Encoder_Start(&htim5,  TIM_CHANNEL_1 | TIM_CHANNEL_2);
  TIM5->EGR=TIM_EGR_UG;
  TIM5->CR1=TIM_CR1_CEN;
}
void resetEncoder() {
	    TIM2->CNT = 0;
        TIM5->CNT = 0;
        EncAngle = 0;
        EncAvg = 0;
        prevEncL = 0;
        prevEncR = 0;
        diffL = 0;
        diffR = 0;
}