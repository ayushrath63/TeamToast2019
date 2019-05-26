
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"



/* USER CODE BEGIN Includes */
#define SWO_DEBUG_ENABLED 0
#include "IMU.hpp"
#include "IRSensor.hpp"
#include "Motor.hpp"
#include "PID.hpp"
#include "MotionProfile.hpp"
#include "Drive.hpp"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile int EncL;
volatile int EncR;
volatile int prevEncL;
volatile int prevEncR;
volatile int diffL;
volatile int diffR;
volatile int EncAvg;
volatile int EncAngle;
volatile bool updatePIDflag = false;

constexpr int32_t CNT_PER_REV = 5760;
constexpr int32_t CELL = 13750;//8920;
constexpr float V_CRUISE = 10.0; // tick/ms
constexpr float MAX_ACCEL = 0.02; // tick/ms/ms
constexpr float V_TURN = 10.0;
int32_t nCommands = 15;

DriveCommand commands[15];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void initEncoders();
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_SYSTICK_Callback(void);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  initEncoders();
  IMU imu(hspi2);
  imu.init();

  //Initialize IR Sensors
  HAL_ADC_Start(&hadc1);
  IRSensor IRLeft(&hadc1, ADC_CHANNEL_8, IR_L_GPIO_Port, IR_L_Pin);
  IRSensor IRTopLeft(&hadc1, ADC_CHANNEL_14, IR_FL_GPIO_Port, IR_FL_Pin);
  IRSensor IRTopRight(&hadc1, ADC_CHANNEL_7, IR_FR_GPIO_Port, IR_FR_Pin);
  IRSensor IRRight(&hadc1, ADC_CHANNEL_5, IR_R_GPIO_Port, IR_R_Pin, true);

  //PID turnPID(0.035,0.0001,0.01); // .025
  //const int gyroTarget = 3675;
  
  Motor motorL(&htim3);
  Motor motorR(&htim4, true);
  MotionProfile mp(MAX_ACCEL, V_CRUISE);
  
  PID motorLPID(20.0,0.35,0.5);
  PID motorRPID(20.0,0.35,0.5);
  PID encAnglePID(0.01,0.0,0.0);

  HAL_Delay(2000);
  int32_t imuSum = 0; 
  int32_t irL, irR, irF, irF_Bad;
  int32_t pwmL, pwmR, motorTarget;
  char gzbuf[128];
  float speedTarget = 0;
  float speedW = 0;
  int32_t dt = 0;
  int32_t start = HAL_GetTick();
  int32_t cellCount = 0;


  for(int32_t i = 0; i < nCommands; ++i)
  {
    commands[i] = DriveCommand::FORWARD;
  }

  sprintf(gzbuf, "STARTING\r\n");
  print((uint8_t*)gzbuf);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //Check for new Command
    if(mp.isDone())
    {
      //Poll IR
      irF = IRLeft.read();
      irF_Bad = 0;///IRRight.read();
      irL = IRTopLeft.read();
      irR = IRTopRight.read();
      sprintf(gzbuf,"FL: %d, L: %d, R: %d, FR: %d\r\n", irF, irL, irR, irF_Bad);
      print((uint8_t*)gzbuf);

      motorLPID.resetError();
      motorRPID.resetError();
      encAnglePID.resetError();

      if(cellCount < nCommands)
      {
        cellCount++;
        mp.resetAll();
        switch(commands[cellCount])
        {
          case DriveCommand::FORWARD:
            mp.generate(CELL);
            break;
          default:
            break;
        }
      }
      start = HAL_GetTick();
    }


    if(updatePIDflag)
    {
      dt = HAL_GetTick() - start;
      
      //Get new speed cap from motion profile;
      speedTarget = mp.update(dt);

      encAnglePID.setTarget(0.0);
      speedW = encAnglePID.update(EncAngle);
      motorLPID.setTarget(speedTarget - speedW);
      motorRPID.setTarget(speedTarget + speedW);
      pwmL = motorLPID.update(diffL);
      pwmR = motorRPID.update(diffR);

      updatePIDflag = false;
      start = HAL_GetTick();
    }

    // motorR.setSpeed(pwmR);
    // motorL.setSpeed(pwmL);
    motorR.setSpeed(0);
    motorL.setSpeed(0);

    HAL_Delay(1);

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */ 

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void initEncoders()
{
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1 | TIM_CHANNEL_2);
  TIM2->EGR=TIM_EGR_UG;
  TIM2->CR1=TIM_CR1_CEN;
  HAL_TIM_Encoder_Start(&htim5,  TIM_CHANNEL_1 | TIM_CHANNEL_2);
  TIM5->EGR=TIM_EGR_UG;
  TIM5->CR1=TIM_CR1_CEN;
}

void HAL_SYSTICK_Callback(void)
{

  prevEncL = EncL;
  prevEncR = EncR;
  EncL = -1*TIM2->CNT;
  EncR = TIM5->CNT;
  EncAvg = (EncL + EncR) >> 2;
  EncAngle = (EncR - EncL);
  diffL = EncL - prevEncL;
  diffR = EncR - prevEncR;

  updatePIDflag = true;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/