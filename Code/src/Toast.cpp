
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
#include "Encoder.hpp"
#include <cmath>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

volatile bool updatePIDflag = false;

constexpr int32_t CNT_PER_REV = 5760;
//constexpr int32_t CELL = 15500;//8920;
constexpr float V_CRUISE = 10.0; // tick/ms
constexpr float MAX_ACCEL = 0.02; // tick/ms/ms
constexpr float V_TURN = 10.0;
int32_t nCommands = 8;

// Drive Command 
// vector< DriveCommand > commands;
// commands.push_back(FORWARD);
DriveCommand commands[8] = 
{
  DriveCommand::FORWARD,
  DriveCommand::TURNRIGHT,
  DriveCommand::TURNLEFT,
  DriveCommand::TURNLEFT,
  DriveCommand::TURNRIGHT, 
  DriveCommand::TURNLEFT,
  DriveCommand::TURNRIGHT,
  DriveCommand::TURNRIGHT,


};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

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

  //PID turnPID(0.035,0.0001,0.01); // .025
  //const int gyroTarget = 3675;
  
  Motor motorL(&htim3);
  Motor motorR(&htim4, true);
  MotionProfile mp(MAX_ACCEL, V_CRUISE);
  


  HAL_Delay(2000);
  int32_t imuSum = 0; 
  char gzbuf[128];
  float speedTarget = 0;
  float speedW = 0;
  float angleTarget = 0;
  int32_t dt = 0;
  int32_t start = HAL_GetTick();
  int32_t cellCount = 0;

  sprintf(gzbuf, "STARTING\r\n");
  print((uint8_t*)gzbuf);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while(1) {
    if (Command::complete) {
      sprintf(gzbuf,"Cur_Command: %d, Next: %d \r\n", (int)Command::cur_command, (int)Command::next_command);
      print((uint8_t*)gzbuf);

      motorR.setSpeed(0);
      motorL.setSpeed(0);
      //Adjust 
      resetEncoder();
      Command::complete = false;
      HAL_Delay(2000);
      IRSensor_readAll();
      if (Command::next_command == DriveCommand::NONE) {
        Command::setNextCommand(); // set the current command too 
      } else {
        Command::cur_command = Command::next_command;
        Command::next_command = DriveCommand::NONE;
      }
    } else {
      //sprintf(gzbuf,"pwmL: %d, pwmR: %d \r\n", (int)(pwmL*1000), (int)(pwmR*1000));
      //print((uint8_t*)gzbuf);
      switch(Command::cur_command) {
        case DriveCommand::FORWARD:
          goForward();
          break;
        case DriveCommand::TURNLEFT:
          turnLeft();
          break;
        case DriveCommand::TURNRIGHT:
          turnRight();
          break;
        case DriveCommand::TURN180:
          turn180();
          break;
        default:
          goForward();
      }
      
    }
    // turn180();
     // sprintf(gzbuf,"EncoderL: %d, EncoderR: %d \r\n", EncL, EncR);
     // print((uint8_t*)gzbuf);
    HAL_Delay(1);
    motorR.setSpeed(pwmR);
    motorL.setSpeed(pwmL);
  }
  // while (1)
  // {
  //   DriveCommand next = commands[cellCount];
  //   //Check for new Command
  //   if(mp.isDone())
  //   {

  //     //Poll IR
  //     // irF = IRLeft.read();
  //     // irF_Bad = 0;///IRRight.read();
  //     // irL = IRTopLeft.read();
  //     // irR = IRTopRight.read();
  //     // sprintf(gzbuf,"FL: %d, L: %d, R: %d, FR: %d\r\n", irF, irL, irR, irF_Bad);
  //     // print((uint8_t*)gzbuf);
  //     IRSensor_readAll();
  //     motorLPID.resetError();
  //     motorRPID.resetError();
  //     encAnglePID.resetError();
  //     motorR.setSpeed(0);
  //     motorL.setSpeed(0);
  //     //imuSum = 0;
  //     if(cellCount < nCommands)
  //     {
  //       cellCount++;
  //       TIM2->CNT = 0;
  //       TIM5->CNT = 0;
  //       EncAngle = 0;
  //       EncAvg = 0;
  //       mp.resetAll();
  //       switch(next)
  //       {
  //         case DriveCommand::FORWARD:
  //           mp.generate(CELL);
  //           angleTarget = 0.0;
  //           break;
  //         case DriveCommand::TURNLEFT:
  //           mp.generate(CELL); 
  //           angleTarget = 5250.0;
  //           break;
  //         case DriveCommand::TURNRIGHT:
  //           mp.generate(CELL); 
  //           angleTarget = -5250.0;
  //           break;
  //         default:
  //           break;
  //       }
  //     }
  //     start = HAL_GetTick();
  //   }

  // float encAnglePIDResult;
  //   if(updatePIDflag)
  //   {
  //     dt = HAL_GetTick() - start;
      
  //     //Get new speed cap from motion profile;
  //     float update = mp.update(dt);
  //     speedTarget = 0;
  //     if(next == DriveCommand::FORWARD)
  //     {
  //       speedTarget = update;
  //     }

  //     encAnglePID.setTarget(angleTarget);
  //     encAnglePIDResult = encAnglePID.update(EncAngle);
  //     speedW = abs(encAnglePIDResult) < 10 ? encAnglePIDResult : encAnglePIDResult / abs(encAnglePIDResult) *10;
  //     motorLPID.setTarget(speedTarget - speedW);
  //     motorRPID.setTarget(speedTarget + speedW);
  //     pwmL = motorLPID.update(diffL);
  //     pwmR = motorRPID.update(diffR);

  //     updatePIDflag = false;
  //     start = HAL_GetTick();
  //   }


  //     sprintf(gzbuf,"Command: %d, Time: %d / %d\r\n", (int)next, (int)mp.m_currTime, (int)mp.m_totalTime);
  //     print((uint8_t*)gzbuf);


  //   motorR.setSpeed(pwmR);
  //   motorL.setSpeed(pwmL);


  //   imuSum += imu.read();
  //   HAL_Delay(1);

  // /* USER CODE END WHILE */

  // /* USER CODE BEGIN 3 */

  // }
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