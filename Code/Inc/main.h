/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define Disp_In_Pin GPIO_PIN_13
#define Disp_In_GPIO_Port GPIOC
#define Disp_Clk_Pin GPIO_PIN_14
#define Disp_Clk_GPIO_Port GPIOC
#define IMU_Int_Pin GPIO_PIN_15
#define IMU_Int_GPIO_Port GPIOC
#define Btn1_Pin GPIO_PIN_0
#define Btn1_GPIO_Port GPIOC
#define R_Encoder_A_Pin GPIO_PIN_0
#define R_Encoder_A_GPIO_Port GPIOA
#define R_Encoder_B_Pin GPIO_PIN_1
#define R_Encoder_B_GPIO_Port GPIOA
#define IR_R_Pin GPIO_PIN_3
#define IR_R_GPIO_Port GPIOA
#define IR_FR_Pin GPIO_PIN_4
#define IR_FR_GPIO_Port GPIOA
#define Rec_R_Pin GPIO_PIN_5
#define Rec_R_GPIO_Port GPIOA
#define Rec_FR_Pin GPIO_PIN_7
#define Rec_FR_GPIO_Port GPIOA
#define Rec_FL_Pin GPIO_PIN_4
#define Rec_FL_GPIO_Port GPIOC
#define Rec_L_Pin GPIO_PIN_0
#define Rec_L_GPIO_Port GPIOB
#define IR_FL_Pin GPIO_PIN_10
#define IR_FL_GPIO_Port GPIOB
#define IR_L_Pin GPIO_PIN_13
#define IR_L_GPIO_Port GPIOB
#define Buzzer_Pin GPIO_PIN_8
#define Buzzer_GPIO_Port GPIOC
#define L_Encoder_A_Pin GPIO_PIN_15
#define L_Encoder_A_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_10
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_11
#define LED2_GPIO_Port GPIOC
#define Disp_CE_Pin GPIO_PIN_12
#define Disp_CE_GPIO_Port GPIOC
#define Disp_RS_Pin GPIO_PIN_2
#define Disp_RS_GPIO_Port GPIOD
#define L_Encoder_B_Pin GPIO_PIN_3
#define L_Encoder_B_GPIO_Port GPIOB
#define Motor_L_F_Pin GPIO_PIN_4
#define Motor_L_F_GPIO_Port GPIOB
#define Motor_L_B_Pin GPIO_PIN_5
#define Motor_L_B_GPIO_Port GPIOB
#define Motor_R_B_Pin GPIO_PIN_6
#define Motor_R_B_GPIO_Port GPIOB
#define Motor_R_F_Pin GPIO_PIN_7
#define Motor_R_F_GPIO_Port GPIOB
#define IMU_nCS_Pin GPIO_PIN_9
#define IMU_nCS_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
volatile extern int EncL;
volatile extern int EncR;
volatile extern int prevEncL;
volatile extern int prevEncR;
volatile extern int diffL;
volatile extern int diffR;
volatile extern int EncAvg;
volatile extern int EncAngle;
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
