/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LCD_B2_Pin GPIO_PIN_2
#define LCD_B2_GPIO_Port GPIOE
#define LCD_B3_Pin GPIO_PIN_3
#define LCD_B3_GPIO_Port GPIOE
#define LCD_B4_Pin GPIO_PIN_4
#define LCD_B4_GPIO_Port GPIOE
#define LCD_B5_Pin GPIO_PIN_5
#define LCD_B5_GPIO_Port GPIOE
#define LCD_B6_Pin GPIO_PIN_6
#define LCD_B6_GPIO_Port GPIOE
#define ADC_A_Pin GPIO_PIN_4
#define ADC_A_GPIO_Port GPIOA
#define ADC_B_Pin GPIO_PIN_5
#define ADC_B_GPIO_Port GPIOA
#define ADC_C_Pin GPIO_PIN_6
#define ADC_C_GPIO_Port GPIOA
#define LCD_B7_Pin GPIO_PIN_7
#define LCD_B7_GPIO_Port GPIOE
#define LCD_RST_Pin GPIO_PIN_8
#define LCD_RST_GPIO_Port GPIOE
#define LCD_PCB_Pin GPIO_PIN_9
#define LCD_PCB_GPIO_Port GPIOE
#define LCD_E_Pin GPIO_PIN_10
#define LCD_E_GPIO_Port GPIOE
#define LCD_RW_Pin GPIO_PIN_11
#define LCD_RW_GPIO_Port GPIOE
#define LCD_RS_Pin GPIO_PIN_12
#define LCD_RS_GPIO_Port GPIOE
#define CP_ERR_Pin GPIO_PIN_12
#define CP_ERR_GPIO_Port GPIOB
#define CP_D_Pin GPIO_PIN_13
#define CP_D_GPIO_Port GPIOB
#define CP_C_Pin GPIO_PIN_14
#define CP_C_GPIO_Port GPIOB
#define CP_B_Pin GPIO_PIN_15
#define CP_B_GPIO_Port GPIOB
#define PP_13A_Pin GPIO_PIN_10
#define PP_13A_GPIO_Port GPIOD
#define PP_20A_Pin GPIO_PIN_11
#define PP_20A_GPIO_Port GPIOD
#define PP_32A_Pin GPIO_PIN_12
#define PP_32A_GPIO_Port GPIOD
#define PP_70A_Pin GPIO_PIN_13
#define PP_70A_GPIO_Port GPIOD
#define PP_ERR_Pin GPIO_PIN_14
#define PP_ERR_GPIO_Port GPIOD
#define BUT1_Pin GPIO_PIN_1
#define BUT1_GPIO_Port GPIOD
#define BUT2_Pin GPIO_PIN_2
#define BUT2_GPIO_Port GPIOD
#define BUT3_Pin GPIO_PIN_3
#define BUT3_GPIO_Port GPIOD
#define BUT4_Pin GPIO_PIN_4
#define BUT4_GPIO_Port GPIOD
#define BUT5_Pin GPIO_PIN_5
#define BUT5_GPIO_Port GPIOD
#define BUT6_Pin GPIO_PIN_6
#define BUT6_GPIO_Port GPIOD
#define LCD_B0_Pin GPIO_PIN_0
#define LCD_B0_GPIO_Port GPIOE
#define LCD_B1_Pin GPIO_PIN_1
#define LCD_B1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
