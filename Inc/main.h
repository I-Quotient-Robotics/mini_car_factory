/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

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
void UART_IDLE_Callback(UART_HandleTypeDef *huart);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED0_Pin GPIO_PIN_0
#define LED0_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_3
#define LED3_GPIO_Port GPIOC
#define WAKE_UP_Pin GPIO_PIN_0
#define WAKE_UP_GPIO_Port GPIOA
#define VL530LX_INT_Pin GPIO_PIN_4
#define VL530LX_INT_GPIO_Port GPIOA
#define VL530LX_XSHUT_Pin GPIO_PIN_5
#define VL530LX_XSHUT_GPIO_Port GPIOA
#define TX2_POWER_Pin GPIO_PIN_6
#define TX2_POWER_GPIO_Port GPIOA
#define TX2_READY_Pin GPIO_PIN_7
#define TX2_READY_GPIO_Port GPIOA
#define LED4_Pin GPIO_PIN_4
#define LED4_GPIO_Port GPIOC
#define LED5_Pin GPIO_PIN_5
#define LED5_GPIO_Port GPIOC
#define IR_1_Pin GPIO_PIN_0
#define IR_1_GPIO_Port GPIOB
#define IR_2_Pin GPIO_PIN_1
#define IR_2_GPIO_Port GPIOB
#define LED6_Pin GPIO_PIN_6
#define LED6_GPIO_Port GPIOC
#define LED7_Pin GPIO_PIN_7
#define LED7_GPIO_Port GPIOC
#define LED8_Pin GPIO_PIN_8
#define LED8_GPIO_Port GPIOC
#define LED9_Pin GPIO_PIN_9
#define LED9_GPIO_Port GPIOC
#define POWER_EN_Pin GPIO_PIN_8
#define POWER_EN_GPIO_Port GPIOA
#define CHARGING_STATE_Pin GPIO_PIN_9
#define CHARGING_STATE_GPIO_Port GPIOA
#define SOUND_EN_Pin GPIO_PIN_10
#define SOUND_EN_GPIO_Port GPIOA
#define TLED_Pin GPIO_PIN_11
#define TLED_GPIO_Port GPIOA
#define LIMIT_A_Pin GPIO_PIN_12
#define LIMIT_A_GPIO_Port GPIOA
#define LIMIT_B_Pin GPIO_PIN_15
#define LIMIT_B_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
