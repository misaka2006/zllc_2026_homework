/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define M4_IN1_Pin GPIO_PIN_13
#define M4_IN1_GPIO_Port GPIOC
#define M4_IN2_Pin GPIO_PIN_14
#define M4_IN2_GPIO_Port GPIOC
#define BlueTooth_TX_Pin GPIO_PIN_2
#define BlueTooth_TX_GPIO_Port GPIOA
#define BlueTooth_RX_Pin GPIO_PIN_3
#define BlueTooth_RX_GPIO_Port GPIOA
#define M2_IN1_Pin GPIO_PIN_4
#define M2_IN1_GPIO_Port GPIOA
#define M2_IN2_Pin GPIO_PIN_5
#define M2_IN2_GPIO_Port GPIOA
#define S1_Pin GPIO_PIN_6
#define S1_GPIO_Port GPIOA
#define S2_Pin GPIO_PIN_7
#define S2_GPIO_Port GPIOA
#define S3_Pin GPIO_PIN_0
#define S3_GPIO_Port GPIOB
#define S4_Pin GPIO_PIN_1
#define S4_GPIO_Port GPIOB
#define M1_PWM_Pin GPIO_PIN_8
#define M1_PWM_GPIO_Port GPIOA
#define M2_PWM_Pin GPIO_PIN_9
#define M2_PWM_GPIO_Port GPIOA
#define M3_PWM_Pin GPIO_PIN_10
#define M3_PWM_GPIO_Port GPIOA
#define M4_PWM_Pin GPIO_PIN_11
#define M4_PWM_GPIO_Port GPIOA
#define M1_IN1_Pin GPIO_PIN_12
#define M1_IN1_GPIO_Port GPIOA
#define M1_IN2_Pin GPIO_PIN_15
#define M1_IN2_GPIO_Port GPIOA
#define M3_IN1_Pin GPIO_PIN_3
#define M3_IN1_GPIO_Port GPIOB
#define M3_IN2_Pin GPIO_PIN_4
#define M3_IN2_GPIO_Port GPIOB
#define UpCom_TX_Pin GPIO_PIN_6
#define UpCom_TX_GPIO_Port GPIOB
#define UpCom_RX_Pin GPIO_PIN_7
#define UpCom_RX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
