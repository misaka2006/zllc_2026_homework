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
// ϵͳʱ������
void SystemClock_Config(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PS2_DI_Pin GPIO_PIN_12
#define PS2_DI_GPIO_Port GPIOB
#define PS2_CMD_Pin GPIO_PIN_13
#define PS2_CMD_GPIO_Port GPIOB
#define PS2_CS_Pin GPIO_PIN_14
#define PS2_CS_GPIO_Port GPIOB
#define PS2_CLK_Pin GPIO_PIN_15
#define PS2_CLK_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
//电机的宏定义
#define MOTOR1_PWM_PIN          GPIO_PIN_0
#define MOTOR1_PWM_PORT         GPIOA
#define MOTOR1_IN1_PIN          GPIO_PIN_4
#define MOTOR1_IN1_PORT         GPIOA
#define MOTOR1_IN2_PIN          GPIO_PIN_5
#define MOTOR1_IN2_PORT         GPIOA

#define MOTOR2_PWM_PIN          GPIO_PIN_1
#define MOTOR2_PWM_PORT         GPIOA
#define MOTOR2_IN1_PIN          GPIO_PIN_6
#define MOTOR2_IN1_PORT         GPIOA
#define MOTOR2_IN2_PIN          GPIO_PIN_7
#define MOTOR2_IN2_PORT         GPIOA

#define MOTOR3_PWM_PIN          GPIO_PIN_2
#define MOTOR3_PWM_PORT         GPIOA
#define MOTOR3_IN1_PIN          GPIO_PIN_0
#define MOTOR3_IN1_PORT         GPIOB
#define MOTOR3_IN2_PIN          GPIO_PIN_1
#define MOTOR3_IN2_PORT         GPIOB

#define MOTOR4_PWM_PIN          GPIO_PIN_3
#define MOTOR4_PWM_PORT         GPIOA
#define MOTOR4_IN1_PIN          GPIO_PIN_9    // �޸ģ�ʹ��PB9���PB2
#define MOTOR4_IN1_PORT         GPIOB
#define MOTOR4_IN2_PIN          GPIO_PIN_3
#define MOTOR4_IN2_PORT         GPIOB

// ϵͳ����
#define CONTROL_FREQ_HZ         50     // 频率50Hz
#define MODE_SWITCH_DEBOUNCE_MS 300    // ģ消抖
#define UART_TIMEOUT_MS         500    // 防止超时

// �������
typedef enum {
    ERROR_NONE = 0,
    ERROR_UART_TIMEOUT,
    ERROR_MOTOR_FAULT
} ErrorCode_t;



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
