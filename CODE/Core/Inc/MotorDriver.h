#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>

// M1
#define M1_IN2_PORT GPIOC
#define M1_IN2_PIN GPIO_PIN_13
#define M1_IN1_PORT GPIOC
#define M1_IN1_PIN GPIO_PIN_14
// M2
#define M2_IN1_PORT GPIOA
#define M2_IN1_PIN GPIO_PIN_5
#define M2_IN2_PORT GPIOB
#define M2_IN2_PIN GPIO_PIN_4
// M3
#define M3_IN1_PORT GPIOC
#define M3_IN1_PIN GPIO_PIN_15
#define M3_IN2_PORT GPIOB
#define M3_IN2_PIN GPIO_PIN_14
// M4
#define M4_IN2_PORT GPIOB
#define M4_IN2_PIN GPIO_PIN_10
#define M4_IN1_PORT GPIOB
#define M4_IN1_PIN GPIO_PIN_11

#define MOTOR_DIR_CW 1   // 正转
#define MOTOR_DIR_CCW 2  // 反转
#define MOTOR_DIR_STOP 3 // 停止

typedef struct
{
    uint8_t motor_speed;
    uint8_t direction;
    bool rev;
    GPIO_TypeDef *motor_in1_port;
    uint16_t motor_in1_pin;
    GPIO_TypeDef *motor_in2_port;
    uint16_t motor_in2_pin;
    uint8_t tim_chn;
} MotorDriver;

void MotorDriverInit(MotorDriver *mot, GPIO_TypeDef *m_in1_port, uint16_t m_in1_pin, GPIO_TypeDef *m_in2_port, uint16_t m_in2_pin, uint8_t dir, bool rev_flag,uint8_t tim_ch);
void MotorDriverSetSpeed(MotorDriver *mot, float spd);
void MotorDriverSetDirection(MotorDriver *mot,uint8_t dir);
void MotorDriverStop(MotorDriver *mot);

#endif
