#ifndef __SERVO_TIM3_H
#define __SERVO_TIM3_H

#include "stm32f1xx_hal.h" 

#define SERVO_MIN_ANGLE     0      
#define SERVO_MAX_ANGLE     180    


#define SERVO_MIN_PULSE     50      //0度对应的PWM比较值
#define SERVO_MAX_PULSE     250     //180度对应的PWM比较值


void Servo_TIM3_SetAngle(uint32_t Channel, uint8_t angle);

#endif