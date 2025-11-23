#include "tim.h"
#include "servo_tim3.h"


static uint16_t Servo_AngleToPulse(uint8_t angle)
{
    //限制角度在0-180范围
    if (angle < SERVO_MIN_ANGLE)
    {
        angle = SERVO_MIN_ANGLE;
    }
    else if (angle > SERVO_MAX_ANGLE)
    {
        angle = SERVO_MAX_ANGLE;
    }
    
    //角度线性映射为比较值
    return SERVO_MIN_PULSE + (uint16_t)((SERVO_MAX_PULSE - SERVO_MIN_PULSE) * angle / (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE));
}

    //控制TIM3通道舵机转动到指定角度
void Servo_TIM3_SetAngle(uint32_t Channel, uint8_t angle)
{
    uint16_t pulse_value = Servo_AngleToPulse(angle);  //角度转比较值
    __HAL_TIM_SET_COMPARE(&htim3, Channel, pulse_value);  //更新PWM占空比
}