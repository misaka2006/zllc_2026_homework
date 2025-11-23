#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "MotorDriver.h"
void MotorDriverInit(MotorDriver *mot, GPIO_TypeDef *m_in1_port, uint16_t m_in1_pin, GPIO_TypeDef *m_in2_port, uint16_t m_in2_pin, uint8_t dir, bool rev_flag,uint8_t tim_ch)
{
    mot->motor_in1_port = m_in1_port;
    mot->motor_in1_pin = m_in1_pin;
    mot->motor_in2_port = m_in2_port;
    mot->motor_in2_pin = m_in2_pin;
    mot->direction = dir;
    mot->rev = rev_flag;
    mot->tim_chn=tim_ch;

}

void MotorDriverSetSpeed(MotorDriver *mot, float spd)
{
    bool inv_flag=0;
    if (spd < 0)
    {
        spd = -spd;
    }
        
    spd *= 100;
    uint8_t _spd = (uint8_t)(spd);
    if (_spd > 100)
        _spd = 100;
    if (_spd < 1)
        _spd = 0;
    mot->motor_speed = _spd;
    switch (mot->direction)
    {
    case MOTOR_DIR_CW:
        HAL_GPIO_WritePin(mot->motor_in1_port,mot->motor_in1_pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(mot->motor_in2_port,mot->motor_in2_pin,GPIO_PIN_RESET);
        break;
    case MOTOR_DIR_CCW:
				inv_flag=1;
        HAL_GPIO_WritePin(mot->motor_in1_port,mot->motor_in1_pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(mot->motor_in2_port,mot->motor_in2_pin,GPIO_PIN_SET);
        break;
    case MOTOR_DIR_STOP:
        HAL_GPIO_WritePin(mot->motor_in1_port,mot->motor_in1_pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(mot->motor_in2_port,mot->motor_in2_pin,GPIO_PIN_RESET);
        break;
    default:
        break;
    }
    __HAL_TIM_SET_COMPARE(&htim4, mot->tim_chn, (inv_flag?100-mot->motor_speed:mot->motor_speed));
}
void MotorDriverStop(MotorDriver *mot)
{
    HAL_GPIO_WritePin(mot->motor_in1_port,mot->motor_in1_pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(mot->motor_in2_port,mot->motor_in2_pin,GPIO_PIN_RESET);
}

void MotorDriverSetDirection(MotorDriver *mot,uint8_t dir)
{   
	mot->direction=dir;
	switch (dir)
    {
    case MOTOR_DIR_CW:
        HAL_GPIO_WritePin(mot->motor_in1_port,mot->motor_in1_pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(mot->motor_in2_port,mot->motor_in2_pin,GPIO_PIN_RESET);
        break;
    case MOTOR_DIR_CCW:
				//inv_flag=1;
        HAL_GPIO_WritePin(mot->motor_in1_port,mot->motor_in1_pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(mot->motor_in2_port,mot->motor_in2_pin,GPIO_PIN_SET);
        break;
    case MOTOR_DIR_STOP:
        HAL_GPIO_WritePin(mot->motor_in1_port,mot->motor_in1_pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(mot->motor_in2_port,mot->motor_in2_pin,GPIO_PIN_RESET);
        break;
    default:
        break;
    }
}
