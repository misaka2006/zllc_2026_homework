#include "task.h"
#include "drv_can.h"
#include "drv_tim.h"
#include "drv_uart.h"

#include "dvc_djimotor.h"
#include "dvc_dr16.h"

#include "crt_chassis.h"

Class_DJI_Motor_C620 motor_3508; 
Class_DJI_Motor_C620 motor_wheel[5];
Class_DR16 DR16;

int flag=0;

Class_Tricycle_Chassis chassis;

float v_x_max,v_y_max;

void Chassis_Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
        case (0x201):
        {
            chassis.Motor_Wheel[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case (0x202):
        {
            chassis.Motor_Wheel[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case (0x203):
        {
            chassis.Motor_Wheel[2].CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case (0x204):
        {
            chassis.Motor_Wheel[3].CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case (0x206):  
        {
            
        }
        break;
        case (0x207):
        {
            
        }
        break;
    }
}

void DR16_UART3_Callback(uint8_t *Buffer, uint16_t Length)
{
    DR16.DR16_UART_RxCpltCallback(Buffer);
    //Judge_DR16_Control_Type();
}

void Image_UART1_Callback(uint8_t *Buffer, uint16_t Length)
{
    DR16.Image_UART_RxCpltCallback(Buffer);
}

void Task1ms_TIM5_Callback()
{
    //flag++;
    //motor_3508.TIM_PID_PeriodElapsedCallback();
    chassis.Set_Target_Velocity_X(DR16.Get_Left_X()*chassis.Get_Velocity_X_Max());
    chassis.Set_Target_Velocity_Y(DR16.Get_Left_Y()*chassis.Get_Velocity_Y_Max());
    chassis.Set_Target_Omega(-1.0*DR16.Get_Right_X()*chassis.Get_Omega_Max());
    // chassis.Set_Target_Velocity_X(1.0f);
    // chassis.Set_Target_Velocity_Y(0.0f);
    //chassis.Set_Now_Velocity_X(1.0f);
    //chassis.Set_Now_Velocity_Y(0.0f);
    chassis.TIM_Calculate_PeriodElapsedCallback(Sprint_Status_DISABLE);
    //DR16.TIM1msMod50_Alive_PeriodElapsedCallback();
    
    TIM_CAN_PeriodElapsedCallback();
    TIM_UART_PeriodElapsedCallback();
}

void Task_Init()
{
    CAN_Init(&hcan1, Chassis_Device_CAN1_Callback);
    UART_Init(&huart3, DR16_UART3_Callback, 18);
    UART_Init(&huart1, Image_UART1_Callback, 40);
    TIM_Init(&htim5,Task1ms_TIM5_Callback);
    
    // motor_wheel[1].Init(&hcan1,DJI_Motor_ID_0x201,DJI_Motor_Control_Method_OMEGA,19.0f);
    // motor_wheel[2].Init(&hcan1,DJI_Motor_ID_0x202,DJI_Motor_Control_Method_OMEGA,19.0f);
    // motor_wheel[3].Init(&hcan1,DJI_Motor_ID_0x203,DJI_Motor_Control_Method_OMEGA,19.0f);
    // motor_wheel[4].Init(&hcan1,DJI_Motor_ID_0x204,DJI_Motor_Control_Method_OMEGA,19.0f);
    
    // for(int i=1;i<=4;i++)
    // {
    //     //motor_wheel[i].Init(&hcan1,i,DJI_Motor_Control_Method_OMEGA,19.0f);
    //     motor_wheel[i].PID_Omega.Init(1.0f,1.0f,0.0f,1.0f);
    // }
    chassis.Init(4.0f,4.0f,8.0f,0.5f);
    chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
    // //motor_3508.Init(&hcan1,DJI_Motor_ID_0x201,DJI_Motor_Control_Method_OMEGA,19.0f);
    DR16.Init(&huart3,&huart1);
    HAL_TIM_Base_Start_IT(&htim5);
    // //motor_3508.PID_Omega.Init(1.0f,1.0f,0.0f,1.0f);
    v_x_max=chassis.Get_Velocity_X_Max();
    v_y_max=chassis.Get_Velocity_Y_Max();
}

void Task_Loop()
{

}

