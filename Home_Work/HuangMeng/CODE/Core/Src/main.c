/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BlueTooth.h"
#include "UpCom.h"
#include <math.h>
//蓝牙通信相关数据
extern uint8_t Bluetooth_RxData[Bluetooth_length];//接收缓冲区
extern struct Struct_Bluetooth_RxData RxData;//存放数据的结构体
extern struct Struct_Bluetooth_PorcessData PorcessData;//处理后的结构体
//与上位机通信相关数据
extern uint8_t UpCom_RxData_Buffer[UpCom_length];//接收缓冲区域
extern struct Struct_UpCom_RxData UpComRxData;//存放数据的结构体
extern struct Struct_UpCom_ProcessData ProcessData;//处理后的结构体
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//存放与驱动轮子有关的数据的结构体
typedef struct {
	float angle;					//手柄指向角度（以手柄松手处为原点平面直角坐标系）
	float divisor;				//除数（用来平衡占空比的值，解决麦轮解算出来后四角的值超过ARR的问题）
	float pct_omega;			//车的角速度对应占空比
	float pct_vx;    			//x方向速度比例
	float pct_vy;    			//y方向速度比例
	float v_wheels[5];		//轮子的速度
	float duty_wheels[5];	//轮子的占空比（pwm直接控制轮子转速）
}Mc_Wheels_Typedef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DY 0.1   					//中心到轮轴的最短距离
#define DX 0.06   				//轮轴中心到轮子中心的距离
#define R_WHEEL 0.03			//轮子半径
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Mc_Wheels_Typedef mc_wheels;		//创建轮子结构体
uint8_t temp[4] = {1,2,3,4};		//临时变量用来要发给电脑的数据（仅仅用于检测）

//舵机位置初始化
int duty_S1 = 1442;//2800初始
int duty_S2 = 2800;
int duty_S3 = 4560;
int duty_S4 = 1920;

//电机速度上限
int Max_Speed = 99;
int Max_Speed_w = 99;

float DUTY_Wheels[5] = {0.0f};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Amon_AllMotor_SharpBrake(void);	//急刹
void Amon_AllMotor_SlowBrake(void);		//缓刹
void Amon_Chassis_Calculate_Contrary(Mc_Wheels_Typedef* Mc_wheels,struct Struct_Bluetooth_PorcessData* PorcessData);  //底盘解算
void Amon_Direct_control(Mc_Wheels_Typedef* mc_wheels);	 //方向控制（控制各个轮子的转向）
void Amon_Serve_Control(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t count;//计数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//一段时间检测以下各个按钮的状态
{
    if (htim->Instance == TIM2)
    {
        Key_Scan();
        //测试按键触发 单次按下只累加一次
        if(PorcessData.Key[0] == Key_Status_TRIG_FREE_PRESSED)
        {
          count ++;
        }
				
         if (huart2.ErrorCode)
        {
            HAL_UART_DMAStop(&huart2); // 停止以重启
            HAL_UARTEx_ReceiveToIdle_DMA(&huart2,Bluetooth_RxData,Bluetooth_length);
        }
    }
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
		//蓝牙接收数据
    if (huart->Instance == USART2)
    {
      if(Bluetooth_RxData[0] == 0xAA && Bluetooth_RxData[1] == 0x55)
      {
        Bluetooth_Data_Process(Bluetooth_RxData);
      }
      HAL_UARTEx_ReceiveToIdle_DMA(&huart2,Bluetooth_RxData,Bluetooth_length);
    }
		
		//上位机接收数据
		if (huart->Instance == USART1){
			if(UpCom_RxData_Buffer[0] == 0x2a && UpCom_RxData_Buffer[UpCom_length - 1] == 0x3b){
				UpCom_Data_Process(UpCom_RxData_Buffer);//数据处理
				UpCom_Clear(UpCom_RxData_Buffer);//清楚缓冲区（解决因为缓冲区数据符合条件数据发了两次的问题）
				HAL_UART_Transmit_DMA(&huart1, temp, 4);//向上位机发送数据（不完善）
				HAL_UARTEx_ReceiveToIdle_DMA(&huart1,UpCom_RxData_Buffer,UpCom_length);//DMA空闲中断
			} 
		}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	//启动定时器
  HAL_TIM_Base_Start_IT(&htim2);//用来时不时检测按键
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2,Bluetooth_RxData,Bluetooth_length);//DMA空闲中断，表示数据已经接收搬运完毕
	
	//电机PWM启动
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);	//左前
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); //右前
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); //左后
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); //右后
	
	//舵机启动
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); 
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	
	//数值初始化
	mc_wheels.pct_omega = 0.0f;//omega初始化
	
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty_S1);//2000-3500  *实际上和旋转的方向有关，精度不高* //3500ok，中间3cm左右的样子；3600的时候差1cm；3700闭合（不知道是否刚好）；3650刚好；
	HAL_Delay(250);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty_S2);
	HAL_Delay(250);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty_S3);//这里后面两个初始化的角度还需要再次测试
	HAL_Delay(250);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, duty_S4);
	HAL_Delay(250);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//总开关
		if (PorcessData.Key[11] == Key_Status_PRESSED){
			
			//舵机的位置控制
			Amon_Serve_Control();
			
			//判断模式切换速度挡位
			if (PorcessData.Key[8] == Key_Status_PRESSED){
				Max_Speed = 99;
				Max_Speed_w = 80;
			} 
			if (PorcessData.Key[9] == Key_Status_PRESSED){
				Max_Speed = 70;
				Max_Speed_w = 73;
			}
			if (PorcessData.Key[10] == Key_Status_PRESSED){
				Max_Speed = 58;
				Max_Speed_w = 67;
			}		
			//计算轮子速度；
			DUTY_Wheels[1] = PorcessData.Remote_Left_Y * Max_Speed + PorcessData.Remote_Right_X * Max_Speed_w;
			DUTY_Wheels[2] = PorcessData.Remote_Left_Y * Max_Speed - PorcessData.Remote_Right_X * Max_Speed_w;
			DUTY_Wheels[3] = PorcessData.Remote_Left_Y * Max_Speed + PorcessData.Remote_Right_X * Max_Speed_w;
			DUTY_Wheels[4] = PorcessData.Remote_Left_Y * Max_Speed - PorcessData.Remote_Right_X * Max_Speed_w;
			
			//限制轮子速度不要超过99；
			if (fabs(DUTY_Wheels[1]) > 99){
				if(DUTY_Wheels[1] > 4){
					DUTY_Wheels[1] = 99;
				} else if(DUTY_Wheels[1] < -4){
					DUTY_Wheels[1] = -99;
				}
			}
			if (fabs(DUTY_Wheels[2]) > 99){
				if(DUTY_Wheels[2] > 4){
					DUTY_Wheels[2] = 99;
				} else if(DUTY_Wheels[2] < -4){
					DUTY_Wheels[2] = -99;
				}
			}
			if (fabs(DUTY_Wheels[3]) > 99){
				if(DUTY_Wheels[3] > 4){
					DUTY_Wheels[3] = 99;
				} else if(DUTY_Wheels[1] < -4){
					DUTY_Wheels[3] = -99;
				}
			}
			if (fabs(DUTY_Wheels[4]) > 99){
				if(DUTY_Wheels[4] > 4){
					DUTY_Wheels[4] = 99;
				} else if(DUTY_Wheels[1] < -4){
					DUTY_Wheels[4] = -99;
				}
			}
			
			//控制转动方向；
			
			//轮子1（左前轮）判断转向
			if (DUTY_Wheels[1] < -4){//这个位置之前反了
				HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, GPIO_PIN_RESET);
			}else if (DUTY_Wheels[1] > 4){
				HAL_GPIO_WritePin(M1_IN1_GPIO_Port,  M1_IN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, GPIO_PIN_SET);
			}else{
				HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, GPIO_PIN_SET);
			}
			
			//轮子2（右前轮）判断转向
			if (DUTY_Wheels[2] > 4){
				HAL_GPIO_WritePin(M2_IN1_GPIO_Port, M2_IN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(M2_IN2_GPIO_Port, M2_IN2_Pin, GPIO_PIN_RESET);
			}else if (DUTY_Wheels[2] < -4){
				HAL_GPIO_WritePin(M2_IN1_GPIO_Port, M2_IN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(M2_IN2_GPIO_Port, M2_IN2_Pin, GPIO_PIN_SET);
			}else{
				HAL_GPIO_WritePin(M2_IN1_GPIO_Port, M2_IN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(M2_IN2_GPIO_Port, M2_IN2_Pin, GPIO_PIN_SET);
			}

			//轮子3（左后轮）判断转向	
			if (DUTY_Wheels[3] < -4){
				HAL_GPIO_WritePin(M3_IN1_GPIO_Port, M3_IN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(M3_IN2_GPIO_Port, M3_IN2_Pin, GPIO_PIN_RESET);
			}else if (DUTY_Wheels[3] > 4){
				HAL_GPIO_WritePin(M3_IN1_GPIO_Port, M3_IN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(M3_IN2_GPIO_Port, M3_IN2_Pin, GPIO_PIN_SET);
			}else{
				HAL_GPIO_WritePin(M3_IN1_GPIO_Port, M3_IN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(M3_IN2_GPIO_Port, M3_IN2_Pin, GPIO_PIN_SET);
			}
			
			//轮子4（右后轮）判断转向
			if (DUTY_Wheels[4] > 4){
				HAL_GPIO_WritePin(M4_IN1_GPIO_Port, M4_IN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(M4_IN2_GPIO_Port, M4_IN2_Pin, GPIO_PIN_RESET);
			}else if (DUTY_Wheels[4] < -4){
				HAL_GPIO_WritePin(M4_IN1_GPIO_Port, M4_IN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(M4_IN2_GPIO_Port, M4_IN2_Pin, GPIO_PIN_SET);
			}else{
				HAL_GPIO_WritePin(M4_IN1_GPIO_Port, M4_IN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(M4_IN2_GPIO_Port, M4_IN2_Pin, GPIO_PIN_SET);
			}
			
			//控制四个电机的占空比
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, fabs(DUTY_Wheels[1]));
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, fabs(DUTY_Wheels[2]));
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, fabs(DUTY_Wheels[3]));
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, fabs(DUTY_Wheels[4]));
			
			if (PorcessData.Key[14] == Key_Status_TRIG_FREE_PRESSED){//放平准备抓取
				
				//夹爪位移
				if (duty_S1 < 1240){
					for (int i = duty_S1; i <= 1240; i++){
						duty_S1 += 1;
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty_S1);	
						HAL_Delay(1);
					}
				} else if(duty_S1 > 1244){
					for (int i = duty_S1; i >= 1244; i--){
						duty_S1 -= 1;
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty_S1);	
						HAL_Delay(1);
					}
				}
				
				//小臂位移
				if (duty_S3 < 3331){
					for (int i = duty_S3; i <= 3331; i++){
						duty_S3 += 1;
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty_S3);	
						HAL_Delay(1);
					}
				} else if(duty_S3 > 3335){
					for (int i = duty_S3; i >= 3335; i--){
						duty_S3 -= 1;
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty_S3);	
						HAL_Delay(1);
					}
				}
				
				//大臂位移
				if (duty_S4 < 1791){
					for (int i = duty_S4; i <= 1791; i++){
						duty_S4 += 1;
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, duty_S4);	
						HAL_Delay(1);
					}
				} else if(duty_S4 > 1795){
					for (int i = duty_S4; i >= 1795; i--){
						duty_S4 -= 1;
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, duty_S4);	
						HAL_Delay(1);
					}
				}
				
				for (int i = 90; i >= 65; i--){
						
					//向前小冲
					HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, GPIO_PIN_SET);
					
					HAL_GPIO_WritePin(M2_IN1_GPIO_Port, M2_IN1_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(M2_IN2_GPIO_Port, M2_IN2_Pin, GPIO_PIN_RESET);
					
					HAL_GPIO_WritePin(M3_IN1_GPIO_Port, M3_IN1_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(M3_IN2_GPIO_Port, M3_IN2_Pin, GPIO_PIN_SET);

					HAL_GPIO_WritePin(M4_IN1_GPIO_Port, M4_IN1_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(M4_IN2_GPIO_Port, M4_IN2_Pin, GPIO_PIN_RESET);
					
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, i);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, i);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, i);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, i);
					
					HAL_Delay(20);
				}
				
				for (int i = 90; i >= 65; i--){
					
					//往后小退
					HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, GPIO_PIN_RESET);
					
					HAL_GPIO_WritePin(M2_IN1_GPIO_Port, M2_IN1_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(M2_IN2_GPIO_Port, M2_IN2_Pin, GPIO_PIN_SET);
					
					HAL_GPIO_WritePin(M3_IN1_GPIO_Port, M3_IN1_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(M3_IN2_GPIO_Port, M3_IN2_Pin, GPIO_PIN_RESET);

					HAL_GPIO_WritePin(M4_IN1_GPIO_Port, M4_IN1_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(M4_IN2_GPIO_Port, M4_IN2_Pin, GPIO_PIN_SET);
					
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, i);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, i);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, i);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, i);
					
					HAL_Delay(6);
				}
				
				//停止
				HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, GPIO_PIN_SET);
				
				HAL_GPIO_WritePin(M2_IN1_GPIO_Port, M2_IN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(M2_IN2_GPIO_Port, M2_IN2_Pin, GPIO_PIN_SET);
					
				HAL_GPIO_WritePin(M3_IN1_GPIO_Port, M3_IN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(M3_IN2_GPIO_Port, M3_IN2_Pin, GPIO_PIN_SET);

				HAL_GPIO_WritePin(M4_IN1_GPIO_Port, M4_IN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(M4_IN2_GPIO_Port, M4_IN2_Pin, GPIO_PIN_SET);
				
				//小臂位移
				if (duty_S3 < 4032){
					for (int i = duty_S3; i <= 4032; i++){
						duty_S3 += 1;
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty_S3);	
						HAL_Delay(1);
					}
				} else if(duty_S3 > 4036){
					for (int i = duty_S3; i >= 4036; i--){
						duty_S3 -= 1;
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty_S3);	
						HAL_Delay(1);
					}
				}
				
				//位移夹爪
				if (duty_S1 < 2148){
					for (int i = duty_S1; i <= 2148; i++){
						duty_S1 += 1;
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty_S1);	
						HAL_Delay(1);
					}
				} else if(duty_S1 > 2152){
					for (int i = duty_S1; i >= 2152; i--){
						duty_S1 -= 1;
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty_S1);	
						HAL_Delay(1);
					}
				}
				
				//小臂位移
				if (duty_S3 < 3331){
					for (int i = duty_S3; i <= 3331; i++){
						duty_S3 += 1;
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty_S3);	
						HAL_Delay(1);
					}
				} else if(duty_S3 > 3335){
					for (int i = duty_S3; i >= 3335; i--){
						duty_S3 -= 1;
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty_S3);	
						HAL_Delay(1);
					}
				}
				
				//大臂位移
				if (duty_S4 < 1791){
					for (int i = duty_S4; i <= 1791; i++){
						duty_S4 += 1;
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, duty_S4);	
						HAL_Delay(1);
					}
				} else if(duty_S4 > 1795){
					for (int i = duty_S4; i >= 1795; i--){
						duty_S4 -= 1;
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, duty_S4);	
						HAL_Delay(1);
					}
				}
				
				continue;
				
			}else if (PorcessData.Key[13] == Key_Status_TRIG_FREE_PRESSED){//位移到过狗洞模式
				
				//大臂上抬升
				if (duty_S4 < 1790){
					for (int i = duty_S4; i <= 1790; i++){
						duty_S4 += 1;
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, duty_S4);	
						HAL_Delay(1);
					}
				} else if(duty_S4 > 1794){
					for (int i = duty_S4; i >= 1794; i--){
						duty_S4 -= 1;
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, duty_S4);	
						HAL_Delay(1);
					}
				}
				
				//小臂下压
				if (duty_S3 < 3766){
					for (int i = duty_S3; i <= 3766; i++){
						duty_S3 += 1;
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty_S3);	
						HAL_Delay(1);
					}
				} else if(duty_S3 > 3770){
					for (int i = duty_S3; i >= 3770; i--){
						duty_S3 -= 1;
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty_S3);	
						HAL_Delay(1);
					}
				}
				

				continue;
				
			} else if (PorcessData.Key[12] == Key_Status_TRIG_FREE_PRESSED) {//位移到稳定抓取模式
				
				//大臂下压
				if (duty_S4 < 2833){
					for (int i = duty_S4; i <= 2833; i++){
						duty_S4 += 1;
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, duty_S4);	
						HAL_Delay(1);
					}
				} else if(duty_S4 > 2837){
					for (int i = duty_S4; i >= 2837; i--){
						duty_S4 -= 1;
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, duty_S4);	
						HAL_Delay(1);
					}
				}
				
				//小臂下压
				if (duty_S3 < 3030){
					for (int i = duty_S3; i <= 3030; i++){
						duty_S3 += 1;
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty_S3);	
						HAL_Delay(1);
					}
				} else if(duty_S3 > 3034){
					for (int i = duty_S3; i >= 3034; i--){
						duty_S3 -= 1;
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty_S3);	
						HAL_Delay(1);
					}
				}
				
				
				continue;
				
			} 
			
			else if (PorcessData.Key[15] == Key_Status_TRIG_FREE_PRESSED){//投矿抓法
				
				//张开夹爪
				if (duty_S1 < 1440){
					for (int i = duty_S1; i <= 1441; i++){
						duty_S1 += 1;
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty_S1);	
						HAL_Delay(1);
					}
				} else if(duty_S1 > 1444){
					for (int i = duty_S1; i >= 1444; i--){
						duty_S1 -= 1;
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty_S1);	
						HAL_Delay(1);
					}
				}
				
				//小臂下压
				if (duty_S3 < 4115){
					for (int i = duty_S3; i <= 4115; i++){
						duty_S3 += 1;
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty_S3);	
						HAL_Delay(1);
					}
				} else if(duty_S3 > 4119){
					for (int i = duty_S3; i >= 4119; i--){
						duty_S3 -= 1;
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty_S3);	
						HAL_Delay(1);
					}
				}
				
				//大臂下压
				if (duty_S4 < 1791){
					for (int i = duty_S4; i <= 1791; i++){
						duty_S4 += 1;
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, duty_S4);	
						HAL_Delay(1);
					}
				} else if(duty_S4 > 1795){
					for (int i = duty_S4; i >= 1795; i--){
						duty_S4 -= 1;
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, duty_S4);	
						HAL_Delay(1);
					}
				}
				
				continue;
				
			} else if (1){
				//设置四个舵机的占空比
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty_S1); 
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty_S2);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty_S3);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, duty_S4);
			}
			
			HAL_Delay(2);		
			
		}//启动结束
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//舵机角度计算
void Amon_Serve_Control(){
	//爪
	if (PorcessData.Key[1] == Key_Status_PRESSED && duty_S1 <= 2150){
		duty_S1 += 3;
	}
	if (PorcessData.Key[2] == Key_Status_PRESSED && duty_S1 >= 1000){
		duty_S1 -= 3;
	}
	
	//转
	if (PorcessData.Key[4] == Key_Status_PRESSED && duty_S2 <= 4500){
		duty_S2 += 4;
	}
	if (PorcessData.Key[5] == Key_Status_PRESSED && duty_S2 >= 1500){
		duty_S2 -= 4;
	}
			
	//角度1
	if (PorcessData.Key[3] == Key_Status_PRESSED && duty_S3 <= 5000){
		duty_S3 += 4;
	}
	if (PorcessData.Key[0] == Key_Status_PRESSED && duty_S3 >= 1000){
		duty_S3 -= 4;
	}
			
	//抬升	
	if (PorcessData.Key[7] == Key_Status_PRESSED && duty_S4 <= 5000){ 
		duty_S4 += 4;
	}
	if (PorcessData.Key[6] == Key_Status_PRESSED && duty_S4 >= 1000){
		duty_S4 -= 4;
	}
	HAL_Delay(2);
}

//急刹车:  四个电机全部设置为高电平
void Amon_AllMotor_SharpBrake(){

		//停止M1马达
		HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, GPIO_PIN_SET);
		
		//停止M2马达
		HAL_GPIO_WritePin(M2_IN1_GPIO_Port, M2_IN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M2_IN2_GPIO_Port, M2_IN2_Pin, GPIO_PIN_SET);
		
		//停止M3马达
		HAL_GPIO_WritePin(M3_IN1_GPIO_Port, M3_IN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M3_IN2_GPIO_Port, M3_IN2_Pin, GPIO_PIN_SET);
		
		//停止M4马达
		HAL_GPIO_WritePin(M4_IN1_GPIO_Port, M4_IN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M4_IN2_GPIO_Port, M4_IN2_Pin, GPIO_PIN_SET);
		
		HAL_Delay(1);		//1毫秒缓一缓
}


//缓慢刹车:  四个电机全部设置为低电平
void Amon_AllMotor_SlowBrake(){

		//停止M1马达
		HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, GPIO_PIN_RESET);
		
		//停止M2马达
		HAL_GPIO_WritePin(M2_IN1_GPIO_Port, M2_IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M2_IN2_GPIO_Port, M2_IN2_Pin, GPIO_PIN_RESET);
		
		//停止M3马达
		HAL_GPIO_WritePin(M3_IN1_GPIO_Port, M3_IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M3_IN2_GPIO_Port, M3_IN2_Pin, GPIO_PIN_RESET);
		
		//停止M4马达
		HAL_GPIO_WritePin(M4_IN1_GPIO_Port, M4_IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M4_IN2_GPIO_Port, M4_IN2_Pin, GPIO_PIN_RESET);
		
		HAL_Delay(1);		//1毫秒缓一缓
}

//将手柄的速度解算到底盘四个轮子上
void Amon_Chassis_Calculate_Contrary(Mc_Wheels_Typedef* Mc_wheels, struct Struct_Bluetooth_PorcessData* PorcessData){
	
	Mc_wheels->pct_omega = PorcessData->Remote_Right_X * 400;//读取手柄的值计算旋转角速度的比例
	float x_circle = PorcessData->Remote_Left_X;//映射到圆后的x值
	float y_circle = PorcessData->Remote_Left_Y;//映射到圆后的y值
	if (fabsf(x_circle) < 0.01f && fabsf(y_circle) < 0.01f) {//防止松手任然计算角度产生过大的值
    Mc_wheels->angle = 0.0f;
	} else {
		Mc_wheels->angle = atan(y_circle / x_circle);//计算手柄指向角度
	}
	Mc_wheels->divisor = 1 + 0.4 * fabs(sin(2 * Mc_wheels->angle));//除数（解决四角值过大的问题）
	//计算比例
	Mc_wheels->pct_vx = x_circle * 100.0;
	Mc_wheels->pct_vy = y_circle * 100.0;
	//计算每个轮子应有的占空比																								
	Mc_wheels->duty_wheels[1] = (Mc_wheels->pct_vy + Mc_wheels->pct_vx) / Mc_wheels->divisor + Mc_wheels->pct_omega * (DX + DY);
	Mc_wheels->duty_wheels[2] = (Mc_wheels->pct_vy - Mc_wheels->pct_vx) / Mc_wheels->divisor - Mc_wheels->pct_omega * (DX + DY);	
	Mc_wheels->duty_wheels[3] = (Mc_wheels->pct_vy - Mc_wheels->pct_vx) / Mc_wheels->divisor + Mc_wheels->pct_omega * (DX + DY);
	Mc_wheels->duty_wheels[4] = (Mc_wheels->pct_vy + Mc_wheels->pct_vx) / Mc_wheels->divisor - Mc_wheels->pct_omega * (DX + DY);
}

//手柄控制转动方向
void Amon_Direct_control(Mc_Wheels_Typedef* mc_wheels){
	
	//轮子1（左前轮）判断转向
	if (mc_wheels->duty_wheels[1] < 0x02){
		HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, GPIO_PIN_RESET);
	}else if (mc_wheels->duty_wheels[1] > -0x02){
		HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, GPIO_PIN_RESET);
	}
	
	//轮子2（右前轮）判断转向
	if (mc_wheels->duty_wheels[2] > 0x02){
		HAL_GPIO_WritePin(M2_IN1_GPIO_Port, M2_IN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M2_IN2_GPIO_Port, M2_IN2_Pin, GPIO_PIN_RESET);
	}else if (mc_wheels->duty_wheels[2] < -0x02){
		HAL_GPIO_WritePin(M2_IN1_GPIO_Port, M2_IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M2_IN2_GPIO_Port, M2_IN2_Pin, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(M2_IN1_GPIO_Port, M2_IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M2_IN2_GPIO_Port, M2_IN2_Pin, GPIO_PIN_RESET);
	}

	//轮子3（左后轮）判断转向	
	if (mc_wheels->duty_wheels[3] < 0x02){
		HAL_GPIO_WritePin(M3_IN1_GPIO_Port, M3_IN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M3_IN2_GPIO_Port, M3_IN2_Pin, GPIO_PIN_RESET);
	}else if (mc_wheels->duty_wheels[3] > -0x02){
		HAL_GPIO_WritePin(M3_IN1_GPIO_Port, M3_IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M3_IN2_GPIO_Port, M3_IN2_Pin, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(M3_IN1_GPIO_Port, M3_IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M3_IN2_GPIO_Port, M3_IN2_Pin, GPIO_PIN_RESET);
	}
	
	//轮子4（右后轮）判断转向
	if (mc_wheels->duty_wheels[4] > 0x02){
		HAL_GPIO_WritePin(M4_IN1_GPIO_Port, M4_IN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M4_IN2_GPIO_Port, M4_IN2_Pin, GPIO_PIN_RESET);
	}else if (mc_wheels->duty_wheels[4] < -0x02){
		HAL_GPIO_WritePin(M4_IN1_GPIO_Port, M4_IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M4_IN2_GPIO_Port, M4_IN2_Pin, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(M4_IN1_GPIO_Port, M4_IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M4_IN2_GPIO_Port, M4_IN2_Pin, GPIO_PIN_RESET);
	}
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
