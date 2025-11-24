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
#include "servo_tim3.h"
#include "MotorDriver.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 与上位机的指令协议（11字节定长，含校验和）
#define FRAME_LEN       11          // 帧总长度：帧头2字节+ 4电机速度(4×2=8字节) + 校验和1字节
#define FRAME_HEAD1     0xAA        // 帧头1
#define FRAME_HEAD2     0x55        // 帧头2
#define CHECKSUM_INDEX  10          // 校验和所在字节索引（最后1字节）

//按键宏定义
#define BTN_UP 1
#define BTN_DOWN 2
#define BTN_LEFT 3
#define BTN_RIGHT 4

#define MOT_L_F 1 //左前电机编号
#define MOT_L_B 2
#define MOT_R_F 3
#define MOT_R_B 4

#define DIR_NONE 0
#define DIR_FORWORD 1
#define DIR_BACK 2
#define DIR_LEFT 3
#define DIR_RIGHT 4
#define DIR_CLOCKWISE 5
#define DIR_ANTICLOCKWISE 6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float remote_lx=0.0,remote_ly=0.0,remote_lz=0.0;
//volatile uint8_t Flag = 0;//0-没接收到数据 1-收到了
//volatile int minipc_alive_flag =0,Pre_minipc_alive_flag;
volatile uint8_t count;
//舵机
volatile int angle_0=0;
volatile int angle_1=0;
volatile int angle_2=0;
volatile int angle_3=0;
//电机
volatile float m1, m2, m3, m4;
extern uint8_t Bluetooth_RxData[Bluetooth_length];
extern struct Struct_Bluetooth_RxData RxData;
extern struct Struct_Bluetooth_PorcessData PorcessData;
volatile uint8_t  Usart2DmaRxBuf[FRAME_LEN];  // USART2 DMA接收缓冲区
MotorDriver mot[5];
uint8_t main_dir=0;
uint8_t main_dir1=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t Calc_Checksum(volatile uint8_t *buf, uint16_t len);    // 计算校验和
uint8_t GetRemoteDirection(float x,float y)
{
	if(x*x+y*y<=0.25*0.25) return DIR_NONE;
	if(y>0.0)
	{
		if(-0.707<=x && x<=0.707) return DIR_FORWORD;
		else if(x>0.707) return DIR_RIGHT;
		else return DIR_LEFT;
	}
	if(y<=0.0)
	{
		if(-0.707<=x && x<=0.707) return DIR_BACK;
		else if(x>0.707) return DIR_RIGHT;
		else return DIR_LEFT;
	}
}

uint8_t Getifclockwise(float z)
	{
		if(z>0.3) return DIR_CLOCKWISE;
	  if(z<-0.3)return DIR_ANTICLOCKWISE;
  }
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        Key_Scan();
        //测试按键触发 单次按下只累加一次
			
        if(PorcessData.Key[4] == Key_Status_TRIG_FREE_PRESSED)
        {
          count ++;
        }
				if(PorcessData.Key[1] == Key_Status_TRIG_FREE_PRESSED)
        {      
					     angle_1 +=5; 
					     if(angle_1 > 15)
							 {
							     angle_1 = 0;
							 }
               Servo_TIM3_SetAngle(TIM_CHANNEL_1, angle_1);
        }
			
				if(PorcessData.Key[2] == Key_Status_TRIG_FREE_PRESSED)
        {      
					     angle_2 += 5;
					     if(angle_2 >15)
							 {
							     angle_2 = 0;
							 }
               Servo_TIM3_SetAngle(TIM_CHANNEL_2, angle_2);
        }

				if(PorcessData.Key[3] == Key_Status_TRIG_FREE_PRESSED)
        {      
					     angle_3 += 5;
					     if(angle_3 >15)
							 {
							     angle_3 = 0;
							 }
               Servo_TIM3_SetAngle(TIM_CHANNEL_3, angle_3);
        }
				
				if(PorcessData.Key[0] == Key_Status_TRIG_FREE_PRESSED)
        {      
					     angle_0 += 15;
					     if(angle_0 > 60)
							 {
							     angle_0 = 0;
							 }                       
               Servo_TIM3_SetAngle(TIM_CHANNEL_4, angle_0);
        }
         if (huart1.ErrorCode)
        {
            HAL_UART_DMAStop(&huart1);                        // 停止以重启
            HAL_UARTEx_ReceiveToIdle_DMA(&huart1,Bluetooth_RxData,Bluetooth_length);
        }
		/*		
				static int mod = 0;   //50HZ 
				mod ++;
				if(mod > 20)          //1-20
				{
					mod = 0;
				}
				
				if(mod == 20)
				{
				if(minipc_alive_flag == Pre_minipc_alive_flag)
				{
					Flag = 0;
				}
				else
				{
					Flag =1;
				}
				Pre_minipc_alive_flag = minipc_alive_flag;
				}                                                           */
    }
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
      if(Bluetooth_RxData[0] == 0xAA && Bluetooth_RxData[1] == 0x55)
      {
         Bluetooth_Data_Process(Bluetooth_RxData);
         
         remote_lx=PorcessData.Remote_Left_X;
				 remote_ly=PorcessData.Remote_Left_Y;
				 remote_lz=PorcessData.Remote_Right_X;
				 main_dir=GetRemoteDirection(remote_lx,remote_ly);
				 main_dir1=Getifclockwise(remote_lz);
				switch(main_dir)
				{
					case DIR_FORWORD:
					{
						for(int i=1;i<=4;i++)
						{
							MotorDriverSetDirection(&mot[i],MOTOR_DIR_CW);
							MotorDriverSetSpeed(&mot[i],remote_ly);
						}
						break;
					}
					case DIR_BACK:
					{
						for(int i=1;i<=4;i++)
						{
							MotorDriverSetDirection(&mot[i],MOTOR_DIR_CCW);
							MotorDriverSetSpeed(&mot[i],remote_ly);
						}
						break;
					}
					case DIR_LEFT:
					{
						MotorDriverSetDirection(&mot[MOT_L_F],MOTOR_DIR_CCW);
						MotorDriverSetDirection(&mot[MOT_R_B],MOTOR_DIR_CCW);
						MotorDriverSetDirection(&mot[MOT_L_B],MOTOR_DIR_CW);
						MotorDriverSetDirection(&mot[MOT_R_F],MOTOR_DIR_CW);
						for(int i=1;i<=4;i++)
						{
							MotorDriverSetSpeed(&mot[i],remote_lx);
						}
						break;
					}
					case DIR_RIGHT:
					{
						MotorDriverSetDirection(&mot[MOT_L_F],MOTOR_DIR_CW);
						MotorDriverSetDirection(&mot[MOT_R_B],MOTOR_DIR_CW);
						MotorDriverSetDirection(&mot[MOT_L_B],MOTOR_DIR_CCW);
						MotorDriverSetDirection(&mot[MOT_R_F],MOTOR_DIR_CCW);
					  for(int i=1;i<=4;i++)
						{
							MotorDriverSetSpeed(&mot[i],remote_lx);
						}
						break;
					}
					case DIR_NONE:
					{
						for(int i=1;i<=4;i++)
						{
							MotorDriverStop(&mot[i]);
						}
						break;
					}

				}
				switch(main_dir1)
				{	
					case DIR_CLOCKWISE:
					{
						MotorDriverSetDirection(&mot[MOT_L_F],MOTOR_DIR_CCW);
						MotorDriverSetDirection(&mot[MOT_R_B],MOTOR_DIR_CW);
						MotorDriverSetDirection(&mot[MOT_L_B],MOTOR_DIR_CW);
						MotorDriverSetDirection(&mot[MOT_R_F],MOTOR_DIR_CCW);
						break;
						
					}
					case DIR_ANTICLOCKWISE:
					{
						MotorDriverSetDirection(&mot[MOT_L_F],MOTOR_DIR_CW);
						MotorDriverSetDirection(&mot[MOT_R_B],MOTOR_DIR_CCW);
						MotorDriverSetDirection(&mot[MOT_L_B],MOTOR_DIR_CCW);
						MotorDriverSetDirection(&mot[MOT_R_F],MOTOR_DIR_CW);
						break;
					}
				}
      }
      HAL_UARTEx_ReceiveToIdle_DMA(&huart1,Bluetooth_RxData,Bluetooth_length);
			
		}
/*		else if(huart->Instance == USART2)
		{  
			if(minipc_alive_flag < 0xFFFFFFFF){
            minipc_alive_flag++;
			}
      else{
			      minipc_alive_flag = 0;
			}
							
		}*/
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
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1,Bluetooth_RxData,Bluetooth_length);
	HAL_UART_Receive_DMA(&huart2, (uint8_t*)Usart2DmaRxBuf, sizeof(Usart2DmaRxBuf));
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Base_Start(&htim4);
	//舵机
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	//电机
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	//Motor_Init();
	MotorDriverInit(&mot[MOT_L_F],M1_IN1_PORT,M1_IN1_PIN,M1_IN2_PORT,M1_IN2_PIN,MOTOR_DIR_CCW,0,TIM_CHANNEL_1);
	MotorDriverInit(&mot[MOT_L_B],M2_IN1_PORT,M2_IN1_PIN,M2_IN2_PORT,M2_IN2_PIN,MOTOR_DIR_CCW,0,TIM_CHANNEL_2);
	MotorDriverInit(&mot[MOT_R_F],M3_IN1_PORT,M3_IN1_PIN,M3_IN2_PORT,M3_IN2_PIN,MOTOR_DIR_CCW,0,TIM_CHANNEL_3);
	MotorDriverInit(&mot[MOT_R_B],M4_IN1_PORT,M4_IN1_PIN,M4_IN2_PORT,M4_IN2_PIN,MOTOR_DIR_CCW,0,TIM_CHANNEL_4);
	/*for(int i=1;i<=4;i++)
	{
		MotorDriverSetSpeed(&mot[i],1.00);
	}                                                           */
  /*
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
uint8_t Calc_Checksum(volatile uint8_t *buf, uint16_t len)
{
    uint8_t checksum = 0;
    for(uint16_t i = 0; i < len; i++)
    {
        checksum += buf[i];  // 所有字节累加
    }
    return checksum & 0xFF; // 取低8位作为校验和
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart2) {
           // 校验帧头
        if (Usart2DmaRxBuf[0] == FRAME_HEAD1 && Usart2DmaRxBuf[1] == FRAME_HEAD2)
        {
           // 校验和验证
            uint8_t recv_checksum = Usart2DmaRxBuf[CHECKSUM_INDEX];               // 接收的校验和
            uint8_t calc_checksum = Calc_Checksum(Usart2DmaRxBuf, FRAME_LEN - 1); // 计算前10字节校验和
            
            if (calc_checksum == recv_checksum)              // 校验和一致，解析数据
            {
                // 解析4个电机速度（int16_t类型，-100~100）
                int16_t speed_m1 = (Usart2DmaRxBuf[2] << 8) | Usart2DmaRxBuf[3]; // M1（左前）
                int16_t speed_m2 = (Usart2DmaRxBuf[4] << 8) | Usart2DmaRxBuf[5]; // M2（左后）
                int16_t speed_m3 = (Usart2DmaRxBuf[6] << 8) | Usart2DmaRxBuf[7]; // M3（右前）
                int16_t speed_m4 = (Usart2DmaRxBuf[8] << 8) | Usart2DmaRxBuf[9]; // M4（右后）

                // 转换为电机控制格式（归一化到-1.0~1.0）
                m1 = (float)speed_m1 / 100.0f;
                m2 = (float)speed_m2 / 100.0f;
                m3 = (float)speed_m3 / 100.0f;
                m4 = (float)speed_m4 / 100.0f;

                // 驱动电机
                //在TIM2中断里调用
            }
            // 若校验和不一致：忽略错误指令
        }
				HAL_UART_Receive_DMA(&huart2, (uint8_t*)Usart2DmaRxBuf, sizeof(Usart2DmaRxBuf));
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
