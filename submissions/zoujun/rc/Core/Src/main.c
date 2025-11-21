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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ax_ps2.h"
#include "motor.h"
#include "mecanum.h"
#include "arm.h"
#include "ps2_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// 电机速度定义
#define FORWARD_SPEED 800
#define TURN_SPEED 600
// 模式定义
typedef enum {
    MODE_MANUAL = 0,    // PS2手动控制模式
    MODE_AUTO_LINE = 1  // 自动巡线模式
}OperationMode;

OperationMode current_mode = MODE_MANUAL;

#define UART_TIMEOUT_MS 500

// 底盘控制变量
Motor_t motor1, motor2, motor3, motor4;
WheelSpeeds_t wheel_speeds;
MotionVector_t chassis_motion;
RobotArm_t robot_arm ;
// 串口命令
UART_Command_t uart_cmd;
uint32_t last_uart_command_time = 0;
// PS2按键定义（均在btn2中）
#define PS2_TRI   0x10    // △键（舵机1向上）
#define PS2_CIR   0x20    // ○键（舵机1向下）
#define PS2_X     0x40    // ×键（舵机2向上）
#define PS2_SQR   0x80    // □键（舵机2向下）
#define PS2_L1    0x01    // L1键（舵机3向右）
#define PS2_R1    0x02    // R1键（舵机3向左）
#define PS2_L2    0x04    // L2键（舵机4向上）
#define PS2_R2    0x08    // R2键（舵机4向下）

// 舵机角度变量（0~180°）
uint16_t servo1_angle = 90;  // 大臂初始角度（中位）
uint16_t servo2_angle = 90;  // 小臂初始角度（中位）
uint16_t servo3_angle = 90;  // 基座初始角度（中位）
uint16_t servo4_angle = 90;  // 机械爪初始角度（中位）

// 舵机角度限制（防止超程）
#define SERVO1_MIN 30    // 大臂最小角度
#define SERVO1_MAX 60  // 大臂最大角度
#define SERVO2_MIN 30    // 小臂最小角度
#define SERVO2_MAX 150   // 小臂最大角度
#define SERVO3_MIN 0     // 基座最小角度
#define SERVO3_MAX 180   // 基座最大角度
#define SERVO4_MIN 30    // 机械爪最小角度
#define SERVO4_MAX 150   // 机械爪最大角度
#define FORWARD_SPEED 800
#define TURN_SPEED 600
// 常量定义
#define MODE_SWITCH_DEBOUNCE 300


// 串口通信相关变量
volatile UART_Command_t rx_command = {0};
volatile UART_Feedback_t tx_feedback = {0};
volatile uint8_t uart_rx_buffer[RX_LENGTH] = {0};
volatile uint8_t uart_rx_index = 0;
volatile uint8_t frame_ready = 0;
//UART_HandleTypeDef huart1;
//TIM_HandleTypeDef htim2;

JOYSTICK_TypeDef joystick;
/* 函数声明 */
uint8_t Is_Start_Button_Pressed(void);
void PS2_Control_Chassis(void);
void Auto_Line_Following(void);
void Safety_Check(void);


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 小车控制函数
void Car_Forward(void) {
    Motor_SetSpeed(&motor1, +FORWARD_SPEED);
    Motor_SetSpeed(&motor2, +FORWARD_SPEED);
    Motor_SetSpeed(&motor3, -FORWARD_SPEED);
    Motor_SetSpeed(&motor4, -FORWARD_SPEED);
}
void Car_TurnRight(void) {
    Motor_SetSpeed(&motor1, -TURN_SPEED);
    Motor_SetSpeed(&motor2, +TURN_SPEED);
    Motor_SetSpeed(&motor3, TURN_SPEED);
    Motor_SetSpeed(&motor4, -TURN_SPEED);
}
void Car_TurnLeft(void) {
    Motor_SetSpeed(&motor1, TURN_SPEED);
    Motor_SetSpeed(&motor2, -TURN_SPEED);
    Motor_SetSpeed(&motor3, -TURN_SPEED);
    Motor_SetSpeed(&motor4, TURN_SPEED);
}
void Car_Stop(void) {
    Motor_Stop(&motor1);
    Motor_Stop(&motor2);
    Motor_Stop(&motor3);
    Motor_Stop(&motor4);
}
/* PS2手动控制（集成底盘和机械臂） */
void PS2_Manual_Control(void)
{
    // 获取控制模式（SELECT键切换）
    PS2ControlMode_t ctrl_mode = PS2_GetControlMode(joystick.btn1);
    
    // 底盘控制
    if(ctrl_mode == CONTROL_MODE_ANALOG) {
        // 摇杆模拟控制
        PS2_Analog_Control(&joystick, &wheel_speeds);
    } else {
        // 按键数字控制
        PS2_Digital_Control(joystick.btn1, &wheel_speeds);
    }
    
    // 应用底盘速度
    Mecanum_ApplySpeeds(&wheel_speeds, &motor1, &motor2, &motor3, &motor4);
    
    // 机械臂控制（使用btn2，无冲突）
	}

// UART接收处理函数
void UART_Receive_Handler(void) {
    if (frame_ready && current_mode== MODE_AUTO_LINE) {
        // 解析接收到的指令
        memcpy((void*)&rx_command, (void*)(uart_rx_buffer + 1), sizeof(UART_Command_t));
        
        // 根据指令控制小车运动
        if (rx_command.flag_forward) {
            if (rx_command.flag_left && !rx_command.flag_right) {
                Car_TurnLeft();
            } else if (rx_command.flag_right && !rx_command.flag_left) {
                Car_TurnRight();
            } else {
                Car_Forward();
            }
        } else {
            Car_Stop();
        }
        
        // 重置接收状态
        frame_ready = 0;
        uart_rx_index = 0;
        memset((void*)uart_rx_buffer, 0, RX_LENGTH);
        
        // 继续接收
        HAL_UART_Receive_IT(&huart1, (uint8_t*)&uart_rx_buffer[uart_rx_index], 1);
    } else if (frame_ready) {
        // 手动模式下收到数据，忽略但重置状态
        frame_ready = 0;
        uart_rx_index = 0;
        memset((void*)uart_rx_buffer, 0, RX_LENGTH);
        HAL_UART_Receive_IT(&huart1, (uint8_t*)&uart_rx_buffer[uart_rx_index], 1);
    }
}
// 串口接收中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        uint8_t received_byte = uart_rx_buffer[uart_rx_index];
        // 帧头检测
        if (uart_rx_index == 0) {
            if (received_byte != FRAME_HEAD) {
                uart_rx_index = 0;
                HAL_UART_Receive_IT(&huart1, (uint8_t*)&uart_rx_buffer[uart_rx_index], 1);
                return;
            }
        }
        uart_rx_index++;
        // 检查是否收到完整帧
        if (uart_rx_index >= RX_LENGTH) {
            if (uart_rx_buffer[RX_LENGTH-1] == FRAME_TAIL) {
                frame_ready = 1;
            }
            uart_rx_index = 0;
        }
        // 继续接收下一个字节
        HAL_UART_Receive_IT(&huart1, (uint8_t*)&uart_rx_buffer[uart_rx_index], 1);
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
\
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	
//MX_GPIO_Init();
//    MX_TIM2_Init();  // 电机PWM
//    MX_TIM1_Init();  // 舵机1
//    MX_TIM4_Init();  // 舵机2,3,4
//    MX_USART1_UART_Init();

    /* USER CODE BEGIN 2 */
    // 初始化四个电机 - PB2改为PB9
    Motor_Init(&motor1, &htim2, TIM_CHANNEL_1, GPIOA, GPIO_PIN_4, GPIOA, GPIO_PIN_5);
    Motor_Init(&motor2, &htim2, TIM_CHANNEL_2, GPIOA, GPIO_PIN_6, GPIOA, GPIO_PIN_7);
    Motor_Init(&motor3, &htim2, TIM_CHANNEL_3, GPIOB, GPIO_PIN_0, GPIOB, GPIO_PIN_1);
    Motor_Init(&motor4, &htim2, TIM_CHANNEL_4, GPIOB, GPIO_PIN_5, GPIOB, GPIO_PIN_3);  // PB2改为PB9

// 启动舵机PWM（重要！）
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // 舵机1
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);  // 舵机2
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);  // 舵机3  
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);  // 舵机4
 // 启动PWM
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
        // 启动串口接收中断
    uart_rx_index = 0;
    HAL_UART_Receive_IT(&huart1, (uint8_t*)&uart_rx_buffer[uart_rx_index], 1);
// 初始化机械臂
Arm_Init(&robot_arm);

       PS2_Control_Init();
			 HAL_Delay(1000);
		 
    // 初始停止所有电机
    Mecanum_StopAll(&motor1, &motor2, &motor3, &motor4);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		AX_PS2_ScanKey(&joystick);
		 PS2_Auto_Reconnect();  // 添加自动重连
    printf("Mode:0x%02X BTN1:0x%02X BTN2:0x%02X RJ(%3d,%3d) LJ(%3d,%3d)\r\n",
        joystick.mode, joystick.btn1, joystick.btn2,
        joystick.RJoy_LR, joystick.RJoy_UD,
        joystick.LJoy_LR, joystick.LJoy_UD);
		delay_us(9);
				
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	// 检查START键是否按下（模式切换）
        if(Is_Start_Button_Pressed()) {
            current_mode = (current_mode == MODE_MANUAL) ? MODE_AUTO_LINE : MODE_MANUAL;
            
            if(current_mode == MODE_AUTO_LINE) {
                // 切换到自动模式时停止底盘，等待上位机指令
                Mecanum_StopAll(&motor1, &motor2, &motor3, &motor4);
                last_uart_command_time = HAL_GetTick();
            }
        }
       
        // 根据模式执行控制
        if(current_mode == MODE_MANUAL) {
            // 手动模式：PS2控制底盘移动 + 机械臂控制
  // 手动模式
            PS2_Manual_Control();
 Arm_Control_With_PS2(&robot_arm, joystick.btn2); 
     
        } else {
           UART_Receive_Handler ();
        }
  
        // 安全检查
        Safety_Check();
        
        HAL_Delay(20);


			}

		} 

  /* USER CODE END 3 */


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

/* 检查START键是否按下 */
uint8_t Is_Start_Button_Pressed(void)
{
    static uint32_t last_press_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    if(joystick.btn1 == 0x08) {
        if(current_time - last_press_time > MODE_SWITCH_DEBOUNCE) {
            last_press_time = current_time;
            return 1;
        }
    }
    return 0;
}




/* 安全检查 */
void Safety_Check(void)
{
    if(current_mode == MODE_AUTO_LINE) {
        if(HAL_GetTick() - last_uart_command_time > UART_TIMEOUT_MS) {
            Mecanum_StopAll(&motor1, &motor2, &motor3, &motor4);
        }
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
		  // 错误时快速闪烁LED
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    HAL_Delay(100);

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
