/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  float kp, ki, kd; // PID系数
  float Target;     // 目标值
  float Error;      // 误差值
  float Error_1;    // 上次误差值
  float Integral;   // 积分值
  float Output;     // 输出值
} PID_Angle;        // 角度控制PID

typedef struct
{
  float kp, ki, kd; // PID系数
  float Target;     // 目标值
  float Error;      // 误差值
  float Error_1;    // 上次误差值
  float Integral;   // 积分值
  float Output;     // 输出值
} PID_AngSpeed;     // 角速度控制PID
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RIGHT_STOP 1500 // 右停止
#define LEFT_STOP 1500  // 左停止
#define MAX_SPEED 300   // 最大速度
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
const uint8_t BufferSize = 33;
uint8_t Buffer1[BufferSize];
uint8_t Buffer2[BufferSize];
extern float Acc[3];                                  // XYZ加速度
extern float AngSpeed[3];                             // XYZ角速度
extern float Ang[3];                                  // XYZ角度
char Z0[3] = {0XFF, 0XAA, 0X52};                      // Z轴归零
char ZSET[3] = {0XFF, 0XAA, 0X65};                    // 水平安装
char YSET[3] = {0XFF, 0XAA, 0X66};                    // 垂直安装
char ASET[3] = {0XFF, 0XAA, 0X67};                    // 加计校准
float INIT_SPEED = 200;                               // 初始速度
int AngFlag = 0;                                      // 角度控制标志
int AngSpeedFlag = 0;                                 // 角速度控制标志
PID_Angle AddPID_Angle = {10, 0, 0, 0, 0, 0, 0};      // 角度PID参数
PID_AngSpeed AddPID_AngSpeed = {5, 0, 0, 0, 0, 0, 0}; // 角速度PID参数
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Motor_Init();                                           // 电机初始化
void StandBy(int time);                                      // 停止
void Back();                                                 // 倒车
void Walk(float right_speed, float left_speed);              // 运动
void AlongLine(int time);                                    // 直线运动
void Start();                                                // 加速起步
void Stop();                                                 // 减速停车
void Redirect(float target, int time);                       // 控制到某角度
void AlongCurve(float target, float r, int flag);            // 曲线运动
void Turn(float target, int flag);                           // 转向
void Slide(int N, int speed, int flag);                      // 丝杆滑台
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim); // 中断回调函数
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Paodao(int flag) // 跑道
{
  AlongLine(3000);
  AlongCurve(180, 15, flag);
  AlongLine(3000);
  AlongCurve(0, 15, flag);
}

void Circle(float r, int flag) // 圆形
{
  AlongCurve(180, r, flag);
  AlongCurve(0, r, flag);
}

void Square1() // 直角正方形逆时针
{
  AlongLine(1500);
  Turn(90, 1);
  AlongLine(1500);
  Turn(180, 1);
  AlongLine(1500);
  Turn(-90, 1);
  AlongLine(1500);
  Turn(0, 1);
}

void Square2() // 圆角正方形逆时针
{
  AlongLine(1200);
  AlongCurve(90, 15, 1);
  AlongLine(1200);
  AlongCurve(180, 15, 1);
  AlongLine(1200);
  AlongCurve(-90, 15, 1);
  AlongLine(1200);
  AlongCurve(0, 15, 1);
}

void Eight() // 八字形
{
  Circle(15, 1);
  Circle(15, -1);
}

void Triangle() // 三角形
{
  AlongLine(1500);
  AlongCurve(120, 5, 1);
  AlongLine(1500);
  AlongCurve(-120, 5, 1);
  AlongLine(1500);
  AlongCurve(0, 5, 1);
}

void Wave() // 蛇形
{
  AlongCurve(30, 15, 1);
  AlongCurve(-30, 15, -1);
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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  Motor_Init();
  __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
  HAL_TIM_Base_Start_IT(&htim2);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
  HAL_Delay(1000);
  HAL_UART_Transmit_DMA(&huart1, (uint8_t *)ASET, 3); // 加计校准
  HAL_UART_Transmit_DMA(&huart1, (uint8_t *)ZSET, 3); // 水平安装
  HAL_UART_Transmit_DMA(&huart1, (uint8_t *)Z0, 3);   // Z轴归零
  HAL_Delay(1000);
  StandBy(500);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Start();
    Paodao(1);
    Square2();
    Square2();
    Triangle();
    Triangle();
    AlongLine(500);
    Circle(25, 1);
    Circle(25, 1);
    Stop();
    StandBy(2000);
    Slide(300, 2, 1);
    Slide(300, 2, -1);
    Slide(300, 2, 1);
    Slide(300, 2, -1);
    StandBy(60000);
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
void Motor_Init() // 电机初始化
{
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}

void StandBy(int time) // 停止
{
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, RIGHT_STOP);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, LEFT_STOP);
  HAL_Delay(time);
}

void Back() // 倒车
{
  INIT_SPEED = -INIT_SPEED;
}

void Walk(float right_speed, float left_speed) // 运动
{
  // 设置速度上限
  if (right_speed > MAX_SPEED)
    right_speed = MAX_SPEED;
  if (right_speed < -MAX_SPEED)
    right_speed = -MAX_SPEED;
  if (left_speed > MAX_SPEED)
    left_speed = MAX_SPEED;
  if (left_speed < -MAX_SPEED)
    left_speed = -MAX_SPEED;
  // 设置速度
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, RIGHT_STOP - right_speed);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, LEFT_STOP + left_speed);
}

void AlongLine(int time) // 直线运动
{
  AddPID_Angle.Target = Ang[2]; // 目标角度为当前角度
  AngFlag = 1;
  HAL_Delay(time);
  AngFlag = 0;
}

void Start() // 加速起步
{
  INIT_SPEED = 0;
  for (int i = 0; i < 200; i++)
  {
    INIT_SPEED++;
    AlongLine(2);
  }
  INIT_SPEED = 200;
}

void Stop() // 减速停车
{
  INIT_SPEED = 200;
  for (int i = 0; i < 200; i++)
  {
    INIT_SPEED--;
    AlongLine(2);
  }
  INIT_SPEED = 0;
}

void Redirect(float target, int time) // 控制到某角度
{
  AddPID_Angle.Target = target; // 目标角度
  AngFlag = 1;
  HAL_Delay(time);
  AngFlag = 0;
}

void AlongCurve(float target, float r, int flag) // 曲线运动
{
  if (flag == 1) // 向左转
  {
    while (Ang[2] < target - 5 * (1 - r / (r + 10)) || Ang[2] > target + 5 * (1 - r / (r + 10)))
      Walk(250, 170 * r / (r + 10));
  }
  if (flag == -1) // 向右转
  {
    while (Ang[2] < target - 5 * (1 - r / (r + 10)) || Ang[2] > target + 5 * (1 - r / (r + 10)))
      Walk(210 * r / (r + 10), 200);
  }
}

void Turn(float target, int flag) // 转向
{
  AlongCurve(target, 0, flag);
}

void Slide(int N, int speed, int flag) // 丝杆滑台
{
  if (flag == 1) // 向电机方向
    for (int i = 0; i < N; i++)
    {
      HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, GPIO_PIN_RESET);
      HAL_Delay(speed);
      HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, GPIO_PIN_RESET);
      HAL_Delay(speed);
      HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, GPIO_PIN_RESET);
      HAL_Delay(speed);
      HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, GPIO_PIN_SET);
      HAL_Delay(speed);
    }
  if (flag == -1) // 向挡板方向
    for (int i = 0; i < N; i++)
    {
      HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, GPIO_PIN_RESET);
      HAL_Delay(speed);
      HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, GPIO_PIN_SET);
      HAL_Delay(speed);
      HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, GPIO_PIN_RESET);
      HAL_Delay(speed);
      HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, GPIO_PIN_RESET);
      HAL_Delay(speed);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // 中断回调函数
{
  if (htim->Instance == TIM2 && AngFlag) // 角度控制
  {
    float Error = Ang[2] - AddPID_Angle.Target;
    if (Error > 180)
      AddPID_Angle.Error = Error - 360;
    else if (Error < -180)
      AddPID_Angle.Error = Error + 360;
    else
      AddPID_Angle.Error = Error;
    AddPID_Angle.Integral += AddPID_Angle.Error;
    AddPID_Angle.Output = AddPID_Angle.kp * AddPID_Angle.Error + AddPID_Angle.ki * AddPID_Angle.Integral + AddPID_Angle.kd * (AddPID_Angle.Error - AddPID_Angle.Error_1);
    AddPID_Angle.Error_1 = AddPID_Angle.Error;
    Walk(INIT_SPEED - AddPID_Angle.Output, INIT_SPEED + AddPID_Angle.Output);
  }

  if (htim->Instance == TIM2 && AngSpeedFlag) // 角速度控制
  {
    AddPID_AngSpeed.Error = AngSpeed[2] - AddPID_AngSpeed.Target;
    AddPID_AngSpeed.Integral += AddPID_AngSpeed.Error;
    AddPID_AngSpeed.Output = AddPID_AngSpeed.kp * AddPID_AngSpeed.Error + AddPID_AngSpeed.ki * AddPID_AngSpeed.Integral + AddPID_AngSpeed.kd * (AddPID_AngSpeed.Error - AddPID_AngSpeed.Error_1);
    AddPID_AngSpeed.Error_1 = AddPID_AngSpeed.Error;
    Walk(INIT_SPEED + AddPID_AngSpeed.Output, INIT_SPEED - AddPID_AngSpeed.Output);
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
