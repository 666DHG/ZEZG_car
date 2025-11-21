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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdlib.h>
#include <stdint.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// 定义全局状态变量
#define BASE_SPEED 400
#define PWM_MAX_SPEED 1000

// --- PID 参数 ---
float Kp = 35.0f;   // #TODO: 比例系数 (需要调试，建议从20-50开始)
float Ki = 0.0f;    // 积分系数 (循迹通常不需要，设为0)
float Kd = 20.0f;   // 微分系数 (用于抑制由于惯性产生的震荡)

int16_t last_error = 0; // 上一次的偏差值，用于计算D
// ----------------

// 定义小车状态
typedef enum {
  STATE_FORWARD_TRACKING,
  STATE_REVERSE_TRACKING,
  STATE_STOPPED
} CarState_t;

CarState_t g_car_state = STATE_STOPPED; // 初始状态为停止

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/*
 * @brief  初始化电机驱动
 * @note   配置控制电机速度的Timer (PWM) 和控制方向的GPIO
 * @retval None
 */
void Motor_Init(void);

/*
 * @brief  读取前端5路灰度传感器的状态
 * @note   读取A0-A4，将5个引脚的状态打包成一个5位二进制数 (uint8_t)
 * 例如: 0b00100 表示中间传感器检测到黑线
 * @retval 0-31 之间的整数，代表5个传感器的状态
 */
uint8_t Read_Front_Sensors(void);

/*
 * @brief  读取后端5路灰度传感器的状态
 * @note   读取B3-B7，逻辑同上
 * @retval 0-31 之间的整数
 */
uint8_t Read_Rear_Sensors(void);

/*
 * @brief  根据传感器状态计算循迹偏差值
 * @param  sensor_value: 5位传感器数据 (来自Read_Front_Sensors或Read_Rear_Sensors)
 * @retval int16_t: 偏差值。
 * 0  表示完全居中 (例如 0b00100)
 * 负值 表示偏左 (例如 0b10000 或 0b01000)
 * 正值 表示偏右 (例如 0b00010 或 0b00001)
 * 特殊值 (如 999) 表示丢失目标 (例如 0b00000)
 */
int16_t Calculate_Deviation(uint8_t sensor_value);

/*
 * @brief  PID控制器初始化
 * @note   设置Kp, Ki, Kd三个参数的初始值
 * @retval None
 */
void PID_Init(void);

/*
 * @brief  PID计算函数
 * @param  deviation: 当前的偏差值 (来自Calculate_Deviation)
 * @retval int16_t: 计算出的电机速度修正量
 */
int16_t PID_Calculate(int16_t deviation);

/*
 * @brief  设置电机速度
 * @param  left_speed: 左轮速度 (-1000 到 1000，假设PWM满量程为1000)
 * @param  right_speed: 右轮速度 (-1000 到 1000)
 * @note   函数内部处理正负号，负号表示反转。
 * 它会控制方向引脚，并设置PWM的占空比。
 */
void Set_Motor_Speed(int16_t left_speed, int16_t right_speed);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  // 初始化
  Motor_Init();
  PID_Init();
  g_car_state = STATE_FORWARD_TRACKING; // 假设开始时是前进

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    HAL_Delay(1000);
    // --- 电机测试代码 ---
    // 前进 (速度 500/1000)
    Set_Motor_Speed(500, 500);
    HAL_Delay(2000); // 持续2秒

    // 停止
    Set_Motor_Speed(0, 0);
    HAL_Delay(1000); // 停1秒

    // 后退 (速度 -500/1000)
    Set_Motor_Speed(-500, -500);
    HAL_Delay(2000); // 持续2秒

    // 停止
    Set_Motor_Speed(0, 0);
    HAL_Delay(1000); // 停1秒
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

// --- 1. 电机初始化 ---
void Motor_Init(void)
{
  // 启动 TIM3 的 4 个通道 PWM
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

// --- 2. 设置电机速度 (DRV8833 逻辑) ---
// 辅助函数：设置单个电机的PWM
static void Set_Single_Motor(uint32_t ch_fwd, uint32_t ch_rev, int16_t speed)
{
  if (speed > PWM_MAX_SPEED) speed = PWM_MAX_SPEED;
  if (speed < -PWM_MAX_SPEED) speed = -PWM_MAX_SPEED;

  if (speed > 0) {
    __HAL_TIM_SET_COMPARE(&htim3, ch_fwd, speed);
    __HAL_TIM_SET_COMPARE(&htim3, ch_rev, 0);
  } else if (speed < 0) {
    __HAL_TIM_SET_COMPARE(&htim3, ch_fwd, 0);
    __HAL_TIM_SET_COMPARE(&htim3, ch_rev, abs(speed));
  } else {
    __HAL_TIM_SET_COMPARE(&htim3, ch_fwd, 0);
    __HAL_TIM_SET_COMPARE(&htim3, ch_rev, 0);
  }
}

void Set_Motor_Speed(int16_t left_speed, int16_t right_speed)
{
  // 左电机：TIM3 CH1(PA6) / CH2(PA7)
  Set_Single_Motor(TIM_CHANNEL_1, TIM_CHANNEL_2, left_speed);
  
  // 右电机：TIM3 CH3(PB0) / CH4(PB1)
  Set_Single_Motor(TIM_CHANNEL_3, TIM_CHANNEL_4, right_speed);
}

// --- 3. 读取前端传感器 (A0-A4) ---
uint8_t Read_Front_Sensors(void)
{
  uint8_t sensor_val = 0;
  // 假设高电平为黑线 (1)，低电平为白底 (0)
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) sensor_val |= (1 << 0);
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET) sensor_val |= (1 << 1);
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET) sensor_val |= (1 << 2);
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET) sensor_val |= (1 << 3);
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET) sensor_val |= (1 << 4);
  return sensor_val;
}

// --- 4. 读取后端传感器 (B3-B7) ---
uint8_t Read_Rear_Sensors(void)
{
  uint8_t sensor_val = 0;
  // #TODO: 注意：PB3 和 PB4 默认是 JTAG 引脚，需要在 CubeMX 中配置 Debug 为 "Serial Wire" 才能作为普通 GPIO 使用
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_SET) sensor_val |= (1 << 0);
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET) sensor_val |= (1 << 1);
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET) sensor_val |= (1 << 2);
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET) sensor_val |= (1 << 3);
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) sensor_val |= (1 << 4);
  return sensor_val;
}

// --- 5. 计算偏差值 ---
int16_t Calculate_Deviation(uint8_t sensor_value)
{
  // 简单的加权平均法
  // 权重分配: 左2(-2), 左1(-1), 中(0), 右1(1), 右2(2)
  // 对应位:   bit4,    bit3,    bit2,  bit1,   bit0 (根据Read函数顺序)
  
  // 这里的 bit0 对应 A0/B3，bit4 对应 A4/B7
  // #TODO: 假设 A0/B3 在最右边，A4/B7 在最左边 (请根据实际焊接调整正负号)
  
  float sum_weighted = 0;
  float sum_active = 0;
  
  // 权重定义：最左(bit4) -> -20, 最右(bit0) -> 20
  // 扩大数值方便 PID 计算
  int weights[5] = {20, 10, 0, -10, -20}; 
  
  for (int i = 0; i < 5; i++) {
    if ((sensor_value >> i) & 0x01) { // 如果第 i 位是 1 (检测到黑线)
      sum_weighted += weights[i];
      sum_active++;
    }
  }
  
  if (sum_active == 0) {
    // 丢失目标：保持上一次的偏差方向，或者返回极大值让其急转
    // 这里简单返回上一次的偏差
    return last_error; 
  }
  
  // 计算平均偏差
  int16_t error = (int16_t)(sum_weighted / sum_active);
  return error;
}

// --- 6. PID 初始化 ---
void PID_Init(void)
{
  last_error = 0;
  // Kp, Ki, Kd 已经在全局变量定义时初始化
}

// --- 7. PID 计算 ---
int16_t PID_Calculate(int16_t deviation)
{
  // 1. 计算 P (比例)
  float P = deviation * Kp;
  
  // 2. 计算 D (微分) - 预测未来趋势
  float D = (deviation - last_error) * Kd;
  
  // 更新 last_error
  last_error = deviation;
  
  // 3. 计算总输出 (忽略 I 积分项，循迹一般不需要)
  int16_t output = (int16_t)(P + D);
  
  // 限制最大修正量，防止电机反转过快
  if (output > 800) output = 800;
  if (output < -800) output = -800;
  
  return output;
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
