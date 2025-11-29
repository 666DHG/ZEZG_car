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
 /*
 TODO LIST
 1. 测试充电时间 CHARGE_TIME_MS 是否合适
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ========== 1. 时间参数定义 ==========
#define CHARGE_TIME_MS 10000 // 充电时间：10000ms (10秒)
#define LEAVE_TIME_MS  500   // 离开充电站时的盲跑时间：500ms (0.5秒)
#define ADC_THRESHOLD  1200  // ADC电压阈值 (1200约等于1.0V，根据实测调整)

// ========== 2. 速度常量定义 ==========
#define BASE_SPEED 300     // 基础速度
#define PWM_MAX_SPEED 1000 // PWM最大速度：定时器周期 (ARR+1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern ADC_HandleTypeDef hadc1;

// ========== 3. PID控制器参数 ==========
float Kp = 15.0f; // 比例系数：提供基础转向力
float Ki = 0.0f;  // 积分系数：循迹通常为0
float Kd = 50.0f; // 微分系数：抑制直角弯震荡，提供瞬时拉力

int16_t last_error = 0; // 记录上一次的偏差值

// ========== 4. 状态机定义 ==========
typedef enum {
  STATE_TRACKING,        // 正常循迹
  STATE_CHARGING,        // 停车充电
  STATE_LEAVING_STATION, // 充满后盲跑离开
  STATE_STOPPED          // 停止/故障
} CarState_t;

CarState_t g_car_state = STATE_TRACKING; // 初始状态
uint32_t g_charge_start_tick = 0;        // 计时器变量

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void Motor_Init(void);
uint8_t Read_Front_Sensors(void);
int16_t Calculate_Deviation(uint8_t sensor_value);
void PID_Init(void);
int16_t PID_Calculate(int16_t deviation);
void Set_Motor_Speed(int16_t left_speed, int16_t right_speed);
uint8_t Check_Charging_Signal(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief 检测是否进入无线充电区域
 * @retval 1: 检测到充电信号 (电压升高)
 * @retval 0: 未检测到
 */
uint8_t Check_Charging_Signal(void)
{
    // 1. 启动 ADC 转换
    HAL_ADC_Start(&hadc1);
    
    // 2. 等待转换完成 (超时 10ms)
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
        // 3. 读取 ADC 值 (0 - 4095, 对应 0 - 3.3V)
        uint32_t adc_value = HAL_ADC_GetValue(&hadc1);
        
        // 4. 判断阈值
        if (adc_value > ADC_THRESHOLD) 
        {
            return 1; // 在充电区
        }
    }
    // 停止 ADC (可选)
    // HAL_ADC_Stop(&hadc1); 
    
    return 0; // 不在充电区
}

// 辅助函数：设置单个电机的PWM (支持跨定时器)
static void Set_Single_Motor(TIM_HandleTypeDef *htim_fwd, uint32_t ch_fwd, 
                             TIM_HandleTypeDef *htim_rev, uint32_t ch_rev, 
                             int16_t speed) 
{
  // 限幅
  if (speed > PWM_MAX_SPEED) speed = PWM_MAX_SPEED;
  if (speed < -PWM_MAX_SPEED) speed = -PWM_MAX_SPEED;

  if (speed > 0) {
    // 前进
    __HAL_TIM_SET_COMPARE(htim_fwd, ch_fwd, speed);
    __HAL_TIM_SET_COMPARE(htim_rev, ch_rev, 0);
  } else if (speed < 0) {
    // 后退
    __HAL_TIM_SET_COMPARE(htim_fwd, ch_fwd, 0);
    __HAL_TIM_SET_COMPARE(htim_rev, ch_rev, abs(speed));
  } else {
    // 停止
    __HAL_TIM_SET_COMPARE(htim_fwd, ch_fwd, 0);
    __HAL_TIM_SET_COMPARE(htim_rev, ch_rev, 0);
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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  
  /* USER CODE BEGIN 2 */
  // ========== 系统启动初始化 ==========
  Motor_Init(); 
  PID_Init(); 
  g_car_state = STATE_CHARGING; // 初始进入充电模式
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    // 1. 读取循迹传感器
    uint8_t sensors = Read_Front_Sensors();
    
    switch (g_car_state)
    {
      case STATE_TRACKING:
        // --- 正常循迹状态 ---
        
        // A. 检测是否需要充电
        if (Check_Charging_Signal()) 
        {
          // 立即停车
          Set_Motor_Speed(0, 0);
          // 记录开始时间，切换状态
          g_charge_start_tick = HAL_GetTick();
          g_car_state = STATE_CHARGING;
        }
        else
        {
          // B. 正常 PID 循迹
          int16_t deviation = Calculate_Deviation(sensors);
          int16_t correction = PID_Calculate(deviation);
          
          int16_t left = BASE_SPEED + correction;
          int16_t right = BASE_SPEED - correction;
          
          Set_Motor_Speed(left, right);
        }
        break;

      case STATE_CHARGING:
        // --- 充电状态 ---
        // 确保停车
        Set_Motor_Speed(0, 0);
        
        // 检查是否充够了时间
        if ((HAL_GetTick() - g_charge_start_tick) > CHARGE_TIME_MS)
        {
          // 充满，切换到盲跑离开模式
          g_charge_start_tick = HAL_GetTick(); // 复用变量记录离开时间
          g_car_state = STATE_LEAVING_STATION;
        }
        break;
        
      case STATE_LEAVING_STATION:
        // --- 盲跑离开状态 ---
        // 强制直行冲出充电区 (防止立刻又检测到电压而停车)
        Set_Motor_Speed(BASE_SPEED, BASE_SPEED); 
        
        // 检查是否跑够了时间
        if ((HAL_GetTick() - g_charge_start_tick) > LEAVE_TIME_MS)
        {
          // 恢复正常循迹
          PID_Init(); // 重置PID误差
          g_car_state = STATE_TRACKING;
        }
        break;
        
      case STATE_STOPPED:
      default:
        Set_Motor_Speed(0, 0);
        break;
    }

    // 控制循环频率 (约 100Hz)
    HAL_Delay(10);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  // 保持 CubeMX 生成的内容，确保 ADC 分频正确 (/6 或 /8)
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  
  // ADC 时钟配置 (关键：确保不超过 14MHz)
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8; // 72MHz / 8 = 9MHz (安全)
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// --- 硬件驱动函数实现 ---

void Motor_Init(void) {
  // 1. 激活 TB6612FNG / DRV8833 (STBY 引脚)
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

  // 2. 启动 TIM3 (左轮全套 + 右轮后退)
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // 左前
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // 左后
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // 右后

  // 3. 启动 TIM4 (右轮前进 - 修复损坏的PB0)
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // 右前 (PB8)
}

void Set_Motor_Speed(int16_t left_speed, int16_t right_speed) {
  // 左电机: TIM3_CH1(Fwd) / TIM3_CH2(Rev)
  Set_Single_Motor(&htim3, TIM_CHANNEL_1, &htim3, TIM_CHANNEL_2, left_speed);

  // 右电机: TIM4_CH3(Fwd) / TIM3_CH4(Rev) -- 混合定时器
  Set_Single_Motor(&htim4, TIM_CHANNEL_3, &htim3, TIM_CHANNEL_4, right_speed);
}

uint8_t Read_Front_Sensors(void) {
  uint8_t sensor_val = 0;
  // 读取 PA0-PA4
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) sensor_val |= (1 << 0);
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET) sensor_val |= (1 << 1);
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET) sensor_val |= (1 << 2);
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET) sensor_val |= (1 << 3);
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET) sensor_val |= (1 << 4);
  return sensor_val;
}

int16_t Calculate_Deviation(uint8_t sensor_value) {
  // 物理位置权重 (非均匀分布)
  // S0(最右)..S4(最左) -> {43, 15, 0, -15, -43}
  int weights[5] = {43, 15, 0, -15, -43};

  float sum_weighted = 0;
  float sum_active = 0;

  for (int i = 0; i < 5; i++) {
    if ((sensor_value >> i) & 0x01) { 
      sum_weighted += weights[i];
      sum_active++;
    }
  }

  // 丢线保持
  if (sum_active == 0) {
    return last_error;
  }

  int16_t error = (int16_t)(sum_weighted / sum_active);
  return error;
}

void PID_Init(void) {
  last_error = 0;
}

int16_t PID_Calculate(int16_t deviation) {
  // P项
  float P = deviation * Kp;
  // D项
  float D = (deviation - last_error) * Kd;
  
  last_error = deviation;

  int16_t output = (int16_t)(P + D);

  // 限幅
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */