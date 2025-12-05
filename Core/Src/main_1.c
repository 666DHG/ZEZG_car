/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @target         : STM32F103C8T6
 * @strategy       : O-P Loop (Inner Charge -> Right Attack -> Left Return)
 ******************************************************************************
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
typedef struct {
    uint32_t buffer[20]; 
    uint8_t head;
} MovingWindow_t;

// --- 核心战术状态机 ---
typedef enum {
    STRAT_INNER_LOOP,    // 0: 内圈模式 (寻找充电桩/充电中)
    STRAT_ATTACK_RIGHT,  // 1: 出击模式 (去右侧X区)
    STRAT_RETURN_HOME    // 2: 回家模式 (找左转路口回内圈)
} StrategyState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ==========================================
// 1. 运动控制参数
// ==========================================
#define BASE_SPEED      300   // 略微降低速度以提高识别率
#define TURN_SPEED_HIGH 400   // 强行转弯时的外轮速度
#define TURN_SPEED_LOW  100   // 强行转弯时的内轮速度
#define PWM_MAX_SPEED   1000  

#define KICK_START_DURATION  50   
#define KICK_START_PWM       800  

// ==========================================
// 2. 充电与检测参数
// ==========================================
#define MAX_CHARGE_TIME_MS   15000 
#define LEAVE_TIME_MS        2000  

#define WINDOW_SIZE          15     
#define ADC_OVERSAMPLE_CNT   32     

#define SLOPE_THRESHOLD_LONG 40     
#define SLOPE_THRESHOLD_PRE  15     
#define SLOPE_FULL_LIMIT     2      

// ==========================================
// 3. 战术时间参数
// ==========================================
#define ATTACK_DURATION_MS   6000   
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern ADC_HandleTypeDef hadc1;

// PID
float Kp = 18.0f; 
float Ki = 0.0f;  
float Kd = 45.0f; 
int16_t last_error = 0; 

// ADC & 充电
MovingWindow_t g_adc_window = {0};
uint32_t g_charge_start_tick = 0;
uint32_t g_full_charge_counter = 0; 

// 状态机
typedef enum {
  STATE_TRACKING,        
  STATE_CHARGING,        
  STATE_LEAVING_STATION, 
  STATE_STOPPED          
} CarAction_t;

CarAction_t g_car_action = STATE_TRACKING; 
StrategyState_t g_strat_state = STRAT_INNER_LOOP; 

// 战术计时器
uint32_t g_strat_timer = 0; 
uint8_t g_just_left_station = 0; 

// 启动逻辑
int16_t g_prev_left_speed = 0;
int16_t g_prev_right_speed = 0;
uint32_t g_left_kick_start_tick = 0;
uint32_t g_right_kick_start_tick = 0;

// 解决窗口初始化/重置问题
uint8_t g_is_adc_window_init = 0; 

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Motor_Init(void);
void Set_Motor_Speed(int16_t left_speed, int16_t right_speed);
uint8_t Read_Front_Sensors(void);

int16_t Calculate_Deviation(uint8_t sensor_value);
int16_t PID_Calculate(int16_t deviation);
void PID_Init(void);

uint32_t Get_Filtered_ADC(void);
void Window_Push(uint32_t val);
uint32_t Window_Get_Oldest(void);

// === 新增/修改函数原型 ===
void Window_Reset(uint32_t val); // 新增：重置ADC窗口基线
uint8_t Check_Charging_Signal(int32_t *out_slope); 
uint8_t Is_Battery_Full(int32_t slope); 
uint8_t Is_Right_Junction(uint8_t sensors); 
uint8_t Is_Left_Junction(uint8_t sensors);  
void Force_Turn_Right(void);
void Force_Turn_Left(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// --- ADC 处理 ---
uint32_t Get_Filtered_ADC(void) {
    uint32_t sum = 0;
    for (int i = 0; i < ADC_OVERSAMPLE_CNT; i++) {
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 2) == HAL_OK) {
            sum += HAL_ADC_GetValue(&hadc1);
        }
    }
    return sum / ADC_OVERSAMPLE_CNT;
}

void Window_Push(uint32_t val) {
    g_adc_window.buffer[g_adc_window.head] = val;
    g_adc_window.head = (g_adc_window.head + 1) % WINDOW_SIZE;
}

uint32_t Window_Get_Oldest(void) {
    return g_adc_window.buffer[g_adc_window.head];
}

/**
 * @brief 强制重置 ADC 窗口的基线
 * @note 解决问题二：消除充/放电过程中的基线剧烈漂移
 */
void Window_Reset(uint32_t val) {
    for(int i = 0; i < WINDOW_SIZE; i++) {
        g_adc_window.buffer[i] = val;
    }
    g_adc_window.head = 0;
    g_is_adc_window_init = 1;
}


/**
 * @brief 改进版充电检测 (滑窗积分法)
 * @retval 0: 无信号 / 1: 疑似信号(减速) / 2: 确认信号(停车)
 */
uint8_t Check_Charging_Signal(int32_t *out_slope)
{
    uint32_t current_adc = Get_Filtered_ADC();
    
    // 如果未初始化，直接返回 0 (在 main 或 STATE_LEAVING_STATION 中进行初始化)
    if (!g_is_adc_window_init) return 0; 

    uint32_t old_adc = Window_Get_Oldest();
    Window_Push(current_adc);

    int32_t delta_long = (int32_t)current_adc - (int32_t)old_adc;
    
    if (out_slope != NULL) *out_slope = delta_long;

    if (delta_long > SLOPE_THRESHOLD_LONG) return 2; 
    if (delta_long > SLOPE_THRESHOLD_PRE)  return 1; 

    return 0;
}

// --- 满电判断 ---
uint8_t Is_Battery_Full(int32_t slope) {
    // 负斜率或极小正斜率视为趋于稳定
    if (slope < SLOPE_FULL_LIMIT) { 
        g_full_charge_counter++;
    } else {
        g_full_charge_counter = 0;
    }
    
    // 连续 500ms (50次) 斜率都很低 -> 满电
    if (g_full_charge_counter > 50) {
        return 1;
    }
    return 0;
}

// --- 路口特征识别 ---
uint8_t Is_Right_Junction(uint8_t sensors) {
    // 特征：右侧 S0, S1 变黑 (binary ...00011 or ...00111)
    if ((sensors & 0x03) == 0x03 && !(sensors & 0x10)) return 1;
    return 0;
}

uint8_t Is_Left_Junction(uint8_t sensors) {
    // 特征：左侧 S4, S3 变黑 (binary 11000... or 11100...)
    if ((sensors & 0x18) == 0x18 && !(sensors & 0x01)) return 1;
    return 0;
}

// --- 强制转向动作 ---
void Force_Turn_Right(void) {
    Set_Motor_Speed(TURN_SPEED_HIGH, TURN_SPEED_LOW); 
    HAL_Delay(500); 
    PID_Init();     
}

void Force_Turn_Left(void) {
    Set_Motor_Speed(TURN_SPEED_LOW, TURN_SPEED_HIGH);
    HAL_Delay(500); 
    PID_Init();
}

// ... Set_Single_Motor, Set_Motor_Speed, Motor_Init (保持原样，省略) ...
static void Set_Single_Motor(TIM_HandleTypeDef *htim_fwd, uint32_t ch_fwd, 
                             TIM_HandleTypeDef *htim_rev, uint32_t ch_rev, 
                             int16_t speed) 
{
  if (speed > PWM_MAX_SPEED) speed = PWM_MAX_SPEED;
  if (speed < -PWM_MAX_SPEED) speed = -PWM_MAX_SPEED;
  if (speed > 0) {
    __HAL_TIM_SET_COMPARE(htim_fwd, ch_fwd, speed);
    __HAL_TIM_SET_COMPARE(htim_rev, ch_rev, 0);
  } else if (speed < 0) {
    __HAL_TIM_SET_COMPARE(htim_fwd, ch_fwd, 0);
    __HAL_TIM_SET_COMPARE(htim_rev, ch_rev, abs(speed));
  } else {
    __HAL_TIM_SET_COMPARE(htim_fwd, ch_fwd, 0);
    __HAL_TIM_SET_COMPARE(htim_rev, ch_rev, 0);
  }
}

void Set_Motor_Speed(int16_t left_speed, int16_t right_speed) {
  uint32_t current_tick = HAL_GetTick();
  int16_t final_left = left_speed;
  int16_t final_right = right_speed;

  if (left_speed != 0) {
    if (g_prev_left_speed == 0) g_left_kick_start_tick = current_tick;
    if ((current_tick - g_left_kick_start_tick) < KICK_START_DURATION)
      final_left = (left_speed > 0) ? KICK_START_PWM : -KICK_START_PWM;
  }
  if (right_speed != 0) {
    if (g_prev_right_speed == 0) g_right_kick_start_tick = current_tick;
    if ((current_tick - g_right_kick_start_tick) < KICK_START_DURATION)
      final_right = (right_speed > 0) ? KICK_START_PWM : -KICK_START_PWM;
  }
  Set_Single_Motor(&htim3, TIM_CHANNEL_1, &htim3, TIM_CHANNEL_2, final_left);
  Set_Single_Motor(&htim4, TIM_CHANNEL_3, &htim3, TIM_CHANNEL_4, final_right);
  g_prev_left_speed = left_speed;
  g_prev_right_speed = right_speed;
}

void Motor_Init(void) {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); 
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); 
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); 
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); 
}

uint8_t Read_Front_Sensors(void) {
  uint8_t sensor_val = 0;
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) sensor_val |= (1 << 0);
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET) sensor_val |= (1 << 1);
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET) sensor_val |= (1 << 2);
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET) sensor_val |= (1 << 3);
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET) sensor_val |= (1 << 4);
  return sensor_val;
}

int16_t Calculate_Deviation(uint8_t sensor_value) {
  int weights[5] = {43, 15, 0, -15, -43};
  float sum_weighted = 0;
  float sum_active = 0;
  for (int i = 0; i < 5; i++) {
    if ((sensor_value >> i) & 0x01) { 
      sum_weighted += weights[i];
      sum_active++;
    }
  }
  if (sum_active == 0) return last_error;
  return (int16_t)(sum_weighted / sum_active);
}

void PID_Init(void) { last_error = 0; }
int16_t PID_Calculate(int16_t deviation) {
  float P = deviation * Kp;
  float D = (deviation - last_error) * Kd;
  last_error = deviation;
  int16_t output = (int16_t)(P + D);
  if (output > 800) output = 800;
  if (output < -800) output = -800;
  return output;
}
/* USER CODE END 0 */


int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM4_Init(); 
  MX_ADC1_Init();
  
  Motor_Init(); 
  PID_Init(); 
  
  // 初始化 ADC 窗口基线
  uint32_t initial_adc = Get_Filtered_ADC();
  Window_Reset(initial_adc);

  while (1)
  {
    uint8_t sensors = Read_Front_Sensors();
    int32_t current_slope = 0;
    
    uint8_t charge_signal = Check_Charging_Signal(&current_slope);

    switch (g_car_action)
    {
      case STATE_TRACKING:
        {
          // 1. 优先处理充电识别
          if (charge_signal == 2) {
            Set_Motor_Speed(0, 0);
            g_charge_start_tick = HAL_GetTick();
            g_car_action = STATE_CHARGING;
            g_strat_state = STRAT_INNER_LOOP; 
            g_full_charge_counter = 0;
            break;
          }

          // 2. 战术分流处理路口
          if (g_strat_state == STRAT_INNER_LOOP) {
              // 策略：只有满油刚出来时，才找右侧入口
              if (g_just_left_station && Is_Right_Junction(sensors)) {
                  Force_Turn_Right(); 
                  
                  g_strat_state = STRAT_ATTACK_RIGHT;
                  g_strat_timer = HAL_GetTick(); 
                  g_just_left_station = 0; 
              }
          }
          else if (g_strat_state == STRAT_ATTACK_RIGHT) {
              // 策略：跑够时间就回家
              if (HAL_GetTick() - g_strat_timer > ATTACK_DURATION_MS) {
                  g_strat_state = STRAT_RETURN_HOME;
              }
          }
          else if (g_strat_state == STRAT_RETURN_HOME) {
              // 策略：找第一个左岔路回家
              if (Is_Left_Junction(sensors)) {
                  Force_Turn_Left();
                  g_strat_state = STRAT_INNER_LOOP;
              }
          }

          // 3. 正常 PID 循迹
          int16_t deviation = Calculate_Deviation(sensors);
          int16_t correction = PID_Calculate(deviation);
          int16_t base_speed = BASE_SPEED;
          
          if (charge_signal == 1) base_speed = BASE_SPEED / 2;

          Set_Motor_Speed(base_speed + correction, base_speed - correction);
        }
        break;

      case STATE_CHARGING:
        Set_Motor_Speed(0, 0);
        
        uint8_t is_full = Is_Battery_Full(current_slope);
        uint8_t is_timeout = (HAL_GetTick() - g_charge_start_tick > MAX_CHARGE_TIME_MS);

        if (is_full || is_timeout)
        {
            g_charge_start_tick = HAL_GetTick();
            g_car_action = STATE_LEAVING_STATION;
            g_just_left_station = 1; 
        }
        break;
        
      // === 问题一：修改盲跑为 PID 循迹离开 ===
      case STATE_LEAVING_STATION:
        {
          // PID 循迹离开充电区
          int16_t deviation_leave = Calculate_Deviation(sensors);
          int16_t correction_leave = PID_Calculate(deviation_leave);
          
          int16_t base_speed_leave = BASE_SPEED; 
          
          Set_Motor_Speed(base_speed_leave + correction_leave, base_speed_leave - correction_leave);
          
          if ((HAL_GetTick() - g_charge_start_tick) > LEAVE_TIME_MS)
          {
              // === 问题二：离开时重置 ADC 窗口 ===
              uint32_t current_adc_val = Get_Filtered_ADC();
              Window_Reset(current_adc_val); // 用当前的稳定电压重置基线
              
              PID_Init(); 
              g_car_action = STATE_TRACKING;
          }
        }
        break;
        
      default:
        Set_Motor_Speed(0, 0);
        break;
    }
    HAL_Delay(10);
  }
}

// ... SystemClock_Config 和 Error_Handler 保持原样 ...
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
}

void Error_Handler(void) { __disable_irq(); while (1) {} }

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif