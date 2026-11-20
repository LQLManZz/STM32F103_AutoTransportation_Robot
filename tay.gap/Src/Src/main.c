/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (2 Servo 180: Kẹp và Nâng hạ)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* --- CẤU HÌNH XUNG PWM (Servo 180) --- */
#define SERVO_MIN_US        500
#define SERVO_MAX_US        2500

/* --- CẤU HÌNH GÓC CHO SERVO KẸP (Channel 1) --- */
#define GRIP_OPEN_ANGLE     140   /* Góc mở ngàm */
#define GRIP_CLOSE_ANGLE    180   /* Góc kẹp vật */

/* --- CẤU HÌNH GÓC CHO SERVO NÂNG HẠ (Channel 2) --- */
/* Cần nâng hạ chỉ quay 1 góc nhỏ để nhấc lên khỏi mặt đất */
#define LIFT_DOWN_ANGLE     90    /* Góc hạ cần (sát đất) */
#define LIFT_UP_ANGLE       130   /* Góc nâng cần (nhấc lên 1 góc 25 độ) */

/* --- TỐC ĐỘ DI CHUYỂN MƯỢT --- */
#define STEP_DEG            1
#define STEP_DELAY_MS       15

/* --- THỜI GIAN CHỜ --- */
#define WAIT_OPEN_MS        3000  /* Đợi trước khi gắp */
#define HOLD_TIME_MS        5000  /* Thời gian giữ vật trên không */
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
extern TIM_HandleTypeDef htim2;

/* Biến lưu trữ góc hiện tại của 2 servo */
static int16_t g_grip_angle = GRIP_OPEN_ANGLE;
static int16_t g_lift_angle = LIFT_DOWN_ANGLE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN 0 */
static void Servo_TimerInit50Hz(void)
{
    uint32_t tim_clk = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
        tim_clk *= 2;

    __HAL_TIM_SET_PRESCALER(&htim2, (tim_clk / 1000000UL) - 1);
    __HAL_TIM_SET_AUTORELOAD(&htim2, 20000 - 1);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);
}

/* Hàm quy đổi góc sang xung PWM */
static uint32_t AngleToPulse(int16_t angle)
{
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    return SERVO_MIN_US + ((uint32_t)angle * (SERVO_MAX_US - SERVO_MIN_US)) / 180;
}

/* ================= ĐIỀU KHIỂN SERVO KẸP (CH1) ================= */
static void Servo_Grip_SetAngle(int16_t angle)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, AngleToPulse(angle));
    g_grip_angle = angle;
}

static void Servo_Grip_MoveSmooth(int16_t target)
{
    while (g_grip_angle != target)
    {
        int16_t next = (g_grip_angle < target) ? g_grip_angle + STEP_DEG : g_grip_angle - STEP_DEG;
        if ((g_grip_angle < target && next > target) || (g_grip_angle > target && next < target)) next = target;
        Servo_Grip_SetAngle(next);
        HAL_Delay(STEP_DELAY_MS);
    }
}

/* ================= ĐIỀU KHIỂN SERVO NÂNG HẠ (CH2) ================= */
static void Servo_Lift_SetAngle(int16_t angle)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, AngleToPulse(angle));
    g_lift_angle = angle;
}

static void Servo_Lift_MoveSmooth(int16_t target)
{
    while (g_lift_angle != target)
    {
        int16_t next = (g_lift_angle < target) ? g_lift_angle + STEP_DEG : g_lift_angle - STEP_DEG;
        if ((g_lift_angle < target && next > target) || (g_lift_angle > target && next < target)) next = target;
        Servo_Lift_SetAngle(next);
        HAL_Delay(STEP_DELAY_MS);
    }
}

/* Bật/Tắt PWM cho cả 2 kênh */
static void Servos_PwmOn(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}
static void Servos_PwmOff(void)
{
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
  Servo_TimerInit50Hz();

  /* TRẠNG THÁI BAN ĐẦU: Cần đang NÂNG LÊN và Ngàm MỞ RA */
  Servo_Lift_SetAngle(LIFT_UP_ANGLE);
  Servo_Grip_SetAngle(GRIP_OPEN_ANGLE);
  Servos_PwmOn();

  HAL_Delay(1000); /* Chờ 2 servo ổn định vị trí ban đầu */
  /* USER CODE END 2 */

  while (1)
  {
      /* 1. Đang ở trên cao, chờ nạp vật liệu */
      HAL_Delay(WAIT_OPEN_MS);

      /* 2. Hạ cần xuống đất để chuẩn bị gắp */
      Servo_Lift_MoveSmooth(LIFT_DOWN_ANGLE);
      HAL_Delay(300); /* Chờ cần hạ xong */

      /* 3. Kẹp vật từ từ */
      Servo_Grip_MoveSmooth(GRIP_CLOSE_ANGLE);
      HAL_Delay(300); /* Dừng một chút cho ngàm kẹp chắc chắn */

      /* 4. Nâng vật lên khỏi mặt đất */
      Servo_Lift_MoveSmooth(LIFT_UP_ANGLE);

      /* 5. Giữ vật trên không */
      HAL_Delay(HOLD_TIME_MS);

      /* 6. Hạ vật xuống mặt đất để nhả */
      Servo_Lift_MoveSmooth(LIFT_DOWN_ANGLE);
      HAL_Delay(300); /* Chờ chạm đất hẳn */

      /* 7. Mở ngàm nhả vật */
      Servo_Grip_MoveSmooth(GRIP_OPEN_ANGLE);
      HAL_Delay(300);

      /* 8. Nâng cần lên lại vị trí chờ ban đầu cho chu kỳ tiếp theo */
      Servo_Lift_MoveSmooth(LIFT_UP_ANGLE);
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Cấu hình Kênh 1 (Kẹp) */
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Cấu hình Kênh 2 (Nâng Hạ) */
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim2);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(SIG_GPIO_Port, SIG_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = SIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SIG_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { }
#endif
