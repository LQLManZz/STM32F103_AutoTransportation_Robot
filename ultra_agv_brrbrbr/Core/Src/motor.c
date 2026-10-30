#include "motor.h"
#include "project_config.h"

extern TIM_HandleTypeDef htim3;

void motorInit(void)
{
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // PA6 - ENA (bánh phải)
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // PA7 - ENB (bánh trái)
  motorStopAll();
}

// Đổi % tốc độ sang duty PWM, từ ngưỡng thắng ma sát tới trần an toàn
static uint16_t percentToDuty(uint8_t percent)
{
  if (percent == 0)
  {
    return 0;
  }
  if (percent > 100)
  {
    percent = 100;
  }
  return (uint16_t)(MOTOR_PWM_MIN_DUTY + ((uint32_t)percent * (MOTOR_PWM_MAX_DUTY - MOTOR_PWM_MIN_DUTY)) / 100);
}

void motorSetSpeed(MotorId motor, MotorDir dir, uint8_t speedPercent)
{
  // Bánh trái: IN3, IN4, TIM3 CH2. Bánh phải: IN1, IN2, TIM3 CH1
  uint16_t pinA = (motor == MOTOR_LEFT) ? MOTOR_PIN_IN3 : MOTOR_PIN_IN1;
  uint16_t pinB = (motor == MOTOR_LEFT) ? MOTOR_PIN_IN4 : MOTOR_PIN_IN2;
  uint32_t channel = (motor == MOTOR_LEFT) ? TIM_CHANNEL_2 : TIM_CHANNEL_1;
  uint16_t duty = percentToDuty(speedPercent);

  switch (dir)
  {
  case MOTOR_FORWARD:
    HAL_GPIO_WritePin(MOTOR_PORT_DIR, pinA, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_PORT_DIR, pinB, GPIO_PIN_SET);
    break;
  case MOTOR_BACKWARD:
    HAL_GPIO_WritePin(MOTOR_PORT_DIR, pinA, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_PORT_DIR, pinB, GPIO_PIN_RESET);
    break;
  case MOTOR_BRAKE: // 2 chân IN cùng mức cao, EN tối đa
    HAL_GPIO_WritePin(MOTOR_PORT_DIR, pinA, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_PORT_DIR, pinB, GPIO_PIN_SET);
    duty = MOTOR_PWM_MAX_DUTY;
    break;
  default: // MOTOR_STOP
    HAL_GPIO_WritePin(MOTOR_PORT_DIR, pinA, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_PORT_DIR, pinB, GPIO_PIN_RESET);
    duty = 0;
    break;
  }
  __HAL_TIM_SET_COMPARE(&htim3, channel, duty);
}

void motorStopAll(void)
{
  motorSetSpeed(MOTOR_LEFT, MOTOR_STOP, 0);
  motorSetSpeed(MOTOR_RIGHT, MOTOR_STOP, 0);
}

void motorBrakeAll(void)
{
  motorSetSpeed(MOTOR_LEFT, MOTOR_BRAKE, 100);
  motorSetSpeed(MOTOR_RIGHT, MOTOR_BRAKE, 100);
}
