#include "motor.h"
#include "project_config.h"

extern TIM_HandleTypeDef htim3;

void motorInit(void)
{
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // PA6 - ENA (bánh phải)
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // PA7 - ENB (bánh trái)
  motorStopAll();
}

static uint16_t percentToDuty(uint8_t percent)
{
  if (percent == 0) return 0;
  if (percent > 100) percent = 100;
  return (uint16_t)(MOTOR_PWM_MIN_DUTY +
                    ((uint32_t)percent * (MOTOR_PWM_MAX_DUTY - MOTOR_PWM_MIN_DUTY)) / 100u);
}

void motorSetDutyRaw(MotorId motor, MotorDir dir, uint16_t duty)
{
  uint16_t pinA = (motor == MOTOR_LEFT) ? MOTOR_PIN_IN3 : MOTOR_PIN_IN1;
  uint16_t pinB = (motor == MOTOR_LEFT) ? MOTOR_PIN_IN4 : MOTOR_PIN_IN2;
  uint32_t ch = (motor == MOTOR_LEFT) ? TIM_CHANNEL_2 : TIM_CHANNEL_1;

  if (duty > MOTOR_PWM_ARR_MAX) duty = MOTOR_PWM_ARR_MAX;
  if (dir == MOTOR_BRAKE) duty = MOTOR_PWM_MAX_DUTY;
  else if (dir == MOTOR_STOP) duty = 0;

  HAL_GPIO_WritePin(MOTOR_PORT_DIR, pinA, (dir == MOTOR_BACKWARD || dir == MOTOR_BRAKE) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_PORT_DIR, pinB, (dir == MOTOR_FORWARD  || dir == MOTOR_BRAKE) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim3, ch, duty);
}

void motorSetSpeed(MotorId motor, MotorDir dir, uint8_t speedPercent)
{
  motorSetDutyRaw(motor, dir, percentToDuty(speedPercent));
}

void motorStopAll(void)
{
  motorSetDutyRaw(MOTOR_LEFT, MOTOR_STOP, 0);
  motorSetDutyRaw(MOTOR_RIGHT, MOTOR_STOP, 0);
}

void motorBrakeAll(void)
{
  motorSetDutyRaw(MOTOR_LEFT, MOTOR_BRAKE, 0);
  motorSetDutyRaw(MOTOR_RIGHT, MOTOR_BRAKE, 0);
}
