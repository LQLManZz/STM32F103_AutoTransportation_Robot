#include "motor.h"
#include "project_config.h"

extern TIM_HandleTypeDef htim3;

static uint8_t motorPwmPercent[2] = {0, 0};
static MotorDir motorCurDir[2] = {MOTOR_STOP, MOTOR_STOP};

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

static uint16_t scaleDutyForMotor(MotorId motor, uint16_t duty)
{
  if (motor == MOTOR_RIGHT && duty > 0u)
  {
    uint32_t scaled = ((uint32_t)duty * MOTOR_RIGHT_SCALE + (MOTOR_LEFT_SCALE / 2u)) / MOTOR_LEFT_SCALE;
    return (scaled > MOTOR_PWM_MAX_DUTY) ? (uint16_t)MOTOR_PWM_MAX_DUTY : (uint16_t)scaled;
  }
  return duty;
}

void motorSetDutyRaw(MotorId motor, MotorDir dir, uint16_t duty)
{
  uint16_t pinA = (motor == MOTOR_LEFT) ? MOTOR_PIN_IN3 : MOTOR_PIN_IN1;
  uint16_t pinB = (motor == MOTOR_LEFT) ? MOTOR_PIN_IN4 : MOTOR_PIN_IN2;
  uint32_t ch = (motor == MOTOR_LEFT) ? TIM_CHANNEL_2 : TIM_CHANNEL_1;

  if (duty > MOTOR_PWM_ARR_MAX) duty = MOTOR_PWM_ARR_MAX;
  // Phanh ngắn mạch L298N cần ENA/ENB mức 100% (ARR max) để nối tắt 2 đầu cuộn dây liên tục
  if (dir == MOTOR_BRAKE) duty = MOTOR_PWM_ARR_MAX;
  else if (dir == MOTOR_STOP) duty = 0;

  motorCurDir[motor] = dir;
  motorPwmPercent[motor] = (dir == MOTOR_STOP || dir == MOTOR_BRAKE || duty == 0)
                               ? 0u
                               : (uint8_t)((duty * 100u + (MOTOR_PWM_ARR_MAX / 2u)) / MOTOR_PWM_ARR_MAX);

  // Tắt PWM trước khi đảo chân IN1..IN4 để tránh ngắn mạch chuyển mạch trên cầu H
  __HAL_TIM_SET_COMPARE(&htim3, ch, 0);
  HAL_GPIO_WritePin(MOTOR_PORT_DIR, pinA, (dir == MOTOR_BACKWARD || dir == MOTOR_BRAKE) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_PORT_DIR, pinB, (dir == MOTOR_FORWARD  || dir == MOTOR_BRAKE) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim3, ch, duty);
}

void motorSetSpeed(MotorId motor, MotorDir dir, uint8_t speedPercent)
{
  motorSetDutyRaw(motor, dir, scaleDutyForMotor(motor, percentToDuty(speedPercent)));
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

uint8_t motorGetSpeedPercent(MotorId motor)
{
  return motorPwmPercent[motor];
}

MotorDir motorGetDir(MotorId motor)
{
  return motorCurDir[motor];
}
