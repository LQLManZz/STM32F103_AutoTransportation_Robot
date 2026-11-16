#include "servo.h"
#include "project_config.h"

extern TIM_HandleTypeDef htim2;

void servoInit(void)
{
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  setServoAngle(SCAN_HOME_ANGLE);
}

void setServoAngle(uint8_t angleDeg)
{
  if (angleDeg > SERVO_MAX_ANGLE_DEG) angleDeg = SERVO_MAX_ANGLE_DEG;
  uint32_t pulseUs = SERVO_MIN_PULSE_US +
                     ((uint32_t)angleDeg * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) / SERVO_MAX_ANGLE_DEG;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulseUs);
}

uint8_t getServoAngle(void)
{
  uint32_t pulseUs = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);
  uint32_t span = SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US;

  if (pulseUs < SERVO_MIN_PULSE_US) pulseUs = SERVO_MIN_PULSE_US;
  if (pulseUs > SERVO_MAX_PULSE_US) pulseUs = SERVO_MAX_PULSE_US;
  return (uint8_t)(((pulseUs - SERVO_MIN_PULSE_US) * SERVO_MAX_ANGLE_DEG + span / 2u) / span);
}

// Quay servo dần tới góc targetDeg, tránh momen đột ngột làm rung cảm biến
void moveServoSmooth(uint8_t targetDeg)
{
  if (targetDeg > SERVO_MAX_ANGLE_DEG) targetDeg = SERVO_MAX_ANGLE_DEG;
  int16_t angle = (int16_t)getServoAngle();

  while (angle != (int16_t)targetDeg)
  {
    int16_t diff = (int16_t)targetDeg - angle;
    if (diff > (int16_t)SERVO_SMOOTH_STEP_DEG) angle += SERVO_SMOOTH_STEP_DEG;
    else if (diff < -(int16_t)SERVO_SMOOTH_STEP_DEG) angle -= SERVO_SMOOTH_STEP_DEG;
    else angle = targetDeg;

    setServoAngle((uint8_t)angle);
    HAL_Delay(SERVO_SMOOTH_DELAY_MS);
  }
}
