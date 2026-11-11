#include "servo.h"
#include "project_config.h"

void setServoAngle(TIM_HandleTypeDef *htim, uint8_t angle)
{
  if (angle > SERVO_MAX_ANGLE_DEG) angle = SERVO_MAX_ANGLE_DEG;
  uint32_t pulseUs = SERVO_MIN_PULSE_US +
                     ((uint32_t)angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) / SERVO_MAX_ANGLE_DEG;
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, pulseUs);
}

uint8_t getServoAngle(TIM_HandleTypeDef *htim)
{
  uint32_t pulseUs = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_2);
  uint32_t span = SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US;

  if (pulseUs < SERVO_MIN_PULSE_US) pulseUs = SERVO_MIN_PULSE_US;
  if (pulseUs > SERVO_MAX_PULSE_US) pulseUs = SERVO_MAX_PULSE_US;
  return (uint8_t)(((pulseUs - SERVO_MIN_PULSE_US) * SERVO_MAX_ANGLE_DEG + span / 2u) / span);
}

// Quay servo dần tới góc target, tránh momen đột ngột làm rung cảm biến
void moveServoSmooth(TIM_HandleTypeDef *htim, uint8_t target)
{
  if (target > SERVO_MAX_ANGLE_DEG) target = SERVO_MAX_ANGLE_DEG;
  int16_t angle = (int16_t)getServoAngle(htim);

  while (angle != (int16_t)target)
  {
    int16_t diff = (int16_t)target - angle;
    if (diff > (int16_t)SERVO_SMOOTH_STEP_DEG) angle += SERVO_SMOOTH_STEP_DEG;
    else if (diff < -(int16_t)SERVO_SMOOTH_STEP_DEG) angle -= SERVO_SMOOTH_STEP_DEG;
    else angle = target;

    setServoAngle(htim, (uint8_t)angle);
    HAL_Delay(SERVO_SMOOTH_DELAY_MS);
  }
}

void setServoRange(TIM_HandleTypeDef *htim, uint8_t angleRange, ServoDirection direction)
{
  int16_t target = (int16_t)getServoAngle(htim) +
                   (direction == SERVO_FORWARD ? (int16_t)angleRange : -(int16_t)angleRange);
  int16_t maxAngle = (int16_t)SERVO_MAX_ANGLE_DEG;
  moveServoSmooth(htim, (uint8_t)(target < 0 ? 0 : (target > maxAngle ? maxAngle : target)));
}
