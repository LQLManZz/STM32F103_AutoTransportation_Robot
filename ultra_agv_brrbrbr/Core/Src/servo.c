#include "servo.h"
#include "project_config.h"

// Đặt góc cố định cho servo
void setServoAngle(TIM_HandleTypeDef *htim, uint8_t angle)
{
  if (angle > 180)
  {
    angle = 180;
  }

  uint32_t pulseUs = SERVO_MIN_PULSE_US +
                         ((uint32_t)angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) / 180u;

  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, pulseUs);
}

// Quay servo dần tới góc target, mỗi bước SERVO_SMOOTH_STEP_DEG; tránh momen đột ngột làm rung cảm biến
void moveServoSmooth(TIM_HandleTypeDef *htim, uint8_t target)
{
  if (target > 180)
  {
    target = 180;
  }

  int16_t angle = (int16_t)getServoAngle(htim);

  while (angle != (int16_t)target)
  {
    if (angle < (int16_t)target)
    {
      angle += SERVO_SMOOTH_STEP_DEG;
      if (angle > (int16_t)target)
      {
        angle = target;
      }
    }
    else
    {
      angle -= SERVO_SMOOTH_STEP_DEG;
      if (angle < (int16_t)target)
      {
        angle = target;
      }
    }

    setServoAngle(htim, (uint8_t)angle);
    HAL_Delay(SERVO_SMOOTH_DELAY_MS);
  }
}

// Lấy góc servo hiện tại (đọc ngược từ thanh ghi CCR)
uint8_t getServoAngle(TIM_HandleTypeDef *htim)
{
  uint32_t pulseUs = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_2);

  const uint32_t span = SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US;

  if (pulseUs < SERVO_MIN_PULSE_US)
  {
    pulseUs = SERVO_MIN_PULSE_US;
  }
  else if (pulseUs > SERVO_MAX_PULSE_US)
  {
    pulseUs = SERVO_MAX_PULSE_US;
  }

  return (uint8_t)(((pulseUs - SERVO_MIN_PULSE_US) * 180u + span / 2u) / span);
}

// Quay dần servo một góc angleRange so với vị trí hiện tại theo direction (giới hạn 0°..180°)
void setServoRange(TIM_HandleTypeDef *htim, uint8_t angleRange, ServoDirection direction)
{
  int16_t target = (int16_t)getServoAngle(htim);

  if (direction == SERVO_FORWARD)
  {
    target += (int16_t)angleRange;
  }
  else
  {
    target -= (int16_t)angleRange;
  }

  if (target < 0)
  {
    target = 0;
  }
  else if (target > 180)
  {
    target = 180;
  }

  moveServoSmooth(htim, (uint8_t)target);
}
