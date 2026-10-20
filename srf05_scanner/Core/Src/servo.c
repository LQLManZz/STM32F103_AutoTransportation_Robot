#include "servo.h"
#include "project_config.h"

// Đặt góc cố định cho servo
void setServoAngle(TIM_HandleTypeDef *htim, uint8_t angle)
{
  if (angle > 180)
  {
    angle = 180;
  }

  uint32_t pulse_servo = SERVO_MIN_PULSE_US +
                         ((uint32_t)angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) / 180u;

  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, pulse_servo);
}

// Lấy góc servo hiện tại (đọc ngược từ thanh ghi CCR)
uint8_t getServoAngle(TIM_HandleTypeDef *htim)
{
  uint32_t pulse_servo = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_2);

  const uint32_t span = SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US;

  if (pulse_servo < SERVO_MIN_PULSE_US)
  {
    pulse_servo = SERVO_MIN_PULSE_US;
  }
  else if (pulse_servo > SERVO_MAX_PULSE_US)
  {
    pulse_servo = SERVO_MAX_PULSE_US;
  }

  return (uint8_t)(((pulse_servo - SERVO_MIN_PULSE_US) * 180u + span / 2u) / span);
}

// Quay servo một góc angle_range so với vị trí hiện tại theo direction (giới hạn 0°..180°)
void setServoRange(TIM_HandleTypeDef *htim, uint8_t angle_range, ServoDirection direction)
{
  int16_t target = (int16_t)getServoAngle(htim);

  if (direction == SERVO_FORWARD)
  {
    target += (int16_t)angle_range;
  }
  else
  {
    target -= (int16_t)angle_range;
  }

  if (target < 0)
  {
    target = 0;
  }
  else if (target > 180)
  {
    target = 180;
  }

  setServoAngle(htim, (uint8_t)target);
}
