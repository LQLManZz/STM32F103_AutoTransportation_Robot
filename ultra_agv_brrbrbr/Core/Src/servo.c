#include "servo.h"
#include "project_config.h"
#include <stdbool.h>

extern TIM_HandleTypeDef htim2;

static uint8_t currentLogicalAngle = SCAN_HOME_ANGLE;
static bool servoPwmOn = false;

// Kẹp góc servo vào vùng an toàn để không cấn dây phía sau
static uint8_t clampSafeAngle(uint8_t angleDeg)
{
  if (angleDeg < SERVO_SAFE_MIN_DEG) return SERVO_SAFE_MIN_DEG;
  if (angleDeg > SERVO_SAFE_MAX_DEG) return SERVO_SAFE_MAX_DEG;
  return angleDeg;
}

void servoInit(void)
{
  setServoAngle(SCAN_HOME_ANGLE);
  HAL_Delay(SERVO_INIT_SETTLE_MS);
}

void setServoAngle(uint8_t angleDeg)
{
  angleDeg = clampSafeAngle(angleDeg);
  currentLogicalAngle = angleDeg;

  int16_t physDeg = (int16_t)angleDeg + (int16_t)SERVO_CENTER_TRIM_DEG;
  if (physDeg < 0) physDeg = 0;
  if (physDeg > (int16_t)SERVO_MAX_ANGLE_DEG) physDeg = (int16_t)SERVO_MAX_ANGLE_DEG;

  uint32_t pulseUs = SERVO_MIN_PULSE_US +
                     ((uint32_t)physDeg * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) / SERVO_MAX_ANGLE_DEG;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulseUs);
  if (!servoPwmOn)
  {
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    servoPwmOn = true;
  }
}

void servoStop(void)
{
  if (!servoPwmOn)
    return;

  // Chờ qua hẳn cửa sổ xung HIGH tối đa (2.5 ms) để không cắt cụt xung làm SG90 giật về biên
  uint32_t startMs = HAL_GetTick();
  while (__HAL_TIM_GET_COUNTER(&htim2) <= (uint32_t)(SERVO_MAX_PULSE_US + 100u))
  {
    if ((HAL_GetTick() - startMs) >= 25u)
      break;
  }

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0u);
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
  servoPwmOn = false;
}

uint8_t getServoAngle(void)
{
  return currentLogicalAngle;
}

// Quay servo dần tới góc targetDeg, tránh momen đột ngột làm rung cảm biến
void moveServoSmooth(uint8_t targetDeg)
{
  targetDeg = clampSafeAngle(targetDeg);
  int16_t angle = (int16_t)getServoAngle();

  if (angle == (int16_t)targetDeg)
  {
    if (!servoPwmOn)
      setServoAngle(targetDeg);
    return;
  }

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
