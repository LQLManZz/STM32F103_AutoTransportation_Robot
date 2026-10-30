#include "scanner.h"
#include "project_config.h"
#include "debug.h"
#include "servo.h"
#include "srf05.h"
#include <math.h>
#include <stdbool.h>

extern TIM_HandleTypeDef htim2;

// Đổi (khoảng cách, góc servo) sang tọa độ: 0° là +x, 90° là +y, 180° là -x
static void toXY(float distanceCm, uint8_t angle, float *xCm, float *yCm)
{
  float rad = (float)angle * (float)M_PI / 180.0f;

  *xCm = distanceCm * cosf(rad);
  *yCm = distanceCm * sinf(rad);
}

// Đo liên tục SCAN_DWELL_MS tại góc hiện tại, true ngay khi thấy vật
static bool scanDwell(float *xCm, float *yCm)
{
  uint32_t start = HAL_GetTick();

  while ((HAL_GetTick() - start) < SCAN_DWELL_MS)
  {
    float d = measureDistanceCM();
    uint8_t angle = getServoAngle(&htim2);
    bool found = (d >= SCAN_MIN_RANGE_CM && d <= SCAN_MAX_RANGE_CM); // d = -1, -2 tự bị loại

    toXY(d, angle, xCm, yCm);
    debugScan(d, angle, *xCm, *yCm, found);
    HAL_Delay(SCAN_PERIOD_MS);
    if (found)
    {
      HAL_Delay(SCAN_FOUND_HOLD_MS);
      return true;
    }
  }
  return false;
}

// Quay từng SCAN_STEP_DEG theo chiều dir tới hết 0°..180°, dừng khi thấy vật
static bool scanSweep(ServoDirection dir, float *xCm, float *yCm)
{
  for (;;)
  {
    uint8_t angle = getServoAngle(&htim2);

    if (dir == SERVO_FORWARD ? (angle + SCAN_STEP_DEG > 180u) : (angle < SCAN_STEP_DEG))
    {
      return false;
    }
    setServoRange(&htim2, SCAN_STEP_DEG, dir);
    HAL_Delay(SCAN_SETTLE_MS);
    if (scanDwell(xCm, yCm))
    {
      return true;
    }
  }
}

void scanInit(void)
{
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  setServoAngle(&htim2, SCAN_HOME_ANGLE);
}

ScanStatus scanForObject(float *xCm, float *yCm)
{
  moveServoSmooth(&htim2, SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);
  if (scanDwell(xCm, yCm) || scanSweep(SERVO_FORWARD, xCm, yCm))
  {
    return SCAN_OBJECT_FOUND;
  }

  // Về HOME (đã quét ở lượt xuôi) rồi quét ngược
  moveServoSmooth(&htim2, SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);
  if (scanSweep(SERVO_REVERSE, xCm, yCm))
  {
    return SCAN_OBJECT_FOUND;
  }
  return ERROR_NO_OBJECTIVE;
}
