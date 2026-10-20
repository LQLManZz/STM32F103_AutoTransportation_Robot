#include "scanner.h"
#include "project_config.h"
#include "servo.h"
#include "srf05.h"
#include <math.h>
#include <stdbool.h>

// Đo liên tục SCAN_DWELL_MS, true ngay khi thấy vật trong tầm
static bool scanDwell(float *distance_cm)
{
  uint32_t start = HAL_GetTick();

  while ((HAL_GetTick() - start) < SCAN_DWELL_MS)
  {
    float d = measureDistanceCM();

    // d < 0 (-1: không có echo, -2: quá gần) tự bị loại bởi cận dưới
    if (d >= SCAN_MIN_RANGE_CM && d <= SCAN_MAX_RANGE_CM)
    {
      *distance_cm = d;
      return true;
    }
    HAL_Delay(SCAN_PERIOD_MS);
  }
  return false;
}

// Đổi (khoảng cách, góc servo) sang tọa độ Descartes: 0° là +x, 90° là +y, 180° là -x
static void toXY(float distance_cm, uint8_t angle, float *x_cm, float *y_cm)
{
  float rad = (float)angle * (float)M_PI / 180.0f;

  *x_cm = distance_cm * cosf(rad);
  *y_cm = distance_cm * sinf(rad);
}

ScanStatus scanForObject(TIM_HandleTypeDef *htim, float *x_cm, float *y_cm)
{
  float distance;

  setServoAngle(htim, SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);

  for (int dir = SERVO_FORWARD; dir <= SERVO_REVERSE; dir++)
  {
    if (dir == SERVO_REVERSE)
    {
      // Hết chiều xuôi: về HOME (đã quét ở lượt xuôi) rồi quay ngược
      setServoAngle(htim, SCAN_HOME_ANGLE);
      HAL_Delay(SCAN_SETTLE_MS);
    }
    else if (scanDwell(&distance))
    {
      toXY(distance, SCAN_HOME_ANGLE, x_cm, y_cm);
      return SCAN_OBJECT_FOUND;
    }

    for (;;)
    {
      uint8_t angle = getServoAngle(htim);
      bool can_step = (dir == SERVO_FORWARD) ? (angle + SCAN_STEP_DEG <= 180u)
                                             : (angle >= SCAN_STEP_DEG);
      if (!can_step)
      {
        break;
      }

      setServoRange(htim, SCAN_STEP_DEG, (ServoDirection)dir);
      HAL_Delay(SCAN_SETTLE_MS);
      if (scanDwell(&distance))
      {
        toXY(distance, getServoAngle(htim), x_cm, y_cm);
        return SCAN_OBJECT_FOUND;
      }
    }
  }
  return ERROR_NO_OBJECTIVE;
}
