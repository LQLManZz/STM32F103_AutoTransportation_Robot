#include "scanner.h"
#include "project_config.h"
#include "debug.h"
#include "servo.h"
#include "srf05.h"
#include <stdbool.h>

extern TIM_HandleTypeDef htim2;

ScanTargetType scanClassifyTarget(float d05RawCm, float d04RawCm)
{
  if (d05RawCm < SCAN_MIN_RANGE_CM || d05RawCm > SCAN_MAX_RANGE_CM)
    return TARGET_NONE;

  if (d04RawCm >= SCAN_MIN_RANGE_CM && d04RawCm <= (SCAN_MAX_RANGE_CM + SENSOR_DEPTH_OFFSET_CM))
  {
    float d04AdjCm = d04RawCm - SENSOR_DEPTH_OFFSET_CM;
    if (d04AdjCm <= (d05RawCm + SCAN_OBSTACLE_TOLERANCE_CM))
      return TARGET_OBSTACLE;
  }
  return TARGET_OBJECT;
}

// Đo liên tục SCAN_DWELL_MS tại góc hiện tại, true khi thấy vật cần gắp SCAN_CONFIRM_COUNT lần liên tiếp
static bool scanDwell(float *distanceCm, uint8_t *angleDeg)
{
  uint32_t start = HAL_GetTick();
  uint8_t hitCount = 0;

  while ((HAL_GetTick() - start) < SCAN_DWELL_MS)
  {
    float d05RawCm = ECHO_ERROR_TIMEOUT;
    float d04RawCm = ECHO_ERROR_TIMEOUT;
    measureDualDistanceCM(&d05RawCm, &d04RawCm);

    uint8_t angle = getServoAngle(&htim2);
    ScanTargetType type = scanClassifyTarget(d05RawCm, d04RawCm);
    hitCount = (type == TARGET_OBJECT) ? (uint8_t)(hitCount + 1u) : 0u;

    debugScan(d05RawCm, d04RawCm, angle, hitCount, type);
    HAL_Delay(SCAN_PERIOD_MS);

    if (hitCount >= SCAN_CONFIRM_COUNT)
    {
      *distanceCm = d05RawCm;
      *angleDeg = angle;
      HAL_Delay(SCAN_FOUND_HOLD_MS);
      return true;
    }
  }
  return false;
}

// Quay từng SCAN_STEP_DEG theo chiều dir tới hết 0°..180°, dừng khi thấy vật cần gắp
static bool scanSweep(ServoDirection dir, float *distanceCm, uint8_t *angleDeg)
{
  for (;;)
  {
    uint8_t angle = getServoAngle(&htim2);
    if (dir == SERVO_FORWARD ? (angle + SCAN_STEP_DEG > SERVO_MAX_ANGLE_DEG) : (angle < SCAN_STEP_DEG))
      return false;

    setServoRange(&htim2, SCAN_STEP_DEG, dir);
    HAL_Delay(SCAN_SETTLE_MS);
    if (scanDwell(distanceCm, angleDeg))
      return true;
  }
}

void scanInit(void)
{
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  setServoAngle(&htim2, SCAN_HOME_ANGLE);
}

ScanStatus scanForObject(float *distanceCm, uint8_t *angleDeg)
{
  moveServoSmooth(&htim2, SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);
  if (scanDwell(distanceCm, angleDeg) || scanSweep(SERVO_FORWARD, distanceCm, angleDeg))
    return SCAN_OBJECT_FOUND;

  moveServoSmooth(&htim2, SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);
  return scanSweep(SERVO_REVERSE, distanceCm, angleDeg) ? SCAN_OBJECT_FOUND : ERROR_NO_OBJECTIVE;
}

ScanStatus scanFine(float *distanceCm, uint8_t *angleDeg)
{
  float bestDist = SCAN_MAX_RANGE_CM + 1.0f;
  uint8_t bestAngle = SCAN_HOME_ANGLE;

  for (uint8_t a = SCAN_FINE_MIN_DEG; a <= SCAN_FINE_MAX_DEG; a += SCAN_FINE_STEP_DEG)
  {
    setServoAngle(&htim2, a);
    HAL_Delay(SCAN_FINE_SETTLE_MS);

    float d05RawCm = ECHO_ERROR_TIMEOUT;
    float d04RawCm = ECHO_ERROR_TIMEOUT;
    measureDualDistanceCM(&d05RawCm, &d04RawCm);

    ScanTargetType type = scanClassifyTarget(d05RawCm, d04RawCm);
    if (type == TARGET_OBJECT && d05RawCm < bestDist)
    {
      bestDist = d05RawCm;
      bestAngle = a;
    }
    debugScanFine(a, d05RawCm, d04RawCm, type, bestDist, bestAngle);
  }
  moveServoSmooth(&htim2, SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);
  if (bestDist > SCAN_MAX_RANGE_CM)
    return ERROR_NO_OBJECTIVE;

  *distanceCm = bestDist;
  *angleDeg = bestAngle;
  return SCAN_OBJECT_FOUND;
}
