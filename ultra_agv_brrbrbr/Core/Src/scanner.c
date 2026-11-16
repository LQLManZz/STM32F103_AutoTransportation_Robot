#include "scanner.h"
#include "project_config.h"
#include "debug.h"
#include "servo.h"
#include "srf05.h"
#include <stdbool.h>

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

// Đo liên tục SCAN_DWELL_MS tại góc angleDeg; detectWall = true cho phép bắt tường ở hướng thẳng
static ScanStatus scanDwell(uint8_t angleDeg, bool detectWall, float *distanceCm, uint8_t *outAngleDeg)
{
  uint32_t start = HAL_GetTick();
  uint8_t hitCount = 0;
  uint8_t wallCount = 0;

  while ((HAL_GetTick() - start) < SCAN_DWELL_MS)
  {
    float d05RawCm = ECHO_ERROR_TIMEOUT;
    float d04RawCm = ECHO_ERROR_TIMEOUT;
    measureDualDistanceCM(&d05RawCm, &d04RawCm);

    ScanTargetType type = scanClassifyTarget(d05RawCm, d04RawCm);
    hitCount = (type == TARGET_OBJECT) ? (uint8_t)(hitCount + 1u) : 0u;
    wallCount = (detectWall && type == TARGET_OBSTACLE) ? (uint8_t)(wallCount + 1u) : 0u;

    debugScan(d05RawCm, d04RawCm, angleDeg, hitCount, type);
    HAL_Delay(SCAN_PERIOD_MS);

    if (hitCount >= SCAN_CONFIRM_COUNT)
    {
      *distanceCm = d05RawCm;
      *outAngleDeg = angleDeg;
      HAL_Delay(SCAN_FOUND_HOLD_MS);
      return SCAN_OBJECT_FOUND;
    }
    if (wallCount >= SCAN_CONFIRM_COUNT)
    {
      *distanceCm = d05RawCm;
      *outAngleDeg = angleDeg;
      debugWallDetected(d05RawCm, d04RawCm);
      HAL_Delay(WALL_HOLD_MS);
      return SCAN_WALL_FOUND;
    }
  }
  return ERROR_NO_OBJECTIVE;
}

// Quét từng bước stepDeg từ HOME tới biên 0°..180°, dừng khi thấy vật cần gắp
static bool scanSweep(int16_t stepDeg, float *distanceCm, uint8_t *angleDeg)
{
  for (int16_t a = (int16_t)SCAN_HOME_ANGLE + stepDeg;
       a >= 0 && a <= (int16_t)SERVO_MAX_ANGLE_DEG;
       a += stepDeg)
  {
    moveServoSmooth((uint8_t)a);
    HAL_Delay(SCAN_SETTLE_MS);
    if (scanDwell((uint8_t)a, false, distanceCm, angleDeg) == SCAN_OBJECT_FOUND)
      return true;
  }
  return false;
}

void scanInit(void)
{
  servoInit();
}

ScanStatus scanForObject(float *distanceCm, uint8_t *angleDeg)
{
  debugDecision("SCANNING");
  moveServoSmooth(SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);
  ScanStatus status = scanDwell(SCAN_HOME_ANGLE, true, distanceCm, angleDeg);
  if (status != ERROR_NO_OBJECTIVE)
    return status;

  if (scanSweep((int16_t)SCAN_STEP_DEG, distanceCm, angleDeg))
    return SCAN_OBJECT_FOUND;

  moveServoSmooth(SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);
  return scanSweep(-(int16_t)SCAN_STEP_DEG, distanceCm, angleDeg) ? SCAN_OBJECT_FOUND : ERROR_NO_OBJECTIVE;
}

ScanStatus scanFine(float *distanceCm, uint8_t *angleDeg)
{
  float bestDist = SCAN_MAX_RANGE_CM + 1.0f;
  uint8_t bestAngle = SCAN_HOME_ANGLE;

  debugDecision("SCANNING");
  for (uint8_t a = SCAN_FINE_MIN_DEG; a <= SCAN_FINE_MAX_DEG; a += SCAN_FINE_STEP_DEG)
  {
    setServoAngle(a);
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
  moveServoSmooth(SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);
  if (bestDist > SCAN_MAX_RANGE_CM)
    return ERROR_NO_OBJECTIVE;

  *distanceCm = bestDist;
  *angleDeg = bestAngle;
  return SCAN_OBJECT_FOUND;
}
