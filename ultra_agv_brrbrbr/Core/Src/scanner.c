#include "scanner.h"
#include "project_config.h"
#include "debug.h"
#include "servo.h"
#include "srf05.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

typedef struct
{
  bool valid;
  uint8_t startDeg;
  uint8_t endDeg;
  uint8_t count;
  float avgDist;
} ScanCluster;

ScanTargetType scanClassifyTarget(float d05RawCm, float d04RawCm)
{
  if (d05RawCm < SCAN_MIN_RANGE_CM || d05RawCm > SCAN_MAX_RANGE_CM)
    return TARGET_NONE;

  if (d04RawCm >= SCAN_MIN_RANGE_CM)
  {
    float d04AdjCm = d04RawCm - SENSOR_DEPTH_OFFSET_CM;
    if (d04AdjCm <= (d05RawCm + SCAN_OBSTACLE_TOLERANCE_CM))
      return TARGET_OBSTACLE;
  }
  return TARGET_OBJECT;
}

// Đo lại nhanh khoảng cách tại góc trung tâm vừa tìm được (loại mẫu nhảy nền phía sau)
static float measureCenterDistance(uint8_t centerDeg, float fallbackDistCm)
{
  moveServoSmooth(centerDeg);
  HAL_Delay(SCAN_SETTLE_MS);

  float sum = 0.0f;
  uint8_t valid = 0u;
  for (uint8_t i = 0u; i < SCAN_CONFIRM_COUNT; i++)
  {
    float d05 = ECHO_ERROR_TIMEOUT;
    float d04 = ECHO_ERROR_TIMEOUT;
    measureDualDistanceCM(&d05, &d04);
    if (scanClassifyTarget(d05, d04) == TARGET_OBJECT &&
        fabsf(d05 - fallbackDistCm) <= SCAN_CLUSTER_DEPTH_JUMP_CM)
    {
      sum += d05;
      valid++;
    }
    HAL_Delay(SCAN_PERIOD_MS);
  }
  return (valid > 0u) ? (sum / (float)valid) : fallbackDistCm;
}

// Cập nhật cụm vật hợp lệ tốt nhất theo tiêu chí ưu tiên vật gần nhất
static void considerCluster(ScanCluster *best, uint8_t startDeg, uint8_t endDeg,
                            uint8_t count, float distSum)
{
  if (count < SCAN_CONFIRM_COUNT)
    return;

  float avgDist = distSum / (float)count;
  if (!best->valid || avgDist < best->avgDist)
  {
    best->valid = true;
    best->startDeg = startDeg;
    best->endDeg = endDeg;
    best->count = count;
    best->avgDist = avgDist;
  }
}

// Quét liên tục từ startDeg tới endDeg: tách cụm khi nhảy độ sâu và ưu tiên chọn vật gần nhất
static ScanStatus scanArcCluster(uint8_t arcStartDeg, uint8_t arcEndDeg, uint8_t stepDeg,
                                 bool stopOnFirstCluster, float *distanceCm, uint8_t *angleDeg)
{
  bool inCluster = false;
  uint8_t curStart = 0u, curEnd = 0u, curCount = 0u;
  float curDistSum = 0.0f;
  float curLastDist = 0.0f;

  ScanCluster best = {0};

  uint8_t wallHits = 0u;
  float wallDist = ECHO_ERROR_TIMEOUT;

  debugDecision("SCANNING");
  moveServoSmooth(arcStartDeg);
  HAL_Delay(SCAN_SETTLE_MS);

  for (uint16_t a = arcStartDeg; a <= (uint16_t)arcEndDeg; a += stepDeg)
  {
    setServoAngle((uint8_t)a);
    HAL_Delay(SCAN_FINE_SETTLE_MS);

    float d05RawCm = ECHO_ERROR_TIMEOUT;
    float d04RawCm = ECHO_ERROR_TIMEOUT;
    measureDualDistanceCM(&d05RawCm, &d04RawCm);
    ScanTargetType type = scanClassifyTarget(d05RawCm, d04RawCm);

    if (type == TARGET_OBJECT)
    {
      if (inCluster && fabsf(d05RawCm - curLastDist) > SCAN_CLUSTER_DEPTH_JUMP_CM)
      {
        considerCluster(&best, curStart, curEnd, curCount, curDistSum);
        inCluster = false;
        curCount = 0u;
        if (stopOnFirstCluster && best.valid)
        {
          debugScan(d05RawCm, d04RawCm, (uint8_t)a, curCount, type);
          break;
        }
      }

      if (!inCluster)
      {
        inCluster = true;
        curStart = (uint8_t)a;
        curCount = 0u;
        curDistSum = 0.0f;
      }
      curEnd = (uint8_t)a;
      curCount++;
      curDistSum += d05RawCm;
      curLastDist = d05RawCm;
      debugScan(d05RawCm, d04RawCm, (uint8_t)a, curCount, type);
    }
    else
    {
      if (inCluster)
      {
        considerCluster(&best, curStart, curEnd, curCount, curDistSum);
        inCluster = false;
        curCount = 0u;
      }

      if (a >= 80u && a <= 100u && type == TARGET_OBSTACLE)
      {
        wallHits++;
        wallDist = d05RawCm;
      }

      debugScan(d05RawCm, d04RawCm, (uint8_t)a, curCount, type);
      if (stopOnFirstCluster && best.valid)
        break;
    }
  }

  if (inCluster)
    considerCluster(&best, curStart, curEnd, curCount, curDistSum);

  if (best.valid)
  {
    uint8_t centerDeg = (uint8_t)(((uint16_t)best.startDeg + (uint16_t)best.endDeg) / 2u);
    *angleDeg = centerDeg;
    *distanceCm = measureCenterDistance(centerDeg, best.avgDist);
    printf("CLUSTER arc=[%u..%u] start=%u end=%u hits=%u -> center=%u d=%s\r\n",
           arcStartDeg, arcEndDeg, best.startDeg, best.endDeg, best.count,
           centerDeg, debugF1(*distanceCm));
    HAL_Delay(SCAN_FOUND_HOLD_MS);
    return SCAN_OBJECT_FOUND;
  }

  moveServoSmooth(SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);

  if (wallHits >= SCAN_CONFIRM_COUNT)
  {
    *distanceCm = wallDist;
    *angleDeg = SCAN_HOME_ANGLE;
    debugWallDetected(wallDist, wallDist + SENSOR_DEPTH_OFFSET_CM);
    HAL_Delay(WALL_HOLD_MS);
    return SCAN_WALL_FOUND;
  }
  printf("SCAN_EMPTY arc=[%u..%u] wallHits=%u\r\n", arcStartDeg, arcEndDeg, wallHits);
  return ERROR_NO_OBJECTIVE;
}

void scanInit(void)
{
  servoInit();
}

ScanStatus scanSideArc(ScanSide side, float *distanceCm, uint8_t *angleDeg)
{
  if (side == SCAN_SIDE_LEFT)
    return scanArcCluster(SCAN_HOME_ANGLE, SERVO_MAX_ANGLE_DEG, SCAN_STEP_DEG, false, distanceCm, angleDeg);
  if (side == SCAN_SIDE_RIGHT)
    return scanArcCluster(0u, SCAN_HOME_ANGLE, SCAN_STEP_DEG, false, distanceCm, angleDeg);
  return scanArcCluster(0u, SERVO_MAX_ANGLE_DEG, SCAN_STEP_DEG, false, distanceCm, angleDeg);
}

ScanStatus scanForObject(float *distanceCm, uint8_t *angleDeg)
{
  return scanArcCluster(0u, SERVO_MAX_ANGLE_DEG, SCAN_STEP_DEG, false, distanceCm, angleDeg);
}

ScanStatus scanFine(float *distanceCm, uint8_t *angleDeg)
{
  ScanStatus status = scanArcCluster(SCAN_FINE_MIN_DEG, SCAN_FINE_MAX_DEG, SCAN_FINE_STEP_DEG,
                                     false, distanceCm, angleDeg);
  moveServoSmooth(SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);
  return status;
}
