#include "agv.h"
#include "project_config.h"
#include "debug.h"
#include "delay.h"
#include "motion.h"
#include "scanner.h"
#include "servo.h"
#include "srf05.h"
#include <stdbool.h>
#include <stdlib.h>

static bool arrived = false;

static void brakeAndStop(void)
{
  motionBrake();
  HAL_Delay(DRIVE_BRAKE_DELAY_MS);
  motionStop();
}

// Chạy thẳng tốc độ cố định tới cự ly mục tiêu (tạm ngắt PID)
static bool driveApproach(float targetDistCm, bool wallMode)
{
  uint32_t startMs = HAL_GetTick();
  uint8_t lostCount = 0;
  float d05RawCm = ECHO_ERROR_TIMEOUT;
  float d04RawCm = ECHO_ERROR_TIMEOUT;

  while ((HAL_GetTick() - startMs) < PID_TIMEOUT_MS)
  {
    uint32_t elapsedMs = HAL_GetTick() - startMs;
    measureDualDistanceCM(&d05RawCm, &d04RawCm);
    ScanTargetType type = scanClassifyTarget(d05RawCm, d04RawCm);

    if (!wallMode && type == TARGET_OBSTACLE)
    {
      brakeAndStop();
      debugDriveEnd(elapsedMs, d05RawCm, d04RawCm, lostCount, "OBSTACLE_FND");
      return false;
    }

    if (d05RawCm == ECHO_ERROR_TOO_CLOSE ||
        (d05RawCm >= 0.0f && d05RawCm <= targetDistCm + PID_DIST_TOLERANCE_CM))
    {
      brakeAndStop();
      debugDriveEnd(elapsedMs, d05RawCm, d04RawCm, lostCount, wallMode ? "WALL_10CM" : "DRV_DONE");
      return true;
    }

    bool validTarget = wallMode
                           ? (d05RawCm >= SCAN_MIN_RANGE_CM && d05RawCm <= SCAN_MAX_RANGE_CM)
                           : (type == TARGET_OBJECT);
    if (!validTarget)
    {
      if (++lostCount >= PID_LOST_MAX)
      {
        brakeAndStop();
        debugDriveEnd(elapsedMs, d05RawCm, d04RawCm, lostCount, wallMode ? "WALL_LOST" : "DRV_LOST");
        return false;
      }
      HAL_Delay(PID_SAMPLE_TIME_MS);
      continue;
    }
    lostCount = 0;

    motionForward(AGV_APPROACH_SPEED);
    debugApproach(elapsedMs, d05RawCm, d04RawCm, targetDistCm, wallMode);
    HAL_Delay(PID_SAMPLE_TIME_MS);
  }
  brakeAndStop();
  debugDriveEnd(HAL_GetTick() - startMs, d05RawCm, d04RawCm, lostCount, wallMode ? "WALL_TOUT" : "DRV_TOUT");
  return false;
}

// Quay servo tới góc cần kiểm tra và xác nhận hướng đó có tường hay không
static bool checkWallAtAngle(uint8_t angleDeg, float *outDistCm)
{
  uint8_t wallHits = 0;
  float lastDistCm = ECHO_ERROR_TIMEOUT;

  debugDecision("SCANNING");
  moveServoSmooth(angleDeg);
  HAL_Delay(SCAN_SETTLE_MS);

  for (uint8_t i = 0; i < WALL_CHECK_SAMPLES; i++)
  {
    float d05RawCm = ECHO_ERROR_TIMEOUT;
    float d04RawCm = ECHO_ERROR_TIMEOUT;
    measureDualDistanceCM(&d05RawCm, &d04RawCm);
    ScanTargetType type = scanClassifyTarget(d05RawCm, d04RawCm);
    bool isWall = (type == TARGET_OBSTACLE) ||
                  (d05RawCm < SCAN_MIN_RANGE_CM && d05RawCm != ECHO_ERROR_TIMEOUT);

    if (isWall)
    {
      wallHits++;
      lastDistCm = d05RawCm;
    }
    else if (lastDistCm < 0.0f && d05RawCm >= 0.0f)
    {
      lastDistCm = d05RawCm;
    }

    debugScan(d05RawCm, d04RawCm, angleDeg, wallHits, isWall ? TARGET_OBSTACLE : type);
    HAL_Delay(SCAN_PERIOD_MS);
  }
  if (outDistCm) *outDistCm = lastDistCm;
  return wallHits >= SCAN_CONFIRM_COUNT;
}

// Tiến tới sát tường cách 10cm, kiểm tra trái/phải và quay sang hướng trống (mặc định trái)
static void handleWall(void)
{
  if (!driveApproach(WALL_STOP_DIST_CM, true))
    return;

  float leftDistCm = ECHO_ERROR_TIMEOUT;
  float rightDistCm = ECHO_ERROR_TIMEOUT;
  bool leftHasWall = checkWallAtAngle(WALL_CHECK_LEFT_ANGLE, &leftDistCm);
  bool rightHasWall = checkWallAtAngle(WALL_CHECK_RIGHT_ANGLE, &rightDistCm);

  moveServoSmooth(SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);

  int16_t turnDeg = WALL_TURN_BACK_DEG;
  if (!leftHasWall)
    turnDeg = WALL_TURN_LEFT_DEG;
  else if (!rightHasWall)
    turnDeg = WALL_TURN_RIGHT_DEG;

  debugWallTurn(leftHasWall, leftDistCm, rightHasWall, rightDistCm, turnDeg);
  HAL_Delay(WALL_HOLD_MS);
  motionSpinAngle(turnDeg, SPEED_SPIN_CALM);
}

// Xoay sơ bộ theo góc cực, quét hẹp vi chỉnh hướng mũi xe rồi tiến tới vật và đưa vào tay gắp
static bool alignAndApproach(int16_t coarseSpinDeg)
{
  float fineDist = 0.0f;
  uint8_t fineAngle = SCAN_HOME_ANGLE;
  int16_t fineSpinDeg = 0;

  moveServoSmooth(SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);
  motionSpinAngle(coarseSpinDeg, SPEED_SPIN_CALM);

  bool fineFound = (scanFine(&fineDist, &fineAngle) == SCAN_OBJECT_FOUND);
  if (fineFound)
  {
    int16_t fineOffset = (int16_t)SCAN_HOME_ANGLE - (int16_t)fineAngle;
    if (abs(fineOffset) > AGV_FINE_DEADBAND_DEG)
    {
      fineSpinDeg = fineOffset;
      motionSpinAngle(fineSpinDeg, SPEED_SPIN_CALM);
    }
  }
  debugAlign(coarseSpinDeg, fineAngle, fineSpinDeg, fineDist, fineFound);

  return driveApproach(PID_DIST_TARGET_CM, false);
}

void agvInit(void)
{
  delayInit();
  debugInit();
  motionInit();
  scanInit();
}

void agvRun(void)
{
  float distanceCm = 0.0f;
  uint8_t angleDeg = SCAN_HOME_ANGLE;

  if (arrived)
    return;

  ScanStatus status = scanForObject(&distanceCm, &angleDeg);
  if (status == SCAN_OBJECT_FOUND)
    arrived = alignAndApproach((int16_t)SCAN_HOME_ANGLE - (int16_t)angleDeg);
  else if (status == SCAN_WALL_FOUND)
    handleWall();
}
