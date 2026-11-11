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

extern TIM_HandleTypeDef htim2;

typedef struct
{
  float integral;
  float prevErrorCm;
  bool hasPrev;
  uint32_t lastTimeMs;
} DistancePid;

static bool arrived = false;

static uint8_t pidCompute(DistancePid *pid, float errorCm, float *pTerm, float *iTerm, float *dTerm)
{
  uint32_t now = HAL_GetTick();
  float dtS = (float)(now - pid->lastTimeMs) / 1000.0f;
  float derivative = 0.0f;

  if (dtS <= 0.0f || dtS > PID_MAX_DT_S)
    dtS = (float)PID_SAMPLE_TIME_MS / 1000.0f;
  pid->lastTimeMs = now;

  if (errorCm < PID_I_ZONE_CM)
  {
    pid->integral += errorCm * dtS;
    if (pid->integral > PID_I_MAX) pid->integral = PID_I_MAX;
  }
  else
  {
    pid->integral = 0.0f;
  }

  if (pid->hasPrev)
    derivative = (errorCm - pid->prevErrorCm) / dtS;
  pid->prevErrorCm = errorCm;
  pid->hasPrev = true;

  *pTerm = PID_KP * errorCm;
  *iTerm = PID_KI * pid->integral;
  *dTerm = PID_KD * derivative;

  float out = *pTerm + *iTerm + *dTerm;
  if (out < (float)PID_SPEED_MIN) out = (float)PID_SPEED_MIN;
  if (out > (float)PID_SPEED_MAX) out = (float)PID_SPEED_MAX;
  return (uint8_t)out;
}

// Xoay bù tại chỗ sau khi dừng PID để đưa vật từ mũi cảm biến sang vùng tay gắp bên phải
static void alignGripper(float d05StopCm, float d04StopCm)
{
  motionSpinAngle(GRIPPER_ALIGN_ANGLE_DEG, SPEED_SPIN_CALM);
  if (GRIPPER_ALIGN_DIST_CM != 0)
    motionDriveDistance(GRIPPER_ALIGN_DIST_CM, PID_SPEED_MIN);
  debugGripReady(d05StopCm, d04StopCm, GRIPPER_ALIGN_ANGLE_DEG, GRIPPER_ALIGN_DIST_CM);
}

static bool drivePid(void)
{
  DistancePid pid = {0.0f, 0.0f, false, HAL_GetTick()};
  uint32_t startMs = HAL_GetTick();
  uint8_t lostCount = 0;
  float d05RawCm = ECHO_ERROR_TIMEOUT;
  float d04RawCm = ECHO_ERROR_TIMEOUT;

  while ((HAL_GetTick() - startMs) < PID_TIMEOUT_MS)
  {
    uint32_t elapsedMs = HAL_GetTick() - startMs;
    measureDualDistanceCM(&d05RawCm, &d04RawCm);
    ScanTargetType type = scanClassifyTarget(d05RawCm, d04RawCm);

    if (type == TARGET_OBSTACLE)
    {
      motionBrake();
      debugDriveEnd(elapsedMs, d05RawCm, d04RawCm, lostCount, "OBSTACLE");
      return false;
    }

    if (d05RawCm >= 0.0f && (d05RawCm < SCAN_MIN_RANGE_CM || (d05RawCm - PID_DIST_TARGET_CM) <= PID_DIST_TOLERANCE_CM))
    {
      motionBrake();
      debugDriveEnd(elapsedMs, d05RawCm, d04RawCm, lostCount, "DRV_DONE");
      alignGripper(d05RawCm, d04RawCm);
      return true;
    }

    if (type == TARGET_NONE)
    {
      if (++lostCount >= PID_LOST_MAX) break;
      HAL_Delay(PID_SAMPLE_TIME_MS);
      continue;
    }
    lostCount = 0;

    float errorCm = d05RawCm - PID_DIST_TARGET_CM;
    float pTerm = 0.0f, iTerm = 0.0f, dTerm = 0.0f;
    uint8_t pwm = pidCompute(&pid, errorCm, &pTerm, &iTerm, &dTerm);

    debugDrive(elapsedMs, d05RawCm, d04RawCm, errorCm, pTerm, iTerm, dTerm, pwm, lostCount);
    motionForward(pwm);
    HAL_Delay(PID_SAMPLE_TIME_MS);
  }
  motionBrake();
  debugDriveEnd(HAL_GetTick() - startMs, d05RawCm, d04RawCm, lostCount, "DRV_LOST");
  return false;
}

// Xoay sơ bộ theo góc cực, quét hẹp vi chỉnh hướng mũi xe rồi chạy PID tiếp cận và đưa vào tay gắp
static bool alignAndApproach(int16_t coarseSpinDeg)
{
  float fineDist = 0.0f;
  uint8_t fineAngle = SCAN_HOME_ANGLE;
  int16_t fineSpinDeg = 0;

  moveServoSmooth(&htim2, SCAN_HOME_ANGLE);
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

  return drivePid();
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

  if (arrived || scanForObject(&distanceCm, &angleDeg) != SCAN_OBJECT_FOUND)
    return;

  arrived = alignAndApproach((int16_t)SCAN_HOME_ANGLE - (int16_t)angleDeg);
}
