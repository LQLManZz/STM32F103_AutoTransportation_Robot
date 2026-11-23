#include "destination.h"
#include "blackbox.h"
#include "debug.h"
#include "encoder.h"
#include "imu.h"
#include "motion.h"
#include "motor.h"
#include "project_config.h"
#include "servo.h"
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

typedef enum
{
  DEST_ACT_SPIN = 0,
  DEST_ACT_DRIVE
} DestActionType;

typedef struct
{
  uint8_t type;
  int16_t value;        // SPIN: độ (>0 CW, <0 CCW); DRIVE: cm (>0 tiến, <0 lùi)
  float prevHeadingDeg; // Hướng IMU (CCW+) trước khi thực hiện bước
} DestAction;

static DestAction actionStack[DEST_MAX_ACTIONS];
static uint8_t actionCount = 0u;
static bool stackOverflow = false;

static float normalizeDeg(float deg)
{
  while (deg > 180.0f) deg -= 360.0f;
  while (deg <= -180.0f) deg += 360.0f;
  return deg;
}

static int16_t normalizeSpinDeg(int32_t deg)
{
  while (deg > 180) deg -= 360;
  while (deg <= -180) deg += 360;
  return (int16_t)deg;
}

// Xoay tại chỗ và cập nhật góc ước lượng cho pose
static void execSpinAndUpdate(int16_t spinCmdDeg, NavPose *pose)
{
  if (spinCmdDeg == 0)
    return;

  servoStop();
  motionSpinAnglePid(spinCmdDeg, PID_TARGET_SPIN_RPM);
  if (!pose)
    return;

  if (imuIsReady())
  {
    pose->thetaDeg = imuGetHeadingDeg();
  }
  else
  {
    uint32_t avgPulses = (encoderGetPulses(MOTOR_LEFT) + encoderGetPulses(MOTOR_RIGHT)) / 2u;
    float actualDeg = ((float)avgPulses * 360.0f) / (float)ENC_SPIN_360_PULSES;
    pose->thetaDeg = normalizeDeg(pose->thetaDeg + ((spinCmdDeg < 0) ? actualDeg : -actualDeg));
  }
}

// Chạy thẳng/lùi theo cự ly và cập nhật tọa độ (x, y) cho pose
static void execDriveAndUpdate(int16_t driveCm, NavPose *pose)
{
  if (driveCm == 0)
    return;

  servoStop();
  motionDriveDistancePid(driveCm, PID_TARGET_DRIVE_RPM);
  if (!pose)
    return;

  uint32_t avgPulses = (encoderGetPulses(MOTOR_LEFT) + encoderGetPulses(MOTOR_RIGHT)) / 2u;
  float actualDistCm = ((float)avgPulses * WHEEL_CIRCUMFERENCE_CM) / (float)ENC_PULSES_PER_REV;
  float sign = (driveCm > 0) ? 1.0f : -1.0f;
  float rad = pose->thetaDeg * (MATH_PI / 180.0f);
  pose->xCm += sign * actualDistCm * cosf(rad);
  pose->yCm += sign * actualDistCm * sinf(rad);
  if (imuIsReady())
    pose->thetaDeg = imuGetHeadingDeg();
}

void destinationReset(void)
{
  actionCount = 0u;
  stackOverflow = false;
}

void destinationRecordSpin(int16_t actualSpinDeg, float prevHeadingDeg)
{
  int16_t normSpin = normalizeSpinDeg((int32_t)actualSpinDeg);
  if (normSpin == 0)
    return;

  // Gộp các bước xoay liên tiếp (giữ nguyên mốc prevHeadingDeg ban đầu của cụm xoay)
  if (actionCount > 0u && actionStack[actionCount - 1u].type == DEST_ACT_SPIN)
  {
    int16_t merged = normalizeSpinDeg((int32_t)actionStack[actionCount - 1u].value + (int32_t)normSpin);
    if (abs(merged) <= DEST_MIN_SPIN_DEG)
      actionCount--;
    else
      actionStack[actionCount - 1u].value = merged;
    return;
  }

  if (abs(normSpin) <= DEST_MIN_SPIN_DEG)
    return;

  if (actionCount < DEST_MAX_ACTIONS)
  {
    actionStack[actionCount].type = DEST_ACT_SPIN;
    actionStack[actionCount].value = normSpin;
    actionStack[actionCount].prevHeadingDeg = prevHeadingDeg;
    actionCount++;
  }
  else if (!stackOverflow)
  {
    stackOverflow = true;
    debugLog("DEST_OVERFLOW max=%u\r\n", DEST_MAX_ACTIONS);
  }
}

void destinationRecordDrive(int16_t actualDriveCm, float prevHeadingDeg)
{
  if (abs(actualDriveCm) < DEST_MIN_DRIVE_CM)
    return;

  // Gộp các bước chạy thẳng/lùi liên tiếp để triệt tiêu các nhịp vi chỉnh thừa
  if (actionCount > 0u && actionStack[actionCount - 1u].type == DEST_ACT_DRIVE)
  {
    int32_t merged = (int32_t)actionStack[actionCount - 1u].value + (int32_t)actualDriveCm;
    if (merged > 32767) merged = 32767;
    if (merged < -32768) merged = -32768;
    if (abs((int)merged) < DEST_MIN_DRIVE_CM)
      actionCount--;
    else
      actionStack[actionCount - 1u].value = (int16_t)merged;
    return;
  }

  if (actionCount < DEST_MAX_ACTIONS)
  {
    actionStack[actionCount].type = DEST_ACT_DRIVE;
    actionStack[actionCount].value = actualDriveCm;
    actionStack[actionCount].prevHeadingDeg = prevHeadingDeg;
    actionCount++;
  }
  else if (!stackOverflow)
  {
    stackOverflow = true;
    debugLog("DEST_OVERFLOW max=%u\r\n", DEST_MAX_ACTIONS);
  }
}

uint8_t destinationGetCount(void)
{
  return actionCount;
}

void destinationReturnHome(NavPose *pose)
{
  debugLog("DEST_RETURN_START steps=%u overflow=%u\r\n", actionCount, stackOverflow ? 1u : 0u);
  debugState("RETURN", "GO_HOME");

  for (int16_t i = (int16_t)actionCount - 1; i >= 0; i--)
  {
    blackboxPollUartCmd();
    DestAction act = actionStack[i];

    if (act.type == DEST_ACT_DRIVE)
    {
      if (imuIsReady())
      {
        float diffMath = normalizeDeg(act.prevHeadingDeg - imuGetHeadingDeg());
        if (fabsf(diffMath) > (float)DEST_HEADING_TRIM_DEG)
        {
          int16_t trimCmd = (int16_t)lroundf(-diffMath);
          debugLog("DEST_STEP idx=%d type=ALIGN cmd=%ddeg\r\n", i, trimCmd);
          execSpinAndUpdate(trimCmd, pose);
          HAL_Delay(NAV_STEP_PAUSE_MS);
        }
      }

      int16_t revDrive = (int16_t)(-act.value);
      debugLog("DEST_STEP idx=%d type=DRIVE cmd=%dcm\r\n", i, revDrive);
      execDriveAndUpdate(revDrive, pose);
      HAL_Delay(NAV_STEP_PAUSE_MS);
    }
    else
    {
      int16_t revSpin = imuIsReady()
                            ? (int16_t)lroundf(-normalizeDeg(act.prevHeadingDeg - imuGetHeadingDeg()))
                            : (int16_t)(-act.value);
      if (abs(revSpin) > DEST_MIN_SPIN_DEG)
      {
        debugLog("DEST_STEP idx=%d type=SPIN cmd=%ddeg\r\n", i, revSpin);
        execSpinAndUpdate(revSpin, pose);
        HAL_Delay(NAV_STEP_PAUSE_MS);
      }
    }
  }

  actionCount = 0u;
  stackOverflow = false;
  motorStopAll();
  debugLog("DEST_RETURN_DONE pos=(%s,%s,%s)\r\n",
           debugF1(pose ? pose->xCm : 0.0f),
           debugF1(pose ? pose->yCm : 0.0f),
           debugF1(pose ? pose->thetaDeg : 0.0f));
}
