#include "nav.h"
#include "project_config.h"
#include "debug.h"
#include "encoder.h"
#include "imu.h"
#include "motion.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static NavPose curPose = {0.0f, 0.0f, 0.0f};

static float normalizeAngleDeg(float deg)
{
  while (deg > 180.0f) deg -= 360.0f;
  while (deg <= -180.0f) deg += 360.0f;
  return deg;
}

static void previewNextAction(uint8_t wpIndex, uint8_t wpTotal, const NavPose *target,
                              const char *nextAction)
{
  uint8_t totalSec = (uint8_t)((NAV_STEP_PREVIEW_MS + 999u) / 1000u);
  bool imuOk = imuIsReady();

  for (uint8_t sec = totalSec; sec > 0u; sec--)
  {
    debugNavPreview(wpIndex, wpTotal, curPose.xCm, curPose.yCm, curPose.thetaDeg,
                    target->xCm, target->yCm, target->thetaDeg, nextAction, sec, imuOk);
    HAL_Delay(1000u);
  }
}

static void spinAndUpdatePose(uint8_t wpIndex, uint8_t wpTotal, const NavPose *target,
                              float deltaMathDeg)
{
  if (fabsf(deltaMathDeg) < NAV_ANGLE_TOLERANCE_DEG)
    return;

  // Hệ tọa độ toán học: +quay trái (CCW); motionSpinAnglePid: >0 quay phải, <0 quay trái
  int16_t spinCmdDeg = (int16_t)lroundf(-deltaMathDeg);
  if (spinCmdDeg == 0)
    return;

  char action[16];
  snprintf(action, sizeof(action), "TURN %d%c", abs(spinCmdDeg), (spinCmdDeg > 0) ? 'R' : 'L');
  previewNextAction(wpIndex, wpTotal, target, action);
  debugNavWaypoint(wpIndex, wpTotal, curPose.xCm, curPose.yCm, curPose.thetaDeg,
                   target->xCm, target->yCm, target->thetaDeg, action);

  motionSpinAnglePid(spinCmdDeg, PID_TARGET_SPIN_RPM);

  if (imuIsReady())
  {
    curPose.thetaDeg += imuGetYawDeg();
  }
  else
  {
    // Fallback nếu chưa gắn IMU: cập nhật góc theo số sườn xung đã quay
    uint32_t avgPulses = (encoderGetPulses(MOTOR_LEFT) + encoderGetPulses(MOTOR_RIGHT)) / 2u;
    float actualDeg = ((float)avgPulses * 360.0f) / (float)ENC_SPIN_360_PULSES;
    curPose.thetaDeg += (spinCmdDeg < 0) ? actualDeg : -actualDeg;
  }
  curPose.thetaDeg = normalizeAngleDeg(curPose.thetaDeg);
  HAL_Delay(NAV_STEP_PAUSE_MS);
}

static void driveAndUpdatePose(uint8_t wpIndex, uint8_t wpTotal, const NavPose *target,
                               float distCm)
{
  int16_t driveCm = (int16_t)lroundf(distCm);
  if (driveCm <= 0)
    return;

  char action[16];
  snprintf(action, sizeof(action), "FWD %dcm", driveCm);
  previewNextAction(wpIndex, wpTotal, target, action);
  debugNavWaypoint(wpIndex, wpTotal, curPose.xCm, curPose.yCm, curPose.thetaDeg,
                   target->xCm, target->yCm, target->thetaDeg, action);

  motionDriveDistancePid(driveCm, PID_TARGET_DRIVE_RPM);

  uint32_t avgPulses = (encoderGetPulses(MOTOR_LEFT) + encoderGetPulses(MOTOR_RIGHT)) / 2u;
  float actualDistCm = ((float)avgPulses * WHEEL_CIRCUMFERENCE_CM) / (float)ENC_PULSES_PER_REV;
  float midYawDeg = imuIsReady() ? (curPose.thetaDeg + 0.5f * imuGetYawDeg()) : curPose.thetaDeg;
  float rad = midYawDeg * (MATH_PI / 180.0f);
  curPose.xCm += actualDistCm * cosf(rad);
  curPose.yCm += actualDistCm * sinf(rad);
  if (imuIsReady())
    curPose.thetaDeg = normalizeAngleDeg(curPose.thetaDeg + imuGetYawDeg());
  HAL_Delay(NAV_STEP_PAUSE_MS);
}

void navResetPose(void)
{
  curPose.xCm = 0.0f;
  curPose.yCm = 0.0f;
  curPose.thetaDeg = 0.0f;
}

NavPose navGetPose(void)
{
  return curPose;
}

void navGoToPose(const NavPose *target, uint8_t wpIndex, uint8_t wpTotal)
{
  if (!target)
    return;

  float dx = target->xCm - curPose.xCm;
  float dy = target->yCm - curPose.yCm;
  float distCm = sqrtf(dx * dx + dy * dy);

  if (distCm >= NAV_DIST_TOLERANCE_CM)
  {
    float headingDeg = atan2f(dy, dx) * (180.0f / MATH_PI);
    float turn1Deg = normalizeAngleDeg(headingDeg - curPose.thetaDeg);
    spinAndUpdatePose(wpIndex, wpTotal, target, turn1Deg);
    driveAndUpdatePose(wpIndex, wpTotal, target, distCm);
  }

  float turn2Deg = normalizeAngleDeg(target->thetaDeg - curPose.thetaDeg);
  spinAndUpdatePose(wpIndex, wpTotal, target, turn2Deg);

  debugNavWaypoint(wpIndex, wpTotal, curPose.xCm, curPose.yCm, curPose.thetaDeg,
                   target->xCm, target->yCm, target->thetaDeg, "WP_OK");
}

void navFollowWaypoints(const NavPose *waypoints, uint8_t count)
{
  if (!waypoints || count == 0u)
    return;

  for (uint8_t i = 0; i < count; i++)
    navGoToPose(&waypoints[i], (uint8_t)(i + 1u), count);
}
