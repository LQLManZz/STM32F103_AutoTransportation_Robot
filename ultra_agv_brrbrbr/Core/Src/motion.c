#include "motion.h"
#include "project_config.h"
#include "debug.h"
#include "encoder.h"
#include "imu.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

void motionInit(void)
{
  motorInit();
  encoderInit();
}

static void waitWithDebug(uint32_t durationMs, const char *decision)
{
  uint32_t startMs = HAL_GetTick();
  uint32_t lastDebugMs = startMs;

  while ((HAL_GetTick() - startMs) < durationMs)
  {
    uint32_t nowMs = HAL_GetTick();
    if ((nowMs - lastDebugMs) >= ENC_SAMPLE_MS)
    {
      lastDebugMs = nowMs;
      debugDecision(decision);
    }
    HAL_Delay(1);
  }
}

static void setBoth(MotorDir dirL, MotorDir dirR, uint8_t speedPercent, const char *decision)
{
  if (speedPercent == 0)
  {
    motionStop();
    return;
  }
  motorSetSpeed(MOTOR_LEFT, dirL, speedPercent);
  motorSetSpeed(MOTOR_RIGHT, dirR, speedPercent);
  if (decision)
    debugDecision(decision);
}

static void motionRun(MotorDir dir, uint8_t speedPercent)
{
  setBoth(dir, dir, speedPercent, (dir == MOTOR_FORWARD) ? "FORWARD" : "BACKWARD");
}

static void brakePulseOpposite(MotorId motor, MotorDir prevDir)
{
  if (prevDir == MOTOR_FORWARD)
    motorSetDutyRaw(motor, MOTOR_BACKWARD, BRAKE_REV_DUTY);
  else if (prevDir == MOTOR_BACKWARD)
    motorSetDutyRaw(motor, MOTOR_FORWARD, BRAKE_REV_DUTY);
  else
    motorSetDutyRaw(motor, MOTOR_BRAKE, 0);
}

void motionForward(uint8_t speedPercent)  { motionRun(MOTOR_FORWARD, speedPercent); }
void motionBackward(uint8_t speedPercent) { motionRun(MOTOR_BACKWARD, speedPercent); }

void motionBrake(void)
{
  MotorDir dirL = motorGetDir(MOTOR_LEFT);
  MotorDir dirR = motorGetDir(MOTOR_RIGHT);

  if (dirL == MOTOR_FORWARD || dirL == MOTOR_BACKWARD ||
      dirR == MOTOR_FORWARD || dirR == MOTOR_BACKWARD)
  {
    brakePulseOpposite(MOTOR_LEFT, dirL);
    brakePulseOpposite(MOTOR_RIGHT, dirR);
    HAL_Delay(BRAKE_REV_PULSE_MS);
  }
  motorBrakeAll();
  debugDecision("BRAKE");
}

void motionStop(void)                     { motorStopAll(); debugDecision("STOP"); }

void motionSpinAngle(int16_t angleDeg, uint8_t speedPercent)
{
  if (angleDeg == 0 || speedPercent == 0)
    return;

  char decision[OLED_LINE_CHARS + 1];
  snprintf(decision, sizeof(decision), "TURN-%d%c", abs(angleDeg), (angleDeg > 0) ? 'R' : 'L');

  MotorDir dirL = (angleDeg > 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;
  MotorDir dirR = (angleDeg > 0) ? MOTOR_BACKWARD : MOTOR_FORWARD;
  setBoth(dirL, dirR, speedPercent, decision);

  waitWithDebug((uint32_t)((float)abs(angleDeg) * SPIN_MS_PER_DEGREE + SPIN_OFFSET_MS), decision);
  motionBrake();
  HAL_Delay(SPIN_BRAKE_DELAY_MS);
  motionStop();
}

static void rampRun(MotorDir dir, uint8_t fromSpeed, uint8_t toSpeed, uint32_t durationMs)
{
  uint32_t steps = durationMs / RAMP_STEP_DELAY_MS;
  if (steps == 0) steps = 1;
  for (uint32_t i = 1; i <= steps; i++)
  {
    int16_t s = (int16_t)fromSpeed + ((int16_t)toSpeed - (int16_t)fromSpeed) * (int16_t)i / (int16_t)steps;
    motionRun(dir, (uint8_t)s);
    HAL_Delay(RAMP_STEP_DELAY_MS);
  }
}

void motionDriveDistance(int16_t distanceCm, uint8_t speedPercent)
{
  if (distanceCm == 0 || speedPercent == 0)
    return;

  MotorDir dir = (distanceCm > 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;
  uint32_t totalMs = (uint32_t)((float)abs(distanceCm) * DRIVE_MS_PER_CM);
  uint32_t rampUpMs = RAMP_UP_MS, rampDownMs = RAMP_DOWN_MS;
  uint8_t startSpeed = (RAMP_START_SPEED < speedPercent) ? RAMP_START_SPEED : speedPercent;

  if (totalMs < RAMP_MIN_TOTAL_MS)
  {
    motionRun(dir, speedPercent);
    waitWithDebug(totalMs, (dir == MOTOR_FORWARD) ? "FORWARD" : "BACKWARD");
  }
  else
  {
    if (rampUpMs + rampDownMs >= totalMs)
    {
      rampUpMs = totalMs / 2;
      rampDownMs = totalMs - rampUpMs;
    }
    rampRun(dir, startSpeed, speedPercent, rampUpMs);
    motionRun(dir, speedPercent);
    waitWithDebug(totalMs - rampUpMs - rampDownMs, (dir == MOTOR_FORWARD) ? "FORWARD" : "BACKWARD");
    rampRun(dir, speedPercent, startSpeed, rampDownMs);
  }
  motionBrake();
  HAL_Delay(DRIVE_BRAKE_DELAY_MS);
  motionStop();
}

static uint16_t scaleBaseDuty(uint16_t refDuty, uint16_t rpm, uint16_t minDuty)
{
  uint32_t duty = ((uint32_t)refDuty * (uint32_t)rpm) / (uint32_t)PID_REF_RPM;
  if (duty < (uint32_t)minDuty) duty = minDuty;
  if (duty > (uint32_t)MOTOR_PWM_MAX_DUTY) duty = MOTOR_PWM_MAX_DUTY;
  return (uint16_t)duty;
}

static void stepWheelPid(MotorDir dirL, MotorDir dirR, uint16_t curTargetRpm, uint16_t minDuty,
                         float integral[2], float prevErr[2], float *yawIntegral)
{
  static const uint16_t refDuty[2] = {PID_BASE_LEFT_DUTY, PID_BASE_RIGHT_DUTY};
  uint32_t pL = encoderGetPulses(MOTOR_LEFT);
  uint32_t pR = encoderGetPulses(MOTOR_RIGHT);
  uint16_t rpmL = encoderGetRpm(MOTOR_LEFT);
  uint16_t rpmR = encoderGetRpm(MOTOR_RIGHT);

  float syncAdj;
  if (dirL == dirR && imuIsReady() && yawIntegral != NULL)
  {
    // Giữ thẳng hướng bằng PID góc Yaw của IMU (tự đảo dấu lái khi lùi)
    float dirSign = (dirL == MOTOR_FORWARD) ? 1.0f : -1.0f;
    float yawErr = -imuGetYawDeg();
    *yawIntegral += yawErr;
    if (*yawIntegral > 50.0f) *yawIntegral = 50.0f;
    if (*yawIntegral < -50.0f) *yawIntegral = -50.0f;

    syncAdj = dirSign * (IMU_HEADING_KP * yawErr +
                         IMU_HEADING_KI * (*yawIntegral) -
                         IMU_HEADING_KD * imuGetRateDps());
    if (syncAdj > IMU_HEADING_MAX_ADJ) syncAdj = IMU_HEADING_MAX_ADJ;
    if (syncAdj < -IMU_HEADING_MAX_ADJ) syncAdj = -IMU_HEADING_MAX_ADJ;
  }
  else
  {
    float maxSync = (dirL != dirR) ? PID_SPIN_SYNC_MAX : IMU_HEADING_MAX_ADJ;
    if (dirL != dirR && (rpmL == 0u || rpmR == 0u))
    {
      syncAdj = 0.0f;
    }
    else
    {
      syncAdj = PID_SYNC_KP * (float)((int32_t)pL - (int32_t)pR) +
                PID_SYNC_KD * ((float)rpmL - (float)rpmR);
      if (syncAdj > maxSync) syncAdj = maxSync;
      if (syncAdj < -maxSync) syncAdj = -maxSync;
    }
  }

  for (uint8_t i = 0; i < 2u; i++)
  {
    MotorId id = (i == 0u) ? MOTOR_LEFT : MOTOR_RIGHT;
    MotorDir dir = (id == MOTOR_LEFT) ? dirL : dirR;
    uint16_t rpm = (id == MOTOR_LEFT) ? rpmL : rpmR;
    float err = (float)curTargetRpm - (float)rpm;

    integral[id] += err;
    if (integral[id] > PID_INTEGRAL_MAX) integral[id] = PID_INTEGRAL_MAX;
    if (integral[id] < -PID_INTEGRAL_MAX) integral[id] = -PID_INTEGRAL_MAX;

    float deriv = err - prevErr[id];
    prevErr[id] = err;

    uint16_t ffDuty = scaleBaseDuty(refDuty[id], curTargetRpm, minDuty);
    float dutyF = (float)ffDuty + PID_KP * err + PID_KI * integral[id] +
                  PID_KD * deriv + ((id == MOTOR_LEFT) ? -syncAdj : syncAdj);
    if (dutyF < (float)minDuty) dutyF = (float)minDuty;
    if (dutyF > (float)MOTOR_PWM_MAX_DUTY) dutyF = (float)MOTOR_PWM_MAX_DUTY;
    motorSetDutyRaw(id, dir, (uint16_t)dutyF);
  }
}

static void waitImuMs(uint32_t durationMs)
{
  uint32_t startMs = HAL_GetTick();
  uint32_t lastImuMs = startMs;

  while ((HAL_GetTick() - startMs) < durationMs)
  {
    uint32_t nowMs = HAL_GetTick();
    if (imuIsReady() && (nowMs - lastImuMs) >= IMU_LOOP_PERIOD_MS)
    {
      lastImuMs = nowMs;
      imuUpdate();
    }
    HAL_Delay(1);
  }
  if (imuIsReady())
    imuUpdate();
}

static void startKickAndBase(MotorDir dirL, MotorDir dirR, uint16_t kickDuty, uint32_t kickMs,
                             uint16_t initRpm, uint16_t minDuty, const char *decision)
{
  motorSetDutyRaw(MOTOR_LEFT, dirL, kickDuty);
  motorSetDutyRaw(MOTOR_RIGHT, dirR, kickDuty);
  waitImuMs(kickMs);

  motorSetDutyRaw(MOTOR_LEFT, dirL, scaleBaseDuty(PID_BASE_LEFT_DUTY, initRpm, minDuty));
  motorSetDutyRaw(MOTOR_RIGHT, dirR, scaleBaseDuty(PID_BASE_RIGHT_DUTY, initRpm, minDuty));
  debugDecision(decision);
}

static void runPidLoop(MotorDir dirL, MotorDir dirR, uint16_t targetRpm, uint16_t decelRpm,
                       uint32_t decelPulses, uint32_t targetPulses, uint32_t durationMs,
                       uint16_t minDuty, const char *decision)
{
  if (targetRpm == 0u)
    return;

  encoderResetPulses();
  if (imuIsReady())
    imuResetYaw();

  float integral[2] = {0.0f, 0.0f};
  float prevErr[2] = {0.0f, 0.0f};
  float yawIntegral = 0.0f;
  bool inDecel = false;

  uint32_t moveTimeoutMs = PID_MOVE_TIMEOUT_MS;
  if (targetPulses > 0u)
  {
    uint32_t estMs = targetPulses * 180u + 1500u;
    if (estMs > moveTimeoutMs)
      moveTimeoutMs = estMs;
  }

  uint16_t initRpm = targetRpm;
  if (targetPulses > 0u && decelPulses > 0u && targetPulses <= decelPulses && decelRpm < targetRpm)
    initRpm = decelRpm + (uint16_t)(((uint32_t)(targetRpm - decelRpm) * targetPulses) / decelPulses);

  // Xung mồi êm phá ma sát tĩnh rồi đặt duty nền tỉ lệ theo RPM mục tiêu
  printf("PID_START dec=%s dirL=%d dirR=%d rpm=%u->%u targetP=%lu timeout=%lums\r\n",
         decision, (int)dirL, (int)dirR, targetRpm, decelRpm, targetPulses, moveTimeoutMs);
  startKickAndBase(dirL, dirR, DRIVE_KICK_DUTY, DRIVE_KICK_MS, initRpm, minDuty, decision);

  uint32_t startMs = HAL_GetTick();
  uint32_t lastPidMs = startMs;
  uint32_t lastImuMs = startMs;
  uint32_t lastDebugMs = startMs;

  while ((durationMs == 0u && targetPulses == 0u)
             ? true
             : (durationMs > 0u
                    ? ((HAL_GetTick() - startMs) < durationMs)
                    : ((encoderGetPulses(MOTOR_LEFT) + encoderGetPulses(MOTOR_RIGHT)) < (2u * targetPulses) &&
                       (HAL_GetTick() - startMs) < moveTimeoutMs)))
  {
    uint32_t nowMs = HAL_GetTick();
    if (imuIsReady() && (nowMs - lastImuMs) >= IMU_LOOP_PERIOD_MS)
    {
      lastImuMs = nowMs;
      imuUpdate();
    }

    if ((nowMs - lastPidMs) >= PID_LOOP_PERIOD_MS)
    {
      lastPidMs = nowMs;
      uint32_t pAvg = (encoderGetPulses(MOTOR_LEFT) + encoderGetPulses(MOTOR_RIGHT)) / 2u;

      // Giảm tốc dần ở vài sườn xung cuối trước khi phanh để hạn chế trượt/vọt lố
      uint16_t curTargetRpm = targetRpm;
      if (targetPulses > 0u && decelPulses > 0u && decelRpm < targetRpm)
      {
        uint32_t remaining = (targetPulses > pAvg) ? (targetPulses - pAvg) : 0u;
        if (remaining <= decelPulses)
        {
          if (!inDecel)
          {
            inDecel = true;
            integral[MOTOR_LEFT] *= 0.5f;
            integral[MOTOR_RIGHT] *= 0.5f;
          }
          curTargetRpm = decelRpm + (uint16_t)(((uint32_t)(targetRpm - decelRpm) * remaining) / decelPulses);
        }
      }

      stepWheelPid(dirL, dirR, curTargetRpm, minDuty, integral, prevErr, &yawIntegral);
    }

    if ((nowMs - lastDebugMs) >= ENC_SAMPLE_MS)
    {
      lastDebugMs = nowMs;
      debugDecision(decision);
    }
    HAL_Delay(1);
  }

  uint32_t endMs = HAL_GetTick() - startMs;
  uint32_t finalPL = encoderGetPulses(MOTOR_LEFT);
  uint32_t finalPR = encoderGetPulses(MOTOR_RIGHT);
  const char *reason = (durationMs > 0u)
                           ? "TIME_OK"
                           : (((finalPL + finalPR) >= (2u * targetPulses)) ? "PULSES_OK" : "TIMEOUT");
  printf("PID_END dec=%s reason=%s dt=%lums pL=%lu pR=%lu targetP=%lu yaw=%s\r\n",
         decision, reason, endMs, finalPL, finalPR, targetPulses, debugF1(imuGetYawDeg()));

  motionBrake();
  waitImuMs(DRIVE_BRAKE_DELAY_MS);
  motionStop();
}

// Xoay chậm tại chỗ và đọc góc IMU liên tục theo thời gian thực tới khi đạt góc mong muốn
static void runSpinImuLoop(int16_t angleDeg, uint16_t targetRpm, uint16_t decelRpm, const char *decision)
{
  MotorDir dirL = (angleDeg > 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;
  MotorDir dirR = (angleDeg > 0) ? MOTOR_BACKWARD : MOTOR_FORWARD;
  // Quy ước motion: >0 quay phải (CW, IMU yaw âm), <0 quay trái (CCW, IMU yaw dương)
  float expectedSign = (angleDeg > 0) ? -1.0f : 1.0f;
  float targetAbsDeg = (float)abs(angleDeg);
  float targetSignedYaw = expectedSign * targetAbsDeg;
  float stopAbsDeg = (targetAbsDeg > IMU_SPIN_LEAD_DEG)
                         ? (targetAbsDeg - IMU_SPIN_LEAD_DEG)
                         : (targetAbsDeg * 0.7f);

  encoderResetPulses();
  imuResetYaw();

  float integral[2] = {0.0f, 0.0f};
  float prevErr[2] = {0.0f, 0.0f};
  bool inDecel = false;
  bool signErr = false;

  uint32_t moveTimeoutMs = (uint32_t)(targetAbsDeg * 60.0f) + 3000u;
  if (moveTimeoutMs < PID_MOVE_TIMEOUT_MS)
    moveTimeoutMs = PID_MOVE_TIMEOUT_MS;

  uint16_t initRpm = (targetAbsDeg <= IMU_SPIN_DECEL_DEG) ? decelRpm : targetRpm;
  startKickAndBase(dirL, dirR, IMU_SPIN_KICK_DUTY, IMU_SPIN_KICK_MS, initRpm, PID_SPIN_MIN_DUTY, decision);
  imuUpdate();

  uint32_t startMs = HAL_GetTick();
  uint32_t lastImuMs = startMs;
  uint32_t lastPidMs = startMs;
  uint32_t lastDebugMs = startMs;
  float turnedAbs = 0.0f;

  while ((HAL_GetTick() - startMs) < moveTimeoutMs)
  {
    uint32_t nowMs = HAL_GetTick();
    if ((nowMs - lastImuMs) >= IMU_LOOP_PERIOD_MS)
    {
      lastImuMs = nowMs;
      turnedAbs = imuUpdate() * expectedSign;
      if (turnedAbs >= stopAbsDeg)
        break;
      if (turnedAbs < IMU_SPIN_OPPOSITE_LIMIT_DEG)
      {
        signErr = true;
        debugDecision("IMU_SIGN_ERR");
        break;
      }
    }

    if ((nowMs - lastPidMs) >= PID_LOOP_PERIOD_MS)
    {
      lastPidMs = nowMs;
      float remainingDeg = targetAbsDeg - turnedAbs;
      uint16_t curTargetRpm = targetRpm;
      if (remainingDeg <= IMU_SPIN_DECEL_DEG && decelRpm < targetRpm)
      {
        if (!inDecel)
        {
          inDecel = true;
          integral[MOTOR_LEFT] *= 0.5f;
          integral[MOTOR_RIGHT] *= 0.5f;
        }
        if (remainingDeg < 0.0f)
          remainingDeg = 0.0f;
        curTargetRpm = decelRpm + (uint16_t)(((float)(targetRpm - decelRpm) * remainingDeg) / IMU_SPIN_DECEL_DEG);
      }
      stepWheelPid(dirL, dirR, curTargetRpm, PID_SPIN_MIN_DUTY, integral, prevErr, NULL);
    }

    // Tránh trễ I2C OLED ở 10° cuối để vòng đọc IMU 5ms bắt điểm dừng chính xác nhất
    if ((targetAbsDeg - turnedAbs) > 10.0f && (nowMs - lastDebugMs) >= ENC_SAMPLE_MS)
    {
      lastDebugMs = nowMs;
      debugDecision(decision);
    }
    HAL_Delay(1);
  }

  motionBrake();
  waitImuMs(SPIN_BRAKE_DELAY_MS);
  motionStop();
  waitImuMs(IMU_SPIN_SETTLE_MS);
  debugDecision(decision);

  // Kết hợp vi chỉnh: chỉ nhích bù thêm nếu lượt xoay chậm đầu tiên vẫn còn lệch quá IMU_SPIN_TOL_DEG
  uint16_t trimDuty = IMU_SPIN_TRIM_DUTY;
  for (uint8_t trim = 0u; !signErr && trim < IMU_SPIN_MAX_TRIMS; trim++)
  {
    float errDeg = targetSignedYaw - imuGetYawDeg();
    if (fabsf(errDeg) <= IMU_SPIN_TOL_DEG)
      break;

    float yawBefore = imuGetYawDeg();
    float trimSign = (errDeg > 0.0f) ? 1.0f : -1.0f;
    MotorDir tDirL = (errDeg > 0.0f) ? MOTOR_BACKWARD : MOTOR_FORWARD;
    MotorDir tDirR = (errDeg > 0.0f) ? MOTOR_FORWARD : MOTOR_BACKWARD;
    const char *trimLabel = (errDeg > 0.0f) ? "TRIM-L" : "TRIM-R";

    motorSetDutyRaw(MOTOR_LEFT, tDirL, trimDuty);
    motorSetDutyRaw(MOTOR_RIGHT, tDirR, trimDuty);

    uint32_t trimStart = HAL_GetTick();
    uint32_t trimLastImu = trimStart;
    while ((HAL_GetTick() - trimStart) < IMU_SPIN_TRIM_TIMEOUT_MS)
    {
      uint32_t nowMs = HAL_GetTick();
      if ((nowMs - trimLastImu) >= IMU_SPIN_TRIM_PERIOD_MS)
      {
        trimLastImu = nowMs;
        float rem = (targetSignedYaw - imuUpdate()) * trimSign;
        if (rem <= IMU_SPIN_TRIM_LEAD_DEG)
          break;
      }
      HAL_Delay(1);
    }

    // Pha vi chỉnh chạy rất chậm nên phanh ngắn mạch trực tiếp, tránh xung đảo chiều làm bật ngược lại
    motorBrakeAll();
    waitImuMs(60u);
    motionStop();
    waitImuMs(80u);
    if (fabsf(imuGetYawDeg() - yawBefore) < 0.5f)
    {
      trimDuty = (uint16_t)(trimDuty + IMU_SPIN_TRIM_STEP_DUTY);
      if (trimDuty > (uint16_t)MOTOR_PWM_MAX_DUTY)
        trimDuty = (uint16_t)MOTOR_PWM_MAX_DUTY;
    }
    debugDecision(trimLabel);
  }
}

void motionSpinAnglePid(int16_t angleDeg, uint16_t targetRpm)
{
  if (angleDeg == 0 || targetRpm == 0u)
    return;

  char decision[OLED_LINE_CHARS + 1];
  snprintf(decision, sizeof(decision), "TURN-%d%c", abs(angleDeg), (angleDeg > 0) ? 'R' : 'L');

  uint16_t decelRpm = (PID_DECEL_SPIN_RPM < targetRpm) ? PID_DECEL_SPIN_RPM : targetRpm;
  if (imuIsReady())
  {
    runSpinImuLoop(angleDeg, targetRpm, decelRpm, decision);
    return;
  }

  MotorDir dirL = (angleDeg > 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;
  MotorDir dirR = (angleDeg > 0) ? MOTOR_BACKWARD : MOTOR_FORWARD;
  uint32_t targetPulses = ((uint32_t)abs(angleDeg) * ENC_SPIN_360_PULSES + 180u) / 360u;
  if (targetPulses == 0u) targetPulses = 1u;
  runPidLoop(dirL, dirR, targetRpm, decelRpm, PID_DECEL_SPIN_PULSES,
             targetPulses, 0u, PID_SPIN_MIN_DUTY, decision);
}

void motionDriveDistancePid(int16_t distanceCm, uint16_t targetRpm)
{
  if (distanceCm == 0 || targetRpm == 0u)
    return;

  MotorDir dir = (distanceCm > 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;
  uint32_t targetPulses = (uint32_t)((float)abs(distanceCm) * (float)ENC_PULSES_PER_REV / WHEEL_CIRCUMFERENCE_CM + 0.5f);
  if (targetPulses == 0u) targetPulses = 1u;
  uint16_t decelRpm = (PID_DECEL_DRIVE_RPM < targetRpm) ? PID_DECEL_DRIVE_RPM : targetRpm;
  runPidLoop(dir, dir, targetRpm, decelRpm, PID_DECEL_DRIVE_PULSES,
             targetPulses, 0u, PID_MIN_DUTY, (dir == MOTOR_FORWARD) ? "FORWARD" : "BACKWARD");
}

void motionDriveTimePid(MotorDir dir, uint32_t durationMs, uint16_t targetRpm)
{
  if ((dir != MOTOR_FORWARD && dir != MOTOR_BACKWARD) || targetRpm == 0u)
    return;

  runPidLoop(dir, dir, targetRpm, targetRpm, 0u, 0u, durationMs, PID_MIN_DUTY,
             (dir == MOTOR_FORWARD) ? "FORWARD" : "BACKWARD");
}

void motionForwardHoldStart(void)
{
  encoderResetPulses();
  if (imuIsReady())
    imuResetYaw();
}

void motionForwardHoldStep(uint8_t speedPercent)
{
  if (speedPercent == 0u)
  {
    motionStop();
    return;
  }
  if (speedPercent > 100u)
    speedPercent = 100u;

  uint16_t baseL = (uint16_t)(MOTOR_PWM_MIN_DUTY +
                              ((uint32_t)speedPercent * (MOTOR_PWM_MAX_DUTY - MOTOR_PWM_MIN_DUTY)) / 100u);
  uint32_t scaledR = ((uint32_t)baseL * MOTOR_RIGHT_SCALE + (MOTOR_LEFT_SCALE / 2u)) / MOTOR_LEFT_SCALE;
  uint16_t baseR = (scaledR > MOTOR_PWM_MAX_DUTY) ? (uint16_t)MOTOR_PWM_MAX_DUTY : (uint16_t)scaledR;

  float adj;
  if (imuIsReady())
  {
    imuUpdate();
    adj = IMU_HEADING_KP * (-imuGetYawDeg()) - IMU_HEADING_KD * imuGetRateDps();
    if (adj > IMU_HEADING_MAX_ADJ) adj = IMU_HEADING_MAX_ADJ;
    if (adj < -IMU_HEADING_MAX_ADJ) adj = -IMU_HEADING_MAX_ADJ;
  }
  else
  {
    int32_t pDiff = (int32_t)encoderGetPulses(MOTOR_LEFT) - (int32_t)encoderGetPulses(MOTOR_RIGHT);
    adj = PID_SYNC_KP * (float)pDiff;
  }

  float dutyLf = (float)baseL - adj;
  float dutyRf = (float)baseR + adj;
  if (dutyLf < (float)PID_MIN_DUTY) dutyLf = (float)PID_MIN_DUTY;
  if (dutyLf > (float)MOTOR_PWM_MAX_DUTY) dutyLf = (float)MOTOR_PWM_MAX_DUTY;
  if (dutyRf < (float)PID_MIN_DUTY) dutyRf = (float)PID_MIN_DUTY;
  if (dutyRf > (float)MOTOR_PWM_MAX_DUTY) dutyRf = (float)MOTOR_PWM_MAX_DUTY;

  motorSetDutyRaw(MOTOR_LEFT, MOTOR_FORWARD, (uint16_t)dutyLf);
  motorSetDutyRaw(MOTOR_RIGHT, MOTOR_FORWARD, (uint16_t)dutyRf);
}
