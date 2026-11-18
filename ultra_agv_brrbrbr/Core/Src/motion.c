#include "motion.h"
#include "project_config.h"
#include "debug.h"
#include "encoder.h"
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

static void runPidLoop(MotorDir dirL, MotorDir dirR, uint16_t targetRpm, uint32_t targetPulses,
                       uint32_t durationMs, const char *decision)
{
  if (targetRpm == 0u)
    return;

  encoderResetPulses();
  float integral[2] = {0.0f, 0.0f};
  float prevErr[2] = {0.0f, 0.0f};
  uint16_t baseDuty[2] = {PID_BASE_LEFT_DUTY, PID_BASE_RIGHT_DUTY};

  // Xung mồi phá ma sát tĩnh rồi đặt duty nền cân bằng 2 bánh
  motorSetDutyRaw(MOTOR_LEFT, dirL, PWM_TEST_KICK_DUTY);
  motorSetDutyRaw(MOTOR_RIGHT, dirR, PWM_TEST_KICK_DUTY);
  HAL_Delay(PWM_TEST_KICK_MS);

  motorSetDutyRaw(MOTOR_LEFT, dirL, baseDuty[MOTOR_LEFT]);
  motorSetDutyRaw(MOTOR_RIGHT, dirR, baseDuty[MOTOR_RIGHT]);
  debugDecision(decision);

  uint32_t startMs = HAL_GetTick();
  uint32_t lastPidMs = startMs;
  uint32_t lastDebugMs = startMs;

  while ((durationMs == 0u && targetPulses == 0u)
             ? true
             : (durationMs > 0u
                    ? ((HAL_GetTick() - startMs) < durationMs)
                    : ((encoderGetPulses(MOTOR_LEFT) + encoderGetPulses(MOTOR_RIGHT)) < (2u * targetPulses) &&
                       (HAL_GetTick() - startMs) < PID_MOVE_TIMEOUT_MS)))
  {
    uint32_t nowMs = HAL_GetTick();
    if ((nowMs - lastPidMs) >= PID_LOOP_PERIOD_MS)
    {
      lastPidMs = nowMs;
      uint32_t pL = encoderGetPulses(MOTOR_LEFT);
      uint32_t pR = encoderGetPulses(MOTOR_RIGHT);
      uint16_t rpmL = encoderGetRpm(MOTOR_LEFT);
      uint16_t rpmR = encoderGetRpm(MOTOR_RIGHT);
      float syncAdj = PID_SYNC_KP * (float)((int32_t)pL - (int32_t)pR) +
                      PID_SYNC_KD * ((float)rpmL - (float)rpmR);

      for (uint8_t i = 0; i < 2u; i++)
      {
        MotorId id = (i == 0u) ? MOTOR_LEFT : MOTOR_RIGHT;
        MotorDir dir = (id == MOTOR_LEFT) ? dirL : dirR;
        uint16_t rpm = (id == MOTOR_LEFT) ? rpmL : rpmR;
        float err = (float)targetRpm - (float)rpm;

        integral[id] += err;
        if (integral[id] > PID_INTEGRAL_MAX) integral[id] = PID_INTEGRAL_MAX;
        if (integral[id] < -PID_INTEGRAL_MAX) integral[id] = -PID_INTEGRAL_MAX;

        float deriv = err - prevErr[id];
        prevErr[id] = err;

        float dutyF = (float)baseDuty[id] + PID_KP * err + PID_KI * integral[id] +
                      PID_KD * deriv + ((id == MOTOR_LEFT) ? -syncAdj : syncAdj);
        if (dutyF < (float)PID_MIN_DUTY) dutyF = (float)PID_MIN_DUTY;
        if (dutyF > (float)MOTOR_PWM_MAX_DUTY) dutyF = (float)MOTOR_PWM_MAX_DUTY;
        motorSetDutyRaw(id, dir, (uint16_t)dutyF);
      }
    }

    if ((nowMs - lastDebugMs) >= ENC_SAMPLE_MS)
    {
      lastDebugMs = nowMs;
      debugDecision(decision);
    }
    HAL_Delay(1);
  }

  motionBrake();
  HAL_Delay(DRIVE_BRAKE_DELAY_MS);
  motionStop();
}

void motionSpinAnglePid(int16_t angleDeg, uint16_t targetRpm)
{
  if (angleDeg == 0 || targetRpm == 0u)
    return;

  char decision[OLED_LINE_CHARS + 1];
  snprintf(decision, sizeof(decision), "TURN-%d%c", abs(angleDeg), (angleDeg > 0) ? 'R' : 'L');

  MotorDir dirL = (angleDeg > 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;
  MotorDir dirR = (angleDeg > 0) ? MOTOR_BACKWARD : MOTOR_FORWARD;
  uint32_t targetPulses = ((uint32_t)abs(angleDeg) * ENC_SPIN_360_PULSES + 180u) / 360u;
  runPidLoop(dirL, dirR, targetRpm, targetPulses, 0u, decision);
}

void motionDriveDistancePid(int16_t distanceCm, uint16_t targetRpm)
{
  if (distanceCm == 0 || targetRpm == 0u)
    return;

  MotorDir dir = (distanceCm > 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;
  uint32_t targetPulses = (uint32_t)((float)abs(distanceCm) * (float)ENC_PULSES_PER_REV / WHEEL_CIRCUMFERENCE_CM + 0.5f);
  runPidLoop(dir, dir, targetRpm, targetPulses, 0u, (dir == MOTOR_FORWARD) ? "FORWARD" : "BACKWARD");
}

void motionDriveTimePid(MotorDir dir, uint32_t durationMs, uint16_t targetRpm)
{
  if ((dir != MOTOR_FORWARD && dir != MOTOR_BACKWARD) || targetRpm == 0u)
    return;

  runPidLoop(dir, dir, targetRpm, 0u, durationMs, (dir == MOTOR_FORWARD) ? "FORWARD" : "BACKWARD");
}
