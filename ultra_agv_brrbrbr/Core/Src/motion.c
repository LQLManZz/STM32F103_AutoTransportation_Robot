#include "motion.h"
#include "project_config.h"
#include "debug.h"
#include <stdio.h>
#include <stdlib.h>

void motionInit(void)
{
  motorInit();
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

  HAL_Delay((uint32_t)((float)abs(angleDeg) * SPIN_MS_PER_DEGREE + SPIN_OFFSET_MS));
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
    HAL_Delay(totalMs);
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
    HAL_Delay(totalMs - rampUpMs - rampDownMs);
    rampRun(dir, speedPercent, startSpeed, rampDownMs);
  }
  motionBrake();
  HAL_Delay(DRIVE_BRAKE_DELAY_MS);
  motionStop();
}
