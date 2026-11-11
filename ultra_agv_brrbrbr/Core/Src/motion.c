#include "motion.h"
#include "project_config.h"
#include <stdlib.h>

void motionInit(void)
{
  motorInit();
}

static uint8_t addOffset(uint8_t speedPercent, int8_t offset)
{
  int16_t s = (int16_t)speedPercent + offset;
  return (uint8_t)(s < 0 ? 0 : (s > 100 ? 100 : s));
}

static void setBoth(MotorDir dirL, int8_t trimL, MotorDir dirR, int8_t trimR, uint8_t speedPercent)
{
  if (speedPercent == 0)
  {
    motorStopAll();
    return;
  }
  motorSetSpeed(MOTOR_LEFT, dirL, addOffset(speedPercent, trimL));
  motorSetSpeed(MOTOR_RIGHT, dirR, addOffset(speedPercent, trimR));
}

static void motionRun(MotorDir dir, uint8_t speedPercent)
{
  setBoth(dir, MOTOR_TRIM_LEFT_OFFSET, dir, MOTOR_TRIM_RIGHT_OFFSET, speedPercent);
}

void motionForward(uint8_t speedPercent)  { motionRun(MOTOR_FORWARD, speedPercent); }
void motionBackward(uint8_t speedPercent) { motionRun(MOTOR_BACKWARD, speedPercent); }
void motionBrake(void)                    { motorBrakeAll(); }
void motionStop(void)                     { motorStopAll(); }

void motionSpinLeft(uint8_t speedPercent)
{
  setBoth(MOTOR_BACKWARD, SPIN_TRIM_LEFT_OFFSET, MOTOR_FORWARD, SPIN_TRIM_RIGHT_OFFSET, speedPercent);
}

void motionSpinRight(uint8_t speedPercent)
{
  setBoth(MOTOR_FORWARD, SPIN_TRIM_LEFT_OFFSET, MOTOR_BACKWARD, SPIN_TRIM_RIGHT_OFFSET, speedPercent);
}

void motionSpinAngle(int16_t angleDeg, uint8_t speedPercent)
{
  if (angleDeg == 0 || speedPercent == 0)
    return;

  if (angleDeg > 0) motionSpinRight(speedPercent);
  else motionSpinLeft(speedPercent);

  HAL_Delay((uint32_t)((float)abs(angleDeg) * SPIN_MS_PER_DEGREE + SPIN_OFFSET_MS));
  motorBrakeAll();
  HAL_Delay(SPIN_BRAKE_DELAY_MS);
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
  motorBrakeAll();
  HAL_Delay(DRIVE_BRAKE_DELAY_MS);
}
