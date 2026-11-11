#include "motion.h"
#include "project_config.h"
#include <stdlib.h>

void motionInit(void)
{
  motorInit();
}

// Cong offset vao toc do, gioi han 0 - 100 %
static uint8_t addOffset(uint8_t speedPercent, int8_t offset)
{
  int16_t speed = (int16_t)speedPercent + offset;

  if (speed < 0)
  {
    return 0;
  }
  if (speed > 100)
  {
    return 100;
  }
  return (uint8_t)speed;
}

// Ca 2 banh cung chieu, cong trim de xe chay thang
static void motionRun(MotorDir dir, uint8_t speedPercent)
{
  if (speedPercent == 0)
  {
    motorStopAll();
    return;
  }
  motorSetSpeed(MOTOR_LEFT, dir, addOffset(speedPercent, MOTOR_TRIM_LEFT_OFFSET));
  motorSetSpeed(MOTOR_RIGHT, dir, addOffset(speedPercent, MOTOR_TRIM_RIGHT_OFFSET));
}

// Hai banh nguoc chieu, cong trim rieng cua xoay de tam quay nam giua 2 banh
static void motionSpin(MotorDir dirLeft, MotorDir dirRight, uint8_t speedPercent)
{
  if (speedPercent == 0)
  {
    motorStopAll();
    return;
  }
  motorSetSpeed(MOTOR_LEFT, dirLeft, addOffset(speedPercent, SPIN_TRIM_LEFT_OFFSET));
  motorSetSpeed(MOTOR_RIGHT, dirRight, addOffset(speedPercent, SPIN_TRIM_RIGHT_OFFSET));
}

void motionForward(uint8_t speedPercent)
{
  motionRun(MOTOR_FORWARD, speedPercent);
}

void motionBackward(uint8_t speedPercent)
{
  motionRun(MOTOR_BACKWARD, speedPercent);
}

void motionBrake(void)
{
  motorBrakeAll();
}

void motionStop(void)
{
  motorStopAll();
}

void motionSpinLeft(uint8_t speedPercent)
{
  motionSpin(MOTOR_BACKWARD, MOTOR_FORWARD, speedPercent);
}

void motionSpinRight(uint8_t speedPercent)
{
  motionSpin(MOTOR_FORWARD, MOTOR_BACKWARD, speedPercent);
}

void motionSpinAngle(int16_t angleDeg, uint8_t speedPercent)
{
  if (angleDeg == 0 || speedPercent == 0)
  {
    return;
  }

  if (angleDeg > 0)
  {
    motionSpinRight(speedPercent);
  }
  else
  {
    motionSpinLeft(speedPercent);
  }
  HAL_Delay((uint32_t)((float)abs(angleDeg) * SPIN_MS_PER_DEGREE + SPIN_OFFSET_MS));

  motorBrakeAll(); // phanh cung chong troi
  HAL_Delay(100);
}

void motionDriveDistance(int16_t distanceCm, uint8_t speedPercent)
{
  if (distanceCm == 0 || speedPercent == 0)
  {
    return;
  }

  MotorDir dir = (distanceCm > 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;
  uint32_t totalMs = (uint32_t)((float)abs(distanceCm) * DRIVE_MS_PER_CM);
  uint32_t rampUpMs = 240;
  uint32_t rampDownMs = 210;
  uint8_t startSpeed = (RAMP_START_SPEED < speedPercent) ? RAMP_START_SPEED : speedPercent;

  if (totalMs < 500) // qua ngan: chay deu, khong ramp
  {
    motionRun(dir, speedPercent);
    HAL_Delay(totalMs);
    motorBrakeAll();
    HAL_Delay(80);
    return;
  }
  if (rampUpMs + rampDownMs >= totalMs)
  {
    rampUpMs = totalMs / 2;
    rampDownMs = totalMs - rampUpMs;
  }

  // Tang toc tu startSpeed len speedPercent
  uint32_t upSteps = rampUpMs / RAMP_STEP_DELAY_MS;
  if (upSteps == 0)
  {
    upSteps = 1;
  }
  for (uint32_t i = 1; i <= upSteps; i++)
  {
    motionRun(dir, startSpeed + (uint8_t)(((uint32_t)(speedPercent - startSpeed) * i) / upSteps));
    HAL_Delay(RAMP_STEP_DELAY_MS);
  }

  // Chay deu
  motionRun(dir, speedPercent);
  HAL_Delay(totalMs - rampUpMs - rampDownMs);

  // Giam toc ve startSpeed
  uint32_t downSteps = rampDownMs / RAMP_STEP_DELAY_MS;
  if (downSteps == 0)
  {
    downSteps = 1;
  }
  for (uint32_t i = 1; i <= downSteps; i++)
  {
    motionRun(dir, speedPercent - (uint8_t)(((uint32_t)(speedPercent - startSpeed) * i) / downSteps));
    HAL_Delay(RAMP_STEP_DELAY_MS);
  }

  motorBrakeAll();
  HAL_Delay(80);
}
