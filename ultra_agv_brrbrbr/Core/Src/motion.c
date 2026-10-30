#include "motion.h"
#include "project_config.h"
#include <stdlib.h>

void motionInit(void)
{
  motorInit();
}

// Cả 2 bánh cùng chiều, cộng trim để xe chạy thẳng
static void motionRun(MotorDir dir, uint8_t speedPercent)
{
  int16_t speedLeft = (int16_t)speedPercent + MOTOR_TRIM_LEFT_OFFSET;
  int16_t speedRight = (int16_t)speedPercent + MOTOR_TRIM_RIGHT_OFFSET;

  if (speedPercent == 0)
  {
    motorStopAll();
    return;
  }
  if (speedLeft > 100)
  {
    speedLeft = 100;
  }
  if (speedRight > 100)
  {
    speedRight = 100;
  }
  motorSetSpeed(MOTOR_LEFT, dir, (uint8_t)speedLeft);
  motorSetSpeed(MOTOR_RIGHT, dir, (uint8_t)speedRight);
}

static void motionSpinLeft(uint8_t speedPercent)
{
  motorSetSpeed(MOTOR_LEFT, MOTOR_BACKWARD, speedPercent);
  motorSetSpeed(MOTOR_RIGHT, MOTOR_FORWARD, speedPercent);
}

static void motionSpinRight(uint8_t speedPercent)
{
  motorSetSpeed(MOTOR_LEFT, MOTOR_FORWARD, speedPercent);
  motorSetSpeed(MOTOR_RIGHT, MOTOR_BACKWARD, speedPercent);
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
  HAL_Delay((uint32_t)((float)abs(angleDeg) * SPIN_MS_PER_DEGREE));

  motorBrakeAll(); // phanh cứng chống trôi
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

  if (totalMs < 500) // quá ngắn: chạy đều, không ramp
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

  // Tăng tốc từ startSpeed lên speedPercent
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

  // Chạy đều
  motionRun(dir, speedPercent);
  HAL_Delay(totalMs - rampUpMs - rampDownMs);

  // Giảm tốc về startSpeed
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
