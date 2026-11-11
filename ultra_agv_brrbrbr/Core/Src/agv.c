#include "agv.h"
#include "project_config.h"
#include "debug.h"
#include "delay.h"
#include "motion.h"
#include "scanner.h"
#include "servo.h"
#include "srf05.h"
#include <stdbool.h>

extern TIM_HandleTypeDef htim2;

typedef struct
{
  float integral;      // Tích phân sai số
  float prevErrorCm;   // Sai số lần trước (cho khâu D)
  bool hasPrev;        // Đã có sai số lần trước chưa
  uint32_t lastTimeMs; // Mốc thời gian lần tính trước (ms)
} DistancePid;

static bool arrived = false; // Đã tới vật, xe đứng yên

// Tính tốc độ (% PWM) từ sai số cự ly (cm), luôn nằm trong PID_SPEED_MIN..PID_SPEED_MAX
static uint8_t pidCompute(DistancePid *pid, float errorCm)
{
  uint32_t now = HAL_GetTick();
  float dtS = (float)(now - pid->lastTimeMs) / 1000.0f;
  float derivative = 0.0f;
  float output;

  if (dtS <= 0.0f || dtS > 0.5f)
  {
    dtS = (float)PID_SAMPLE_TIME_MS / 1000.0f;
  }
  pid->lastTimeMs = now;

  // Chỉ tích phân khi gần vật, có chống bão hòa
  if (errorCm < PID_I_ZONE_CM)
  {
    pid->integral += errorCm * dtS;
    if (pid->integral > PID_I_MAX)
    {
      pid->integral = PID_I_MAX;
    }
  }
  else
  {
    pid->integral = 0.0f;
  }

  if (pid->hasPrev)
  {
    derivative = (errorCm - pid->prevErrorCm) / dtS;
  }
  pid->prevErrorCm = errorCm;
  pid->hasPrev = true;

  output = PID_KP * errorCm + PID_KI * pid->integral + PID_KD * derivative;
  if (output < (float)PID_SPEED_MIN)
  {
    output = (float)PID_SPEED_MIN; // Không để xe dừng giữa đường
  }
  if (output > (float)PID_SPEED_MAX)
  {
    output = (float)PID_SPEED_MAX;
  }
  return (uint8_t)output;
}

// Chạy tới vật theo cự ly siêu âm; true nếu tới nơi, false nếu mất vật hoặc quá thời gian
static bool drivePid(void)
{
  DistancePid pid = {0.0f, 0.0f, false, HAL_GetTick()};
  uint32_t startMs = HAL_GetTick();
  uint8_t lostCount = 0;

  while ((HAL_GetTick() - startMs) < PID_TIMEOUT_MS)
  {
    float d = measureDistanceCM();

    debugDrive(d);
    if (d == -1.0f || d > SCAN_MAX_RANGE_CM) // Mất vật
    {
      lostCount++;
      if (lostCount >= PID_LOST_MAX)
      {
        break;
      }
      HAL_Delay(PID_SAMPLE_TIME_MS);
      continue;
    }
    lostCount = 0;

    // Quá gần (kể cả -2) hoặc đã vào dải dung sai thì dừng
    if (d < SCAN_MIN_RANGE_CM || (d - PID_DIST_TARGET_CM) <= PID_DIST_TOLERANCE_CM)
    {
      motionBrake();
      return true;
    }
    motionForward(pidCompute(&pid, d - PID_DIST_TARGET_CM));
    HAL_Delay(PID_SAMPLE_TIME_MS);
  }
  motionBrake();
  debugDriveLost();
  return false;
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
  float distanceCm; // Cự ly quét được (cm), PID đo lại khi chạy
  uint8_t angleDeg;

  if (arrived)
  {
    return;
  }
  if (scanForObject(&distanceCm, &angleDeg) != SCAN_OBJECT_FOUND)
  {
    return;
  }

  // Xoay hở theo góc: servo 90° là thẳng trước mặt, nhỏ hơn 90° là vật bên phải
  moveServoSmooth(&htim2, SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);
  motionSpinAngle((int16_t)SCAN_HOME_ANGLE - (int16_t)angleDeg, SPEED_SPIN_CALM);

  // Chạy tới vật bằng PID cự ly
  if (drivePid())
  {
    motionDriveDistance(AGV_RAM_EXTRA_CM, AGV_DRIVE_SPEED); // Đẩy nhẹ để chắc chắn chạm vật
    arrived = true;
  }
}
