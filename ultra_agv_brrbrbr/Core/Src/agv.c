#include "agv.h"
#include "project_config.h"
#include "debug.h"
#include "delay.h"
#include "motion.h"
#include "scanner.h"
#include "servo.h"
#include "srf05.h"
#include <math.h>

extern TIM_HandleTypeDef htim2;

// Quay xe về phía vật (x phải, y trước) rồi chạy từng đoạn ngắn tới vật, mỗi đoạn đo lại khoảng cách
static void driveToObject(float xCm, float yCm)
{
  float headingDeg = atan2f(xCm, yCm) * 180.0f / (float)M_PI; // > 0: vật lệch phải

  motionSpinAngle((uint16_t)lroundf(headingDeg), SPEED_SPIN_CALM);
  moveServoSmooth(&htim2, SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);

  for (;;)
  {
    float d = measureDistanceCM();

    debugDrive(d);
    if (d == -1.0f || d > SCAN_MAX_RANGE_CM) // mất vật thì dừng
    {
      debugDriveLost();
      return;
    }
    if (d < SCAN_MIN_RANGE_CM) // quá gần (-2 hoặc dưới ngưỡng): đẩy nốt
    {
      motionDriveDistance(AGV_RAM_EXTRA_CM, AGV_DRIVE_SPEED);
      return;
    }
    if (d <= AGV_STEP_MAX_CM) // đoạn cuối
    {
      motionDriveDistance((int16_t)lroundf(d) + AGV_RAM_EXTRA_CM, AGV_DRIVE_SPEED);
      return;
    }
    motionDriveDistance(AGV_STEP_MAX_CM, AGV_DRIVE_SPEED);
    HAL_Delay(SCAN_PERIOD_MS);
  }
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
  float xCm, yCm;

  if (scanForObject(&xCm, &yCm) == SCAN_OBJECT_FOUND)
  {
    driveToObject(xCm, yCm);
  }
}
