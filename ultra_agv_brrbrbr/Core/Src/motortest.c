#include "motortest.h"
#include "project_config.h"
#include "debug.h"
#include "motion.h"

static bool done = false;

// Báo tên bước, nghỉ cho xe đứng yên rồi mới làm động tác
static void startStep(const char *name)
{
  debugMotorTest(name);
  HAL_Delay(MOTOR_TEST_PAUSE_MS);
}

void motorTestRun(void)
{
  if (done)
  {
    return;
  }

  debugMotorTest("WAIT");
  HAL_Delay(MOTOR_TEST_START_DELAY_MS);

  startStep("FORWARD");
  motionDriveDistance(MOTOR_TEST_DRIVE_CM, MOTOR_TEST_DRIVE_SPEED);
  startStep("BACKWARD");
  motionDriveDistance(-MOTOR_TEST_DRIVE_CM, MOTOR_TEST_DRIVE_SPEED);
  startStep("RIGHT 90");
  motionSpinAngle(90, SPEED_SPIN_CALM);
  startStep("LEFT 90");
  motionSpinAngle(-90, SPEED_SPIN_CALM);
  startStep("RIGHT 180");
  motionSpinAngle(180, SPEED_SPIN_CALM);
  startStep("LEFT 180");
  motionSpinAngle(-180, SPEED_SPIN_CALM);

  debugMotorTest("DONE");
  done = true;
}
