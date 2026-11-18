#include "motortest.h"
#include "project_config.h"
#include "debug.h"
#include "motion.h"
#include "motor.h"
#include <stdio.h>

static void startStep(const char *name)
{
  debugMotorTest(name);
  HAL_Delay(MOTOR_TEST_PAUSE_MS);
}

static void setWheelsRaw(MotorDir dirL, uint16_t dutyL, MotorDir dirR, uint16_t dutyR, const char *label)
{
  motorSetDutyRaw(MOTOR_LEFT, dutyL ? dirL : MOTOR_STOP, dutyL);
  motorSetDutyRaw(MOTOR_RIGHT, dutyR ? dirR : MOTOR_STOP, dutyR);
  debugMotorPwmTest(dutyL, motorGetSpeedPercent(MOTOR_LEFT), label);
}

static void setBothRaw(MotorDir dirL, MotorDir dirR, uint16_t duty, const char *label)
{
  setWheelsRaw(dirL, duty, dirR, duty, label);
}

static void rampDownPwm(MotorDir dirL, MotorDir dirR, const char *label)
{
  setBothRaw(MOTOR_STOP, MOTOR_STOP, 0, "WAIT 3S");
  HAL_Delay(MOTOR_TEST_START_DELAY_MS);

  uint16_t duty = PWM_TEST_MAX_DUTY;
  setBothRaw(dirL, dirR, duty, label);
  HAL_Delay(PWM_TEST_HOLD_MAX_MS);

  while (duty > 0)
  {
    duty = (duty > PWM_TEST_STEP_DUTY) ? (duty - PWM_TEST_STEP_DUTY) : 0;
    setBothRaw(dirL, dirR, duty, duty ? label : "ZERO PWM");
    HAL_Delay(PWM_TEST_STEP_MS);
  }
  setBothRaw(MOTOR_STOP, MOTOR_STOP, 0, "FINISHED");
  HAL_Delay(PWM_TEST_CYCLE_PAUSE_MS);
}

void motorTestRun(void)
{
  static const struct
  {
    const char *name;
    bool drive;
    int16_t val;
  } steps[] = {
      {"FORWARD", true, MOTOR_TEST_DRIVE_CM},
      {"BACKWARD", true, -MOTOR_TEST_DRIVE_CM},
      {"RIGHT 90", false, 90},
      {"LEFT 90", false, -90},
      {"RIGHT 180", false, 180},
      {"LEFT 180", false, -180},
  };
  static bool done = false;
  if (done) return;

  debugMotorTest("WAIT");
  HAL_Delay(MOTOR_TEST_START_DELAY_MS);

  for (uint8_t i = 0; i < (uint8_t)(sizeof(steps) / sizeof(steps[0])); i++)
  {
    startStep(steps[i].name);
    if (steps[i].drive)
      motionDriveDistance(steps[i].val, MOTOR_TEST_DRIVE_SPEED);
    else
      motionSpinAngle(steps[i].val, SPEED_SPIN_CALM);
  }

  debugMotorTest("DONE");
  done = true;
}

void motorTestPidRun(void)
{
  debugMotorTest("WAIT");
  HAL_Delay(MOTOR_TEST_START_DELAY_MS);

  startStep("FORWARD");
  motionDriveTimePid(MOTOR_FORWARD, 0u, PID_TARGET_DRIVE_RPM);
}

void motorTestMinPwm(void)
{
  rampDownPwm(MOTOR_FORWARD, MOTOR_FORWARD, "RAMP DOWN");
}

void motorTestMinSpinPwm(void)
{
  static bool cw = true;
  rampDownPwm(cw ? MOTOR_FORWARD : MOTOR_BACKWARD,
              cw ? MOTOR_BACKWARD : MOTOR_FORWARD,
              cw ? "SPIN CW" : "SPIN CCW");
  cw = !cw;
}

void motorTestDirectSpin350(void)
{
  static bool cw = true;
  setBothRaw(MOTOR_STOP, MOTOR_STOP, 0, "WAIT 3S");
  HAL_Delay(MOTOR_TEST_START_DELAY_MS);

  setBothRaw(cw ? MOTOR_FORWARD : MOTOR_BACKWARD,
             cw ? MOTOR_BACKWARD : MOTOR_FORWARD,
             PWM_TEST_SPIN_MIN_DUTY, cw ? "SPIN CW" : "SPIN CCW");
  HAL_Delay(PWM_TEST_CYCLE_PAUSE_MS);

  setBothRaw(MOTOR_STOP, MOTOR_STOP, 0, "STOPPED");
  HAL_Delay(MOTOR_TEST_START_DELAY_MS);
  cw = !cw;
}

static void runContinuousLowest(MotorDir dirL, uint16_t dutyL, MotorDir dirR, uint16_t dutyR, const char *prefix)
{
  char label[OLED_LINE_CHARS + 1];
  if (dutyL == dutyR)
    snprintf(label, sizeof(label), "%s %u", prefix, dutyL);
  else
    snprintf(label, sizeof(label), "%s L%u R%u", prefix, dutyL, dutyR);

  setBothRaw(MOTOR_STOP, MOTOR_STOP, 0, "WAIT 3S");
  HAL_Delay(MOTOR_TEST_START_DELAY_MS);

  // Xung mồi phá ma sát tĩnh trước khi hạ xuống mức PWM duy trì thấp nhất
  motorSetDutyRaw(MOTOR_LEFT, dirL, PWM_TEST_KICK_DUTY);
  motorSetDutyRaw(MOTOR_RIGHT, dirR, PWM_TEST_KICK_DUTY);
  HAL_Delay(PWM_TEST_KICK_MS);

  setWheelsRaw(dirL, dutyL, dirR, dutyR, label);
  while (1)
    HAL_Delay(1000);
}

void motorSpinContinuousLowestSpeed(void)
{
  runContinuousLowest(MOTOR_FORWARD, PWM_TEST_SPIN_MIN_DUTY,
                      MOTOR_BACKWARD, PWM_TEST_SPIN_MIN_DUTY, "SPIN MIN");
}

void motorDriveContinuousLowestSpeed(void)
{
  runContinuousLowest(MOTOR_FORWARD, PWM_TEST_DRIVE_LEFT_DUTY,
                      MOTOR_FORWARD, PWM_TEST_DRIVE_RIGHT_DUTY, "DRIVE");
}
