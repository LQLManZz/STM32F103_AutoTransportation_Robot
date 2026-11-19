#include "motortest.h"
#include "project_config.h"
#include "debug.h"
#include "imu.h"
#include "motion.h"
#include "motor.h"
#include "nav.h"
#include <math.h>
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

static void prepareImuBeforeTest(const char *testTitle)
{
  debugOledScreen(testTitle, "NEXT: PUT ON FLOOR", "WAIT 2s...", "IMU: CHECKING...");
  HAL_Delay(2000u);

  if (imuIsReady() || imuInit())
  {
    debugOledScreen("IMU: MPU6050 READY", "CALIBRATING GYRO", "KEEP ROBOT STILL!", "WAIT ~1s...");
    imuCalibrate();
  }
  else
  {
    debugOledScreen("IMU: NOT FOUND!", "FALLBACK: ENCODER", "CHECK PB8/PB9", "STARTING IN 2s...");
    HAL_Delay(2000u);
  }
}

void motorTestWaypointsRun(void)
{
  // Kịch bản mẫu hình vuông 40x40 cm: xuất phát (0, 0, 0°) -> đi qua 4 điểm -> về lại gốc (0, 0, 0°)
  static const NavPose waypoints[] = {
      {40.0f,  0.0f,   0.0f},
      {40.0f, 40.0f,  90.0f},
      { 0.0f, 40.0f, 180.0f},
      { 0.0f,  0.0f,   0.0f},
  };
  static bool done = false;
  if (done)
    return;

  prepareImuBeforeTest("TEST: SQUARE 40x40");

  navResetPose();
  navFollowWaypoints(waypoints, (uint8_t)(sizeof(waypoints) / sizeof(waypoints[0])));

  NavPose endPose = navGetPose();
  char finalLine[OLED_LINE_CHARS + 1];
  snprintf(finalLine, sizeof(finalLine), "END:%d,%d %dd",
           (int)lroundf(endPose.xCm), (int)lroundf(endPose.yCm), (int)lroundf(endPose.thetaDeg));
  debugOledScreen("ALL 4 PTS DONE!", finalLine,
                  imuIsReady() ? "MODE: IMU+ENC" : "MODE: ENCODER",
                  "PRESS RST TO REDO");
  printf("NAV_ALL_DONE end=(%.1f,%.1f,%.1f) imu=%d\r\n",
         endPose.xCm, endPose.yCm, endPose.thetaDeg, imuIsReady() ? 1 : 0);
  done = true;
}

void motorTestMotionSuiteRun(void)
{
  static const struct
  {
    const char *label;
    bool isDrive;
    int16_t value; // >0 tiến / quay phải; <0 lùi / quay trái
  } steps[] = {
      {"FWD 100cm", true,   100},
      {"BWD 100cm", true,  -100},
      {"TURN 45L",  false,  -45},
      {"TURN 45R",  false,   45},
      {"TURN 90L",  false,  -90},
      {"TURN 90R",  false,   90},
      {"TURN 180L", false, -180},
      {"TURN 180L", false, -180},
  };
  static bool done = false;
  if (done)
    return;

  prepareImuBeforeTest("TEST: 1m & ANGLES");

  uint8_t total = (uint8_t)(sizeof(steps) / sizeof(steps[0]));
  uint8_t previewSec = (uint8_t)((NAV_STEP_PREVIEW_MS + 999u) / 1000u);

  for (uint8_t i = 0u; i < total; i++)
  {
    for (uint8_t sec = previewSec; sec > 0u; sec--)
    {
      debugStepPreview((uint8_t)(i + 1u), total, steps[i].label, sec, imuIsReady());
      HAL_Delay(1000u);
    }

    char stepHeader[OLED_LINE_CHARS + 1];
    snprintf(stepHeader, sizeof(stepHeader), "STEP %u/%u", (uint8_t)(i + 1u), total);
    debugOledScreen(stepHeader, NULL, NULL, NULL);

    if (steps[i].isDrive)
      motionDriveDistancePid(steps[i].value, PID_TARGET_DRIVE_RPM);
    else
      motionSpinAnglePid(steps[i].value, PID_TARGET_SPIN_RPM);

    HAL_Delay(NAV_STEP_PAUSE_MS);
  }

  debugOledScreen("TEST 8/8 DONE!", "ALL STEPS FINISHED",
                  imuIsReady() ? "MODE: IMU+ENC" : "MODE: ENCODER",
                  "PRESS RST TO REDO");
  printf("TEST_SUITE_DONE imu=%d\r\n", imuIsReady() ? 1 : 0);
  done = true;
}
