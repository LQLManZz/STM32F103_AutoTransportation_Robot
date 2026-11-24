#include "motortest.h"
#include "project_config.h"
#include "debug.h"
#include "encoder.h"
#include "imu.h"
#include "motion.h"
#include "motor.h"
#include "nav.h"
#include "servo.h"
#include <stdio.h>
#include <stdlib.h>

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
  char label[LCD_COLS + 1];
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

static void prepareImuBeforeTest(void)
{
  servoStop();
  debugState("WAIT", "IMU_CHK");
  HAL_Delay(2000u);

  if (imuIsReady() || imuInit())
  {
    debugState("CALIB", "IMU_OK");
    imuCalibrate();
  }
  else
  {
    debugState("WAIT", "IMU_ERR");
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

  prepareImuBeforeTest();

  uint8_t wpCount = (uint8_t)(sizeof(waypoints) / sizeof(waypoints[0]));
  navResetPose();
  navFollowWaypoints(waypoints, wpCount);

  NavPose endPose = navGetPose();
  debugNavWaypoint(wpCount, wpCount, endPose.xCm, endPose.yCm, endPose.thetaDeg,
                   waypoints[wpCount - 1u].xCm, waypoints[wpCount - 1u].yCm, waypoints[wpCount - 1u].thetaDeg,
                   "DONE");
  debugLog("NAV_ALL_DONE end=(%s,%s,%s) imu=%d\r\n",
           debugF1(endPose.xCm), debugF1(endPose.yCm), debugF1(endPose.thetaDeg),
           imuIsReady() ? 1 : 0);
  done = true;
}

typedef struct
{
  const char *label;
  bool isDrive;
  int16_t value; // >0 tiến / quay phải; <0 lùi / quay trái
} PidTestStep;

static void runPidTestSteps(const PidTestStep *steps, uint8_t total, const char *suiteName)
{
  prepareImuBeforeTest();

  uint8_t previewSec = (uint8_t)((NAV_STEP_PREVIEW_MS + 999u) / 1000u);
  for (uint8_t i = 0u; i < total; i++)
  {
    for (uint8_t sec = previewSec; sec > 0u; sec--)
    {
      debugStepPreview((uint8_t)(i + 1u), total, steps[i].label, sec, imuIsReady());
      HAL_Delay(1000u);
    }

    if (steps[i].isDrive)
    {
      motionDriveDistancePid(steps[i].value, PID_TARGET_DRIVE_RPM);
      uint32_t pL = encoderGetPulses(MOTOR_LEFT);
      uint32_t pR = encoderGetPulses(MOTOR_RIGHT);
      float actualCm = ((float)(pL + pR) * 0.5f * WHEEL_CIRCUMFERENCE_CM) / (float)ENC_PULSES_PER_REV;
      if (steps[i].value < 0)
        actualCm = -actualCm;
      float errCm = actualCm - (float)steps[i].value;
      debugLog("PID_DRV_CHK suite=%s step=%u/%u cmd=%dcm act=%scm err=%scm yaw=%s pL=%lu pR=%lu\r\n",
               suiteName, (unsigned)(i + 1u), (unsigned)total, steps[i].value,
               debugF1(actualCm), debugF1(errCm), debugF1(imuGetYawDeg()), pL, pR);
    }
    else
    {
      motionSpinAnglePid(steps[i].value, PID_TARGET_SPIN_RPM);
      uint32_t pL = encoderGetPulses(MOTOR_LEFT);
      uint32_t pR = encoderGetPulses(MOTOR_RIGHT);
      float actualDeg;
      if (imuIsReady())
      {
        // motionSpinAnglePid: >0 quay phải (IMU yaw âm), <0 quay trái (IMU yaw dương)
        actualDeg = -imuGetYawDeg();
      }
      else
      {
        float absDeg = ((float)(pL + pR) * 0.5f * 360.0f) / (float)ENC_SPIN_360_PULSES;
        actualDeg = (steps[i].value > 0) ? absDeg : -absDeg;
      }
      float errDeg = actualDeg - (float)steps[i].value;
      debugLog("PID_SPIN_CHK suite=%s step=%u/%u cmd=%d act=%s err=%s H=%s pL=%lu pR=%lu\r\n",
               suiteName, (unsigned)(i + 1u), (unsigned)total, steps[i].value,
               debugF1(actualDeg), debugF1(errDeg), debugF1(imuGetHeadingDeg()), pL, pR);
    }

    HAL_Delay(NAV_STEP_PAUSE_MS);
  }

  debugState("DONE", imuIsReady() ? "IMU_OK" : "ENC_ONLY");
  debugLog("PID_SUITE_DONE suite=%s endH=%s imu=%d\r\n",
           suiteName, debugF1(imuGetHeadingDeg()), imuIsReady() ? 1 : 0);
}

void motorTestMotionSuiteRun(void)
{
  static const PidTestStep steps[] = {
      {"FWD 100cm", true,   100},
      {"BWD 100cm", true,  -100},
      {"TURN 45L",  false,  -45},
      {"TURN 45R",  false,   45},
      {"TURN 90L",  false,  -90},
      {"TURN 90R",  false,   90},
      {"TURN 180L", false, -180},
      {"TURN 180R", false,  180},
  };
  static bool done = false;
  if (done)
    return;

  runPidTestSteps(steps, (uint8_t)(sizeof(steps) / sizeof(steps[0])), "SUITE");
  done = true;
}

void motorTestPidSpinRun(void)
{
  // Chuỗi góc chuẩn (90, -90, 180, -180) + góc lẻ (tổng đại số = 0° để kiểm tra tích lũy trôi)
  static const PidTestStep steps[] = {
      {"TURN 90R",  false,   90},
      {"TURN 90L",  false,  -90},
      {"TURN 180R", false,  180},
      {"TURN 180L", false, -180},
      {"TURN 15R",  false,   15},
      {"TURN 15L",  false,  -15},
      {"TURN 37R",  false,   37},
      {"TURN 53R",  false,   53},
      {"TURN 125L", false, -125},
      {"TURN 35R",  false,   35},
  };
  static bool done = false;
  if (done)
    return;

  runPidTestSteps(steps, (uint8_t)(sizeof(steps) / sizeof(steps[0])), "SPIN");
  done = true;
}

void motorTestPidDriveRun(void)
{
  static const PidTestStep steps[] = {
      {"FWD 50cm", true,  50},
      {"BWD 50cm", true, -50},
  };
  static bool done = false;
  if (done)
    return;

  runPidTestSteps(steps, (uint8_t)(sizeof(steps) / sizeof(steps[0])), "DRIVE");
  done = true;
}

void motorTestPidSpinAngleRun(int16_t angleDeg)
{
  static bool done = false;
  if (done || angleDeg == 0)
    return;

  char label[16];
  snprintf(label, sizeof(label), "TURN %d%c", abs(angleDeg), (angleDeg > 0) ? 'R' : 'L');
  PidTestStep step = {label, false, angleDeg};
  runPidTestSteps(&step, 1u, "SPIN_ONE");
  done = true;
}
