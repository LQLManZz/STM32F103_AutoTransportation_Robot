#include "agv.h"
#include "project_config.h"
#include "blackbox.h"
#include "debug.h"
#include "delay.h"
#include "encoder.h"
#include "imu.h"
#include "motion.h"
#include "nav.h"
#include "scanner.h"
#include "servo.h"
#include "srf05.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define AGV_PI 3.14159265f

typedef enum
{
  AGV_STATE_INIT_CALIB = 0,
  AGV_STATE_SCAN,
  AGV_STATE_ARRIVED
} AgvState;

static AgvState agvState = AGV_STATE_INIT_CALIB;
static NavPose robotPose = {0.0f, 0.0f, 0.0f};
static NavPose objectCenter = {0.0f, 0.0f, 0.0f};
static bool flushedEmptyScan = false;

static float normalizeDeg(float deg)
{
  while (deg > 180.0f) deg -= 360.0f;
  while (deg <= -180.0f) deg += 360.0f;
  return deg;
}

static void spinAndTrackPose(int16_t spinCmdDeg)
{
  if (spinCmdDeg == 0)
    return;

  motionSpinAnglePid(spinCmdDeg, PID_TARGET_SPIN_RPM);
  if (imuIsReady())
  {
    robotPose.thetaDeg = imuGetHeadingDeg();
  }
  else
  {
    uint32_t avgPulses = (encoderGetPulses(MOTOR_LEFT) + encoderGetPulses(MOTOR_RIGHT)) / 2u;
    float actualDeg = ((float)avgPulses * 360.0f) / (float)ENC_SPIN_360_PULSES;
    robotPose.thetaDeg = normalizeDeg(robotPose.thetaDeg + ((spinCmdDeg < 0) ? actualDeg : -actualDeg));
  }
}

static void drivePidAndTrackPose(int16_t driveCm)
{
  if (driveCm == 0)
    return;

  motionDriveDistancePid(driveCm, PID_TARGET_DRIVE_RPM);

  uint32_t avgPulses = (encoderGetPulses(MOTOR_LEFT) + encoderGetPulses(MOTOR_RIGHT)) / 2u;
  float actualDistCm = ((float)avgPulses * WHEEL_CIRCUMFERENCE_CM) / (float)ENC_PULSES_PER_REV;
  float sign = (driveCm > 0) ? 1.0f : -1.0f;
  float rad = robotPose.thetaDeg * (AGV_PI / 180.0f);
  robotPose.xCm += sign * actualDistCm * cosf(rad);
  robotPose.yCm += sign * actualDistCm * sinf(rad);
  if (imuIsReady())
    robotPose.thetaDeg = imuGetHeadingDeg();
}

// Tính tọa độ tâm vật (x, y) từ góc servo và khoảng cách SRF05 theo tâm trục 2 bánh xe
static NavPose computeObjectCenter(uint8_t servoAngleDeg, float sensorDistCm)
{
  float thetaRad = robotPose.thetaDeg * (AGV_PI / 180.0f);
  float pivotX = robotPose.xCm + SERVO_PIVOT_OFFSET_CM * cosf(thetaRad);
  float pivotY = robotPose.yCm + SERVO_PIVOT_OFFSET_CM * sinf(thetaRad);

  float rayAngleDeg = robotPose.thetaDeg + ((float)servoAngleDeg - (float)SCAN_HOME_ANGLE);
  float rayRad = rayAngleDeg * (AGV_PI / 180.0f);
  float rayDistCm = SENSOR_FACE_OFFSET_CM + sensorDistCm + OBJ_CENTER_OFFSET_CM;

  NavPose obj;
  obj.xCm = pivotX + rayDistCm * cosf(rayRad);
  obj.yCm = pivotY + rayDistCm * sinf(rayRad);
  obj.thetaDeg = normalizeDeg(rayAngleDeg);
  return obj;
}

static void brakeAndStop(void)
{
  motionBrake();
  HAL_Delay(DRIVE_BRAKE_DELAY_MS);
  motionStop();
}

// Tiến chậm tới cách tường 10 cm (chỉ dùng khi gặp tường chắn phía trước)
static bool approachWallDist(float targetDistCm)
{
  uint32_t startMs = HAL_GetTick();
  uint8_t lostCount = 0;
  float d05RawCm = ECHO_ERROR_TIMEOUT;
  float d04RawCm = ECHO_ERROR_TIMEOUT;

  motionForwardHoldStart();
  while ((HAL_GetTick() - startMs) < PID_TIMEOUT_MS)
  {
    uint32_t elapsedMs = HAL_GetTick() - startMs;
    measureDualDistanceCM(&d05RawCm, &d04RawCm);

    if (d05RawCm == ECHO_ERROR_TOO_CLOSE ||
        (d05RawCm >= 0.0f && d05RawCm <= targetDistCm + PID_DIST_TOLERANCE_CM))
    {
      brakeAndStop();
      uint32_t avgPulses = (encoderGetPulses(MOTOR_LEFT) + encoderGetPulses(MOTOR_RIGHT)) / 2u;
      float movedCm = ((float)avgPulses * WHEEL_CIRCUMFERENCE_CM) / (float)ENC_PULSES_PER_REV;
      float rad = robotPose.thetaDeg * (AGV_PI / 180.0f);
      robotPose.xCm += movedCm * cosf(rad);
      robotPose.yCm += movedCm * sinf(rad);
      if (imuIsReady())
        robotPose.thetaDeg = imuGetHeadingDeg();
      debugDriveEnd(elapsedMs, d05RawCm, d04RawCm, lostCount, "WALL_10CM");
      return true;
    }

    if (d05RawCm < SCAN_MIN_RANGE_CM || d05RawCm > SCAN_MAX_RANGE_CM)
    {
      if (++lostCount >= PID_LOST_MAX)
      {
        brakeAndStop();
        debugDriveEnd(elapsedMs, d05RawCm, d04RawCm, lostCount, "WALL_LOST");
        return false;
      }
      HAL_Delay(PID_SAMPLE_TIME_MS);
      continue;
    }
    lostCount = 0;

    motionForwardHoldStep(AGV_APPROACH_SPEED);
    debugApproach(elapsedMs, d05RawCm, d04RawCm, targetDistCm, true);
    HAL_Delay(PID_SAMPLE_TIME_MS);
  }
  brakeAndStop();
  return false;
}

static bool checkWallAtAngle(uint8_t angleDeg, float *outDistCm)
{
  uint8_t wallHits = 0;
  float lastDistCm = ECHO_ERROR_TIMEOUT;

  debugDecision("SCANNING");
  moveServoSmooth(angleDeg);
  HAL_Delay(SCAN_SETTLE_MS);

  for (uint8_t i = 0; i < WALL_CHECK_SAMPLES; i++)
  {
    float d05RawCm = ECHO_ERROR_TIMEOUT;
    float d04RawCm = ECHO_ERROR_TIMEOUT;
    measureDualDistanceCM(&d05RawCm, &d04RawCm);
    ScanTargetType type = scanClassifyTarget(d05RawCm, d04RawCm);
    bool isWall = (type == TARGET_OBSTACLE) ||
                  (d05RawCm < SCAN_MIN_RANGE_CM && d05RawCm != ECHO_ERROR_TIMEOUT);

    if (isWall)
    {
      wallHits++;
      lastDistCm = d05RawCm;
    }
    else if (lastDistCm < 0.0f && d05RawCm >= 0.0f)
    {
      lastDistCm = d05RawCm;
    }

    debugScan(d05RawCm, d04RawCm, angleDeg, wallHits, isWall ? TARGET_OBSTACLE : type);
    HAL_Delay(SCAN_PERIOD_MS);
  }
  if (outDistCm) *outDistCm = lastDistCm;
  return wallHits >= SCAN_CONFIRM_COUNT;
}

static void handleWall(void)
{
  if (!approachWallDist(WALL_STOP_DIST_CM))
    return;

  float leftDistCm = ECHO_ERROR_TIMEOUT;
  float rightDistCm = ECHO_ERROR_TIMEOUT;
  bool leftHasWall = checkWallAtAngle(WALL_CHECK_LEFT_ANGLE, &leftDistCm);
  bool rightHasWall = checkWallAtAngle(WALL_CHECK_RIGHT_ANGLE, &rightDistCm);

  moveServoSmooth(SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);

  int16_t turnDeg = WALL_TURN_BACK_DEG;
  if (!leftHasWall)
    turnDeg = WALL_TURN_LEFT_DEG;
  else if (!rightHasWall)
    turnDeg = WALL_TURN_RIGHT_DEG;

  debugWallTurn(leftHasWall, leftDistCm, rightHasWall, rightDistCm, turnDeg);
  HAL_Delay(WALL_HOLD_MS);
  spinAndTrackPose(turnDeg);
}

// Cấp tọa độ tâm vật, xoay hướng và chạy tới tọa độ đích bằng PID + 2 encoder rồi dừng hẳn
static bool goToDetectedObject(float distanceCm, uint8_t angleDeg)
{
  objectCenter = computeObjectCenter(angleDeg, distanceCm);
  printf("OBJ_COORD raw_ang=%u raw_d=%s -> obj=(%s,%s) cur=(%s,%s,%s)\r\n",
         angleDeg, debugF1(distanceCm),
         debugF1(objectCenter.xCm), debugF1(objectCenter.yCm),
         debugF1(robotPose.xCm), debugF1(robotPose.yCm), debugF1(robotPose.thetaDeg));

  // Xoay tâm trục xe hướng thẳng về tọa độ tâm vật vừa tính
  float dx = objectCenter.xCm - robotPose.xCm;
  float dy = objectCenter.yCm - robotPose.yCm;
  float headingToObj = atan2f(dy, dx) * (180.0f / AGV_PI);
  float turnMathDeg = normalizeDeg(headingToObj - robotPose.thetaDeg);

  moveServoSmooth(SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);
  int16_t coarseCmd = 0;
  int16_t fineSpinDeg = 0;
  if (fabsf(turnMathDeg) > (float)AGV_FINE_DEADBAND_DEG)
  {
    coarseCmd = (int16_t)lroundf(-turnMathDeg);
    printf("SPIN_COARSE cmd=%d deg\r\n", coarseCmd);
    spinAndTrackPose(coarseCmd);
    printf("AFTER_COARSE pos=(%s,%s,%s)\r\n",
           debugF1(robotPose.xCm), debugF1(robotPose.yCm), debugF1(robotPose.thetaDeg));
  }

  // Nếu sau xoay thô hướng xe vẫn lệch quá 15° so với vật thì không chạy bừa, quay lại quét
  float postSpinErr = fabsf(normalizeDeg(headingToObj - robotPose.thetaDeg));
  if (postSpinErr > 15.0f)
  {
    printf("ABORT_DRIVE spin_err=%s\r\n", debugF1(postSpinErr));
    return false;
  }

  // Quét tinh để chốt lại chính xác tọa độ tâm vật và cự ly thẳng
  float fineDist = 0.0f;
  uint8_t fineAngle = SCAN_HOME_ANGLE;
  bool fineFound = (scanFine(&fineDist, &fineAngle) == SCAN_OBJECT_FOUND);
  printf("SCAN_FINE found=%d fine_ang=%u fine_d=%s\r\n",
         fineFound ? 1 : 0, fineAngle, debugF1(fineDist));
  if (!fineFound)
  {
    debugAlign(coarseCmd, fineAngle, fineSpinDeg, fineDist, fineFound);
    printf("ABORT_DRIVE fine_not_found\r\n");
    return false;
  }

  int16_t fineOffset = (int16_t)SCAN_HOME_ANGLE - (int16_t)fineAngle;
  if (abs(fineOffset) > AGV_FINE_DEADBAND_DEG)
  {
    fineSpinDeg = fineOffset;
    printf("SPIN_FINE cmd=%d deg\r\n", fineSpinDeg);
    spinAndTrackPose(fineSpinDeg);
  }
  debugAlign(coarseCmd, fineAngle, fineSpinDeg, fineDist, fineFound);
  objectCenter = computeObjectCenter(SCAN_HOME_ANGLE, fineDist);

  // Tiến tới tọa độ tâm vật bằng PID + 2 encoder; kiểm tra lại khoảng cách & cập nhật vị trí nếu cần đi tiếp
  float stopOffsetCm = SERVO_PIVOT_OFFSET_CM + SENSOR_FACE_OFFSET_CM + OBJ_CENTER_OFFSET_CM + PID_DIST_TARGET_CM;
  uint32_t driveStartMs = HAL_GetTick();
  float lastD05 = fineDist;
  float lastD04 = ECHO_ERROR_TIMEOUT;
  for (uint8_t step = 0u; step < 2u; step++)
  {
    dx = objectCenter.xCm - robotPose.xCm;
    dy = objectCenter.yCm - robotPose.yCm;
    float distToCenterCm = sqrtf(dx * dx + dy * dy);
    int16_t driveCm = (int16_t)lroundf(distToCenterCm - stopOffsetCm);

    printf("OBJ_TARGET step=%u obj=(%s,%s) pos=(%s,%s,%s) dist_center=%s drive=%dcm\r\n",
           step + 1u, debugF1(objectCenter.xCm), debugF1(objectCenter.yCm),
           debugF1(robotPose.xCm), debugF1(robotPose.yCm), debugF1(robotPose.thetaDeg),
           debugF1(distToCenterCm), driveCm);

    if (driveCm <= 0)
    {
      debugDriveEnd(HAL_GetTick() - driveStartMs, lastD05, lastD04, 0u, "DRV_DONE");
      return true;
    }

    drivePidAndTrackPose(driveCm);

    // Đo kiểm tra sau khi dừng: nếu đã sát <= 6 cm thì xong; nếu còn xa thì cập nhật tọa độ đi tiếp
    float d05 = ECHO_ERROR_TIMEOUT;
    float d04 = ECHO_ERROR_TIMEOUT;
    measureDualDistanceCM(&d05, &d04);
    lastD05 = d05;
    lastD04 = d04;
    printf("AFTER_DRIVE pL=%lu pR=%lu pos=(%s,%s,%s) check_d05=%s check_d04=%s\r\n",
           encoderGetPulses(MOTOR_LEFT), encoderGetPulses(MOTOR_RIGHT),
           debugF1(robotPose.xCm), debugF1(robotPose.yCm), debugF1(robotPose.thetaDeg),
           debugF1(d05), debugF1(d04));
    if (d05 == ECHO_ERROR_TOO_CLOSE || (d05 >= 0.0f && d05 <= (PID_DIST_TARGET_CM + 2.5f)))
    {
      debugDriveEnd(HAL_GetTick() - driveStartMs, d05, d04, 0u, "DRV_DONE");
      return true;
    }

    if (scanClassifyTarget(d05, d04) != TARGET_OBJECT)
    {
      debugDriveEnd(HAL_GetTick() - driveStartMs, d05, d04, PID_LOST_MAX, "DRV_LOST");
      return false;
    }

    objectCenter = computeObjectCenter(SCAN_HOME_ANGLE, d05);
  }
  debugDriveEnd(HAL_GetTick() - driveStartMs, lastD05, lastD04, 0u, "DRV_DONE");
  return true;
}

void agvInit(void)
{
  delayInit();
  debugInit();
  blackboxInit();
  imuInit();
  motionInit();
  scanInit();
  agvState = AGV_STATE_INIT_CALIB;
  flushedEmptyScan = false;
  robotPose.xCm = 0.0f;
  robotPose.yCm = 0.0f;
  robotPose.thetaDeg = 0.0f;
  objectCenter = robotPose;
}

void agvRun(void)
{
  float distanceCm = 0.0f;
  uint8_t angleDeg = SCAN_HOME_ANGLE;

  switch (agvState)
  {
  case AGV_STATE_INIT_CALIB:
    blackboxDumpToUart();
    blackboxServiceBootCmd(BLACKBOX_BOOT_WAIT_MS);

    blackboxStartCapture();
    printf("AGV_INIT imu=%d\r\n", imuIsReady() ? 1 : 0);
    if (imuIsReady() || imuInit())
      imuCalibrate();
    printf("AGV_CALIB_DONE imu=%d\r\n", imuIsReady() ? 1 : 0);
    robotPose.xCm = 0.0f;
    robotPose.yCm = 0.0f;
    robotPose.thetaDeg = 0.0f;
    agvState = AGV_STATE_SCAN;
    break;

  case AGV_STATE_SCAN:
  {
    ScanStatus status = scanForObject(&distanceCm, &angleDeg);
    if (status == SCAN_OBJECT_FOUND)
    {
      flushedEmptyScan = false;
      blackboxFlush();
      if (goToDetectedObject(distanceCm, angleDeg))
      {
        printf("ARRIVED obj=(%s,%s) final_pos=(%s,%s,%s)\r\n",
               debugF1(objectCenter.xCm), debugF1(objectCenter.yCm),
               debugF1(robotPose.xCm), debugF1(robotPose.yCm), debugF1(robotPose.thetaDeg));
        blackboxFlush();
        blackboxStopCapture();
        agvState = AGV_STATE_ARRIVED;
      }
      else
      {
        blackboxFlush();
      }
    }
    else if (status == SCAN_WALL_FOUND)
    {
      flushedEmptyScan = false;
      blackboxFlush();
      handleWall();
      blackboxFlush();
    }
    else
    {
      if (!flushedEmptyScan)
      {
        blackboxFlush();
        flushedEmptyScan = true;
      }
    }
    break;
  }

  case AGV_STATE_ARRIVED:
    HAL_Delay(100u);
    break;
  }
}
