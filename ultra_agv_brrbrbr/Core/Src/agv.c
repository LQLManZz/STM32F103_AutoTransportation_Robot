#include "agv.h"
#include "blackbox.h"
#include "project_config.h"
#include "debug.h"
#include "delay.h"
#include "encoder.h"
#include "grabber.h"
#include "imu.h"
#include "motion.h"
#include "nav.h"
#include "scanner.h"
#include "servo.h"
#include "srf05.h"
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

// Tạm thời comment thuật toán đầy đủ để test scanner quét thấy vật và tiến tới đơn giản
/*
typedef enum
{
  AGV_STATE_INIT_CALIB = 0,
  AGV_STATE_SCAN,
  AGV_STATE_ARRIVED
} AgvState;

static AgvState agvState = AGV_STATE_INIT_CALIB;
static NavPose robotPose = {0.0f, 0.0f, 0.0f};
static NavPose objectCenter = {0.0f, 0.0f, 0.0f};
static uint8_t corridorSteps = 0u;
static int16_t prevExpandSpin = 0;
static bool justEscapedWall = false;
static uint8_t wallEscapeCount = 0u;

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
  float rad = robotPose.thetaDeg * (MATH_PI / 180.0f);
  robotPose.xCm += sign * actualDistCm * cosf(rad);
  robotPose.yCm += sign * actualDistCm * sinf(rad);
  if (imuIsReady())
    robotPose.thetaDeg = imuGetHeadingDeg();
}

// Tính tọa độ tâm vật (x, y) từ góc servo và khoảng cách SRF05 theo tâm trục 2 bánh xe
static NavPose computeObjectCenter(uint8_t servoAngleDeg, float sensorDistCm)
{
  float thetaRad = robotPose.thetaDeg * (MATH_PI / 180.0f);
  float pivotX = robotPose.xCm + SERVO_PIVOT_OFFSET_CM * cosf(thetaRad);
  float pivotY = robotPose.yCm + SERVO_PIVOT_OFFSET_CM * sinf(thetaRad);

  float rayAngleDeg = robotPose.thetaDeg + ((float)servoAngleDeg - (float)SCAN_HOME_ANGLE);
  float rayRad = rayAngleDeg * (MATH_PI / 180.0f);
  float rayDistCm = SENSOR_FACE_OFFSET_CM + sensorDistCm + OBJ_CENTER_OFFSET_CM;

  NavPose obj;
  obj.xCm = pivotX + rayDistCm * cosf(rayRad);
  obj.yCm = pivotY + rayDistCm * sinf(rayRad);
  obj.thetaDeg = normalizeDeg(rayAngleDeg);
  return obj;
}
*/

// Bật hoặc tắt tạm ngắt encoder để tránh delayUs(20) trong ISR làm sai lệch độ rộng xung Echo khi xe đứng yên
static void setEncoderIrq(bool enable)
{
  if (!enable)
  {
    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
    return;
  }
  __HAL_GPIO_EXTI_CLEAR_IT(ENC_LEFT_PIN | ENC_RIGHT_PIN);
  HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/*
// Đo kiểm tra tường phía trước mũi xe (giữ servo cố định tại HOME 90°)
static bool checkWallAhead(float *outDistCm)
{
  uint8_t wallHits = 0;
  float lastDistCm = ECHO_ERROR_TIMEOUT;

  debugDecision("SCANNING");
  moveServoSmooth(SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);

  setEncoderIrq(false);
  for (uint8_t i = 0; i < WALL_CHECK_SAMPLES; i++)
  {
    float d05RawCm = ECHO_ERROR_TIMEOUT;
    float d04RawCm = ECHO_ERROR_TIMEOUT;
    measureDualDistanceCM(&d05RawCm, &d04RawCm);
    ScanTargetType type = scanClassifyTarget(d05RawCm, d04RawCm);
    bool isWall = (type == TARGET_OBSTACLE) ||
                  (d05RawCm == ECHO_ERROR_TOO_CLOSE) ||
                  (d05RawCm > 0.0f && d05RawCm < SCAN_MIN_RANGE_CM) ||
                  (d05RawCm == ECHO_ERROR_TIMEOUT &&
                   (d04RawCm <= SCAN_CLOSE_WALL_SRF04_CM || d04RawCm < 0.0f));

    if (isWall)
    {
      wallHits++;
      if (d05RawCm >= 0.0f && d05RawCm <= SCAN_MAX_RANGE_CM)
      {
        lastDistCm = d05RawCm;
      }
      else if (d04RawCm >= SCAN_MIN_RANGE_CM && d04RawCm <= SRF04_MAX_RANGE_CM)
      {
        float adj = d04RawCm - SENSOR_DEPTH_OFFSET_CM;
        lastDistCm = (adj > 0.0f) ? adj : 0.0f;
      }
      else
      {
        lastDistCm = d05RawCm;
      }
    }
    else if (lastDistCm < 0.0f && d05RawCm >= 0.0f)
    {
      lastDistCm = d05RawCm;
    }

    debugScan(d05RawCm, d04RawCm, SCAN_HOME_ANGLE, wallHits, isWall ? TARGET_OBSTACLE : type);
  }
  setEncoderIrq(true);
  if (outDistCm) *outDistCm = lastDistCm;
  return wallHits >= SCAN_CONFIRM_COUNT;
}

// Xử lý tường/ngõ cụt: lùi nếu áp sát (< 15 cm), ưu tiên rẽ về phía thoáng rồi mới quay đầu nếu kẹt
static void handleWall(float wallDistCm)
{
  if (wallDistCm < WALL_BACKUP_TRIGGER_CM)
  {
    debugLog("WALL_BACKUP dist=-%dcm wall_d=%s\r\n", WALL_BACKUP_DIST_CM, debugF1(wallDistCm));
    drivePidAndTrackPose(-WALL_BACKUP_DIST_CM);
  }

  bool frontClear = false;
  bool leftWall = false;
  bool rightWall = false;
  scanGetLastCorridorHint(&frontClear, &leftWall, &rightWall);
  int16_t firstTurn = (rightWall && !leftWall) ? (int16_t)WALL_TURN_LEFT_DEG : (int16_t)WALL_TURN_RIGHT_DEG;
  int16_t secondTurn = (firstTurn < 0) ? (int16_t)WALL_TURN_RIGHT_DEG : (int16_t)WALL_TURN_LEFT_DEG;

  debugLog("WALL_CHECK side=%s spin=%d\r\n", (firstTurn < 0) ? "LEFT" : "RIGHT", firstTurn);
  spinAndTrackPose(firstTurn);
  float firstDistCm = ECHO_ERROR_TIMEOUT;
  bool firstHasWall = checkWallAhead(&firstDistCm);
  if (!firstHasWall)
  {
    justEscapedWall = (wallEscapeCount == 1u);
    debugWallTurn(false, (firstTurn < 0) ? firstDistCm : ECHO_ERROR_TIMEOUT,
                  false, (firstTurn > 0) ? firstDistCm : ECHO_ERROR_TIMEOUT, firstTurn);
    HAL_Delay(WALL_HOLD_MS);
    return;
  }

  debugLog("WALL_CHECK side=%s spin=%d\r\n", (secondTurn < 0) ? "LEFT" : "RIGHT", WALL_TURN_BACK_DEG);
  spinAndTrackPose(WALL_TURN_BACK_DEG);
  float secondDistCm = ECHO_ERROR_TIMEOUT;
  bool secondHasWall = checkWallAhead(&secondDistCm);
  if (!secondHasWall)
  {
    debugWallTurn((firstTurn < 0), (firstTurn < 0) ? firstDistCm : secondDistCm,
                  (firstTurn > 0), (firstTurn > 0) ? firstDistCm : secondDistCm, secondTurn);
    HAL_Delay(WALL_HOLD_MS);
    return;
  }

  debugWallTurn(true, (firstTurn < 0) ? firstDistCm : secondDistCm,
                true, (firstTurn > 0) ? firstDistCm : secondDistCm, WALL_TURN_BACK_DEG);
  HAL_Delay(WALL_HOLD_MS);
  debugLog("WALL_CHECK side=BACK spin=%d\r\n", secondTurn);
  spinAndTrackPose(secondTurn);
}

// Cấp tọa độ tâm vật, xoay hướng và chạy tới tọa độ đích bằng PID + 2 encoder rồi dừng hẳn
static bool goToDetectedObject(float distanceCm, uint8_t angleDeg)
{
  objectCenter = computeObjectCenter(angleDeg, distanceCm);
  debugLog("OBJ_COORD raw_ang=%u raw_d=%s -> obj=(%s,%s) cur=(%s,%s,%s)\r\n",
           angleDeg, debugF1(distanceCm),
           debugF1(objectCenter.xCm), debugF1(objectCenter.yCm),
           debugF1(robotPose.xCm), debugF1(robotPose.yCm), debugF1(robotPose.thetaDeg));

  // Xoay tâm trục xe hướng thẳng về tọa độ tâm vật vừa tính
  float dx = objectCenter.xCm - robotPose.xCm;
  float dy = objectCenter.yCm - robotPose.yCm;
  float headingToObj = atan2f(dy, dx) * (180.0f / MATH_PI);
  float turnMathDeg = normalizeDeg(headingToObj - robotPose.thetaDeg);

  moveServoSmooth(SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);
  int16_t coarseCmd = 0;
  int16_t fineSpinDeg = 0;
  if (fabsf(turnMathDeg) > (float)AGV_FINE_DEADBAND_DEG)
  {
    coarseCmd = (int16_t)lroundf(-turnMathDeg);
    debugLog("SPIN_COARSE cmd=%d deg\r\n", coarseCmd);
    spinAndTrackPose(coarseCmd);
    debugLog("AFTER_COARSE pos=(%s,%s,%s)\r\n",
             debugF1(robotPose.xCm), debugF1(robotPose.yCm), debugF1(robotPose.thetaDeg));
  }

  // Nếu sau xoay thô hướng xe vẫn lệch quá AGV_POST_SPIN_MAX_ERR_DEG so với vật thì không chạy bừa, quay lại quét
  float postSpinErr = fabsf(normalizeDeg(headingToObj - robotPose.thetaDeg));
  if (postSpinErr > AGV_POST_SPIN_MAX_ERR_DEG)
  {
    debugLog("ABORT_DRIVE spin_err=%s\r\n", debugF1(postSpinErr));
    return false;
  }

  // Quét tinh để chốt lại chính xác tọa độ tâm vật và cự ly thẳng (chặn bám tường sau xoay thô)
  float fineDist = 0.0f;
  uint8_t fineAngle = SCAN_HOME_ANGLE;
  setEncoderIrq(false);
  ScanStatus fineStatus = scanFine(&fineDist, &fineAngle);
  setEncoderIrq(true);
  bool fineFound = (fineStatus == SCAN_OBJECT_FOUND);
  debugLog("SCAN_FINE found=%d fine_ang=%u fine_d=%s\r\n",
           fineFound ? 1 : 0, fineAngle, debugF1(fineDist));
  if (fineStatus == SCAN_WALL_FOUND)
  {
    debugAlign(coarseCmd, fineAngle, fineSpinDeg, fineDist, false);
    debugLog("ABORT_DRIVE fine_is_wall d=%s\r\n", debugF1(fineDist));
    if (coarseCmd != 0)
      spinAndTrackPose((int16_t)(-coarseCmd));
    else
      handleWall(fineDist);
    return false;
  }
  if (!fineFound)
  {
    debugAlign(coarseCmd, fineAngle, fineSpinDeg, fineDist, fineFound);
    debugLog("ABORT_DRIVE fine_not_found\r\n");
    return false;
  }

  int16_t fineOffset = (int16_t)SCAN_HOME_ANGLE - (int16_t)fineAngle;
  if (abs(fineOffset) > AGV_FINE_DEADBAND_DEG)
  {
    fineSpinDeg = fineOffset;
    debugLog("SPIN_FINE cmd=%d deg\r\n", fineSpinDeg);
    spinAndTrackPose(fineSpinDeg);

    float chk05 = ECHO_ERROR_TIMEOUT;
    float chk04 = ECHO_ERROR_TIMEOUT;
    setEncoderIrq(false);
    measureDualDistanceMedianCM(&chk05, &chk04);
    setEncoderIrq(true);
    if (scanClassifyTarget(chk05, chk04) == TARGET_OBSTACLE)
    {
      debugLog("ABORT_DRIVE pre_drive_wall d05=%s d04=%s\r\n", debugF1(chk05), debugF1(chk04));
      int16_t totalSpin = (int16_t)(coarseCmd + fineSpinDeg);
      if (totalSpin != 0)
        spinAndTrackPose((int16_t)(-totalSpin));
      else
        handleWall(chk05);
      return false;
    }
  }
  debugAlign(coarseCmd, fineAngle, fineSpinDeg, fineDist, fineFound);
  objectCenter = computeObjectCenter(SCAN_HOME_ANGLE, fineDist);

  // Tiến tới tọa độ tâm vật bằng PID + 2 encoder; kiểm tra lại khoảng cách & cập nhật vị trí nếu cần đi tiếp
  float stopOffsetCm = SERVO_PIVOT_OFFSET_CM + SENSOR_FACE_OFFSET_CM + OBJ_CENTER_OFFSET_CM + PID_DIST_TARGET_CM;
  uint32_t driveStartMs = HAL_GetTick();
  float lastD05 = fineDist;
  float lastD04 = ECHO_ERROR_TIMEOUT;
  for (uint8_t step = 0u; step < AGV_APPROACH_MAX_STEPS; step++)
  {
    dx = objectCenter.xCm - robotPose.xCm;
    dy = objectCenter.yCm - robotPose.yCm;
    float distToCenterCm = sqrtf(dx * dx + dy * dy);
    int16_t driveCm = (int16_t)lroundf(distToCenterCm - stopOffsetCm);

    debugLog("OBJ_TARGET step=%u obj=(%s,%s) pos=(%s,%s,%s) dist_center=%s drive=%dcm\r\n",
             step + 1u, debugF1(objectCenter.xCm), debugF1(objectCenter.yCm),
             debugF1(robotPose.xCm), debugF1(robotPose.yCm), debugF1(robotPose.thetaDeg),
             debugF1(distToCenterCm), driveCm);

    if (driveCm <= 0)
    {
      debugDriveEnd(HAL_GetTick() - driveStartMs, lastD05, lastD04, 0u, "DRV_DONE");
      return true;
    }

    drivePidAndTrackPose(driveCm);

    // Đo kiểm tra sau khi dừng: lọc trung vị 3 mẫu tránh 1 phát nhiễu báo mất vật oan
    float d05 = ECHO_ERROR_TIMEOUT;
    float d04 = ECHO_ERROR_TIMEOUT;
    setEncoderIrq(false);
    measureDualDistanceMedianCM(&d05, &d04);
    setEncoderIrq(true);
    float prevD05 = lastD05;
    lastD05 = d05;
    lastD04 = d04;
    debugLog("AFTER_DRIVE pL=%lu pR=%lu pos=(%s,%s,%s) SRF05_bot=%s SRF04_top=%s\r\n",
             encoderGetPulses(MOTOR_LEFT), encoderGetPulses(MOTOR_RIGHT),
             debugF1(robotPose.xCm), debugF1(robotPose.yCm), debugF1(robotPose.thetaDeg),
             debugF1(d05), debugF1(d04));

    bool wallAtEnd = (scanClassifyTarget(d05, d04) == TARGET_OBSTACLE) ||
                     (d05 >= 0.0f && d05 <= (PID_DIST_TARGET_CM + AGV_ARRIVE_TOLERANCE_CM) &&
                      d04 >= 0.0f && d04 <= (PID_DIST_TARGET_CM + AGV_ARRIVE_TOLERANCE_CM +
                                             SENSOR_DEPTH_OFFSET_CM + SCAN_OBSTACLE_TOLERANCE_CM)) ||
                     (d05 == ECHO_ERROR_TIMEOUT &&
                      (d04 <= SCAN_CLOSE_WALL_SRF04_CM || d04 < 0.0f));
    if (wallAtEnd)
    {
      debugDriveEnd(HAL_GetTick() - driveStartMs, d05, d04, PID_LOST_MAX, "DRV_WALL");
      handleWall(d05);
      return false;
    }

    if (d05 == ECHO_ERROR_TOO_CLOSE || (d05 >= 0.0f && d05 <= (PID_DIST_TARGET_CM + AGV_ARRIVE_TOLERANCE_CM)))
    {
      debugDriveEnd(HAL_GetTick() - driveStartMs, d05, d04, 0u, "DRV_DONE");
      return true;
    }

    if (scanClassifyTarget(d05, d04) != TARGET_OBJECT)
    {
      debugDriveEnd(HAL_GetTick() - driveStartMs, d05, d04, PID_LOST_MAX, "DRV_LOST");
      return false;
    }
    if (step == 0u && driveCm >= AGV_DRIVE_MISS_MIN_STEP_CM &&
        d05 > (prevD05 - AGV_DRIVE_MISS_RATIO * (float)driveCm))
    {
      debugDriveEnd(HAL_GetTick() - driveStartMs, d05, d04, PID_LOST_MAX, "DRV_MISSED");
      return false;
    }

    objectCenter = computeObjectCenter(SCAN_HOME_ANGLE, d05);
    lastD05 = d05;
    lastD04 = d04;
  }
  debugDriveEnd(HAL_GetTick() - driveStartMs, lastD05, lastD04, 0u, "DRV_DONE");
  return true;
}

void agvInit(void)
{
  delayInit();
  debugInit();
  imuInit();
  motionInit();
  scanInit();
  agvState = AGV_STATE_INIT_CALIB;
  corridorSteps = 0u;
  prevExpandSpin = 0;
  justEscapedWall = false;
  wallEscapeCount = 0u;
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
    debugState("CALIB", "IMU_CHK");
    debugLog("AGV_INIT imu=%d\r\n", imuIsReady() ? 1 : 0);
    if (imuIsReady() || imuInit())
      imuCalibrate();
    debugLog("AGV_CALIB_DONE imu=%d\r\n", imuIsReady() ? 1 : 0);
    debugState("STOP", imuIsReady() ? "IMU_OK" : "IMU_ERR");
    robotPose.xCm = 0.0f;
    robotPose.yCm = 0.0f;
    robotPose.thetaDeg = 0.0f;
    agvState = AGV_STATE_SCAN;
    break;

  case AGV_STATE_SCAN:
  {
    setEncoderIrq(false);
    ScanStatus status = scanForObject(&distanceCm, &angleDeg);
    setEncoderIrq(true);
    if (status == SCAN_OBJECT_FOUND)
    {
      corridorSteps = 0u;
      prevExpandSpin = 0;
      justEscapedWall = false;
      wallEscapeCount = 0u;
      if (goToDetectedObject(distanceCm, angleDeg))
      {
        debugLog("ARRIVED obj=(%s,%s) final_pos=(%s,%s,%s)\r\n",
                 debugF1(objectCenter.xCm), debugF1(objectCenter.yCm),
                 debugF1(robotPose.xCm), debugF1(robotPose.yCm), debugF1(robotPose.thetaDeg));
        agvState = AGV_STATE_ARRIVED;
      }
    }
    else if (status == SCAN_WALL_FOUND)
    {
      corridorSteps = 0u;
      prevExpandSpin = 0;
      justEscapedWall = false;
      wallEscapeCount++;
      handleWall(distanceCm);
    }
    else
    {
      bool frontClear = false;
      bool leftWall = false;
      bool rightWall = false;
      scanGetLastCorridorHint(&frontClear, &leftWall, &rightWall);
      if (frontClear && leftWall && rightWall && corridorSteps < AGV_MAX_CORRIDOR_STEPS)
      {
        corridorSteps++;
        prevExpandSpin = 0;
        wallEscapeCount = 0u;
        debugLog("SCAN_ADVANCE step=%u dist=%dcm\r\n", corridorSteps, AGV_CORRIDOR_STEP_CM);
        drivePidAndTrackPose(AGV_CORRIDOR_STEP_CM);
      }
      else if (justEscapedWall)
      {
        justEscapedWall = false;
        prevExpandSpin = 0;
        corridorSteps = 0u;
        wallEscapeCount = 0u;
        debugLog("SCAN_FLIP_BACK spin=%d deg\r\n", WALL_TURN_BACK_DEG);
        spinAndTrackPose(WALL_TURN_BACK_DEG);
      }
      else
      {
        wallEscapeCount = 0u;
        int16_t expandSpin = (prevExpandSpin != 0)
                                 ? prevExpandSpin
                                 : ((leftWall && !rightWall)
                                        ? (int16_t)(-AGV_SCAN_EXPAND_SPIN_DEG)
                                        : (int16_t)AGV_SCAN_EXPAND_SPIN_DEG);
        prevExpandSpin = expandSpin;
        debugLog("SCAN_EXPAND spin=%d deg\r\n", expandSpin);
        spinAndTrackPose(expandSpin);
      }
    }
    break;
  }

  case AGV_STATE_ARRIVED:
    HAL_Delay(100u);
    break;
  }
}
*/

static bool agvArrived = false;
static NavPose simplePose = {0.0f, 0.0f, 0.0f};

static float simpleNormalizeDeg(float deg)
{
  while (deg > 180.0f) deg -= 360.0f;
  while (deg <= -180.0f) deg += 360.0f;
  return deg;
}

// Xoay theo góc toán học (+CCW quay trái, -CW quay phải) và cập nhật tọa độ hiện tại
static void simpleSpinAndTrack(float deltaMathDeg)
{
  deltaMathDeg = simpleNormalizeDeg(deltaMathDeg);
  if (fabsf(deltaMathDeg) <= NAV_ANGLE_TOLERANCE_DEG)
    return;

  int16_t spinCmd = (int16_t)lroundf(-deltaMathDeg);
  if (spinCmd == 0)
    return;

  motionSpinAnglePid(spinCmd, PID_TARGET_SPIN_RPM);
  if (imuIsReady())
  {
    simplePose.thetaDeg = imuGetHeadingDeg();
  }
  else
  {
    uint32_t avgPulses = (encoderGetPulses(MOTOR_LEFT) + encoderGetPulses(MOTOR_RIGHT)) / 2u;
    float actualDeg = ((float)avgPulses * 360.0f) / (float)ENC_SPIN_360_PULSES;
    simplePose.thetaDeg = simpleNormalizeDeg(simplePose.thetaDeg + ((spinCmd < 0) ? actualDeg : -actualDeg));
  }
  HAL_Delay(NAV_STEP_PAUSE_MS);
}

// Chạy thẳng (tiến > 0, lùi < 0) và cập nhật tọa độ (x, y, theta) từ encoder + IMU
static void simpleDriveAndTrack(int16_t driveCm)
{
  if (driveCm == 0)
    return;

  float startTheta = simplePose.thetaDeg;
  motionDriveDistancePid(driveCm, PID_TARGET_DRIVE_RPM);

  uint32_t avgPulses = (encoderGetPulses(MOTOR_LEFT) + encoderGetPulses(MOTOR_RIGHT)) / 2u;
  float actualDistCm = ((float)avgPulses * WHEEL_CIRCUMFERENCE_CM) / (float)ENC_PULSES_PER_REV;
  float endTheta = imuIsReady() ? imuGetHeadingDeg() : startTheta;
  float midTheta = simpleNormalizeDeg(startTheta + 0.5f * simpleNormalizeDeg(endTheta - startTheta));
  float midRad = midTheta * (MATH_PI / 180.0f);
  float sign = (driveCm > 0) ? 1.0f : -1.0f;

  simplePose.xCm += sign * actualDistCm * cosf(midRad);
  simplePose.yCm += sign * actualDistCm * sinf(midRad);
  simplePose.thetaDeg = endTheta;
  HAL_Delay(NAV_STEP_PAUSE_MS);
}

void agvInit(void)
{
  delayInit();
  debugInit();
  blackboxInit();
  blackboxDumpToUart();
  blackboxServiceBootCmd(BLACKBOX_BOOT_WAIT_MS);
  imuInit();
  motionInit();
  scanInit();
  agvArrived = false;
  simplePose.xCm = 0.0f;
  simplePose.yCm = 0.0f;
  simplePose.thetaDeg = 0.0f;

  debugState("CALIB", "IMU_CHK");
  if (imuIsReady() || imuInit())
    imuCalibrate();
  debugState("STOP", imuIsReady() ? "IMU_OK" : "IMU_ERR");
}

void agvRun(void)
{
  if (agvArrived)
  {
    HAL_Delay(SCAN_PERIOD_MS);
    return;
  }

  float avgDistCm = 0.0f;
  uint8_t avgAngleDeg = SCAN_HOME_ANGLE;

  // 1. Quét 1 lượt liên tục: gặp tường/trống thì bỏ qua quét tiếp, gặp vật thì quét hết vật rồi tính trung bình
  setEncoderIrq(false);
  ScanStatus status = scanSimpleForObject(&avgDistCm, &avgAngleDeg);
  setEncoderIrq(true);
  if (status != SCAN_OBJECT_FOUND)
    return;

  // Đưa servo quét về thẳng mũi và ngắt PWM trước khi chạy động cơ
  moveServoSmooth(SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);
  servoStop();

  // 2. Tính tọa độ tâm vật (objX, objY) và tọa độ dừng mục tiêu (goalX, goalY) từ điểm trung bình
  float thetaRad = simplePose.thetaDeg * (MATH_PI / 180.0f);
  float pivotX = simplePose.xCm + SERVO_PIVOT_OFFSET_CM * cosf(thetaRad);
  float pivotY = simplePose.yCm + SERVO_PIVOT_OFFSET_CM * sinf(thetaRad);

  float rayAngleDeg = simplePose.thetaDeg + ((float)avgAngleDeg - (float)SCAN_HOME_ANGLE);
  float rayRad = rayAngleDeg * (MATH_PI / 180.0f);
  float rayDistCm = SENSOR_FACE_OFFSET_CM + avgDistCm + OBJ_CENTER_OFFSET_CM;
  float objX = pivotX + rayDistCm * cosf(rayRad);
  float objY = pivotY + rayDistCm * sinf(rayRad);

  float dx = objX - simplePose.xCm;
  float dy = objY - simplePose.yCm;
  float distToObjCm = sqrtf(dx * dx + dy * dy);
  float headingToObj = atan2f(dy, dx) * (180.0f / MATH_PI);
  float stopOffsetCm = SERVO_PIVOT_OFFSET_CM + SENSOR_FACE_OFFSET_CM + OBJ_CENTER_OFFSET_CM + PID_DIST_TARGET_CM;
  float goalDistCm = (distToObjCm > stopOffsetCm) ? (distToObjCm - stopOffsetCm) : 0.0f;
  float headingRad = headingToObj * (MATH_PI / 180.0f);
  float goalX = simplePose.xCm + goalDistCm * cosf(headingRad);
  float goalY = simplePose.yCm + goalDistCm * sinf(headingRad);

  debugLog("SIMPLE_TARGET ang=%u d=%s obj=(%s,%s) goal=(%s,%s,%s)\r\n",
           avgAngleDeg, debugF1(avgDistCm),
           debugF1(objX), debugF1(objY),
           debugF1(goalX), debugF1(goalY), debugF1(headingToObj));

  // 3. Đi thẳng tới tọa độ mục tiêu đã tính (không quét lại lần 2)
  simpleSpinAndTrack(headingToObj - simplePose.thetaDeg);
  if (goalDistCm >= NAV_DIST_TOLERANCE_CM)
    simpleDriveAndTrack((int16_t)lroundf(goalDistCm));

  // 4. Khi tới nơi: so sánh tọa độ hiện tại với tọa độ đã tính và căn chỉnh vị trí + hướng
  float errX = goalX - simplePose.xCm;
  float errY = goalY - simplePose.yCm;
  float errDistCm = sqrtf(errX * errX + errY * errY);

  debugLog("SIMPLE_CHECK cur=(%s,%s,%s) goal=(%s,%s) err_d=%s\r\n",
           debugF1(simplePose.xCm), debugF1(simplePose.yCm), debugF1(simplePose.thetaDeg),
           debugF1(goalX), debugF1(goalY), debugF1(errDistCm));

  if (errDistCm > NAV_DIST_TOLERANCE_CM)
  {
    float fixHeading = atan2f(errY, errX) * (180.0f / MATH_PI);
    float fixTurn = simpleNormalizeDeg(fixHeading - simplePose.thetaDeg);
    if (fabsf(fixTurn) > 90.0f)
    {
      // Nếu xe bị chạy lố qua đích: căn góc rồi lùi nhẹ thay vì quay đầu 180°
      float backTurn = simpleNormalizeDeg(fixHeading + 180.0f - simplePose.thetaDeg);
      simpleSpinAndTrack(backTurn);
      simpleDriveAndTrack(-(int16_t)lroundf(errDistCm));
    }
    else
    {
      simpleSpinAndTrack(fixTurn);
      simpleDriveAndTrack((int16_t)lroundf(errDistCm));
    }
  }

  // Căn chỉnh hướng cuối hướng thẳng về tâm vật đã tính
  float finalHeading = atan2f(objY - simplePose.yCm, objX - simplePose.xCm) * (180.0f / MATH_PI);
  simpleSpinAndTrack(finalHeading - simplePose.thetaDeg);

  debugLog("SIMPLE_ARRIVED cur=(%s,%s,%s) goal=(%s,%s) obj=(%s,%s)\r\n",
           debugF1(simplePose.xCm), debugF1(simplePose.yCm), debugF1(simplePose.thetaDeg),
           debugF1(goalX), debugF1(goalY), debugF1(objX), debugF1(objY));
  debugState("STOP", "ARRIVED");
  agvArrived = true;
}

void agvTestScanGrabInit(void)
{
  delayInit();
  debugInit();
  setEncoderIrq(false);
  scanInit();
  servoStop();
  grabberInit();
  debugGrabber(grabberGetGripAngle(), grabberGetLiftAngle(), "SCAN_GRAB", "WAIT_OBJ");
}

void agvTestScanGrabRun(void)
{
  float objDistCm = 0.0f;
  uint8_t objAngleDeg = SCAN_HOME_ANGLE;

  // 1. Quét tìm vật bằng scanner
  ScanStatus scanSt = scanForObject(&objDistCm, &objAngleDeg);
  if (scanSt != SCAN_OBJECT_FOUND)
  {
    debugGrabber(grabberGetGripAngle(), grabberGetLiftAngle(), "SCAN_GRAB", "WAIT_OBJ");
    return;
  }

  debugLog("TEST_SG_DETECT ang=%u d=%s\r\n", objAngleDeg, debugF1(objDistCm));

  // Đưa đầu quét về thẳng mũi và ngắt PWM servo quét trước khi hạ tay gắp
  moveServoSmooth(SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);
  servoStop();

  // 2. Hạ tay gắp, kẹp và nhấc vật lên
  grabberPick();

  // 3. Bật lại scanner ngay sau khi gắp xong để kiểm tra vật đã thật sự được nhấc lên chưa
  float chkDistCm = 0.0f;
  uint8_t chkAngleDeg = SCAN_HOME_ANGLE;
  ScanStatus pickChkSt = scanForObject(&chkDistCm, &chkAngleDeg);
  moveServoSmooth(SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);
  servoStop();

  bool stillAtOldSpot = (pickChkSt == SCAN_OBJECT_FOUND) &&
                        (chkDistCm <= (objDistCm + GRABBER_VERIFY_DIST_TOL_CM));
  bool pickOk = !stillAtOldSpot;

  debugLog("TEST_SG_PICK result=%s before_d=%s after_st=%d after_ang=%u after_d=%s\r\n",
           pickOk ? "PICK_OK" : "PICK_FAIL",
           debugF1(objDistCm), (int)pickChkSt, chkAngleDeg, debugF1(chkDistCm));
  debugGrabber(grabberGetGripAngle(), grabberGetLiftAngle(),
               "VERIFY_PICK", pickOk ? "PICK_OK" : "PICK_FAIL");
  HAL_Delay(GRABBER_VERIFY_HOLD_MS);

  if (!pickOk)
  {
    // Gắp trượt: mở ngàm, ngắt PWM kẹp rồi quét lại ở chu kỳ sau
    grabberMoveGripSmooth(GRABBER_GRIP_OPEN_DEG);
    HAL_Delay(GRABBER_ACTION_SETTLE_MS);
    grabberGripStop();
    debugGrabber(grabberGetGripAngle(), grabberGetLiftAngle(), "SCAN_GRAB", "WAIT_OBJ");
    HAL_Delay(GRABBER_TEST_WAIT_MS);
    return;
  }

  // Giữ vật trên không trước khi đặt xuống
  HAL_Delay(GRABBER_TEST_HOLD_MS);

  // 4. Hạ tay, mở ngàm đặt vật xuống và nâng cần về vị trí chờ
  grabberRelease();

  // 5. Bật lại scanner ngay sau khi đặt xong để kiểm tra vật có thật sự đã đặt xuống chưa
  float dropDistCm = 0.0f;
  uint8_t dropAngleDeg = SCAN_HOME_ANGLE;
  ScanStatus dropChkSt = scanForObject(&dropDistCm, &dropAngleDeg);
  moveServoSmooth(SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);
  servoStop();

  bool dropOk = (dropChkSt == SCAN_OBJECT_FOUND) &&
                (dropDistCm <= (objDistCm + GRABBER_VERIFY_DIST_TOL_CM));

  debugLog("TEST_SG_DROP result=%s before_d=%s after_st=%d after_ang=%u after_d=%s\r\n",
           dropOk ? "DROP_OK" : "DROP_FAIL",
           debugF1(objDistCm), (int)dropChkSt, dropAngleDeg, debugF1(dropDistCm));
  debugGrabber(grabberGetGripAngle(), grabberGetLiftAngle(),
               "VERIFY_DROP", dropOk ? "DROP_OK" : "DROP_FAIL");
  HAL_Delay(GRABBER_VERIFY_HOLD_MS);

  // Nghỉ ngắn trước chu kỳ test tiếp theo
  HAL_Delay(GRABBER_TEST_WAIT_MS);
}

void agvTestScannerSuiteRun(void)
{
  static bool done = false;
  if (done)
  {
    blackboxPollUartCmd();
    HAL_Delay(20u);
    return;
  }

  static const struct
  {
    const char *name;
    const char *expectStr;
    uint8_t expectMask;
  } cases[] = {
      {"OPEN_SPACE",  "NONE",   (uint8_t)(1u << ERROR_NO_OBJECTIVE)},                                // 1. Trống > 105 cm
      {"WALL_FRONT",  "WALL",   (uint8_t)(1u << SCAN_WALL_FOUND)},                                   // 2. Tường chắn chính diện (25-45 cm)
      {"SIDE_WALL",   "NO_OBJ", (uint8_t)((1u << SCAN_WALL_FOUND) | (1u << ERROR_NO_OBJECTIVE))},    // 3. Tường dọc/gắt >= 60° (bắt trượt gương SRF04)
      {"WALL_CLOSE",  "WALL",   (uint8_t)(1u << SCAN_WALL_FOUND)},                                   // 4. Tường áp sát mũi (8-12 cm)
      {"CORNER_L",    "WALL",   (uint8_t)(1u << SCAN_WALL_FOUND)},                                   // 5. Góc tường chữ L (20-35 cm)
      {"OBJ_CENTER",  "OBJ",    (uint8_t)(1u << SCAN_OBJECT_FOUND)},                                 // 6. Vật 5x5x3 cm chính diện (15-45 cm)
      {"OBJ_SIDE",    "OBJ",    (uint8_t)(1u << SCAN_OBJECT_FOUND)},                                 // 7. Vật lệch rìa cung (55-70° hoặc 110-125°)
      {"OBJ_ON_WALL", "OBJ",    (uint8_t)(1u << SCAN_OBJECT_FOUND)},                                 // 8. Vật dính sát mặt tường rộng (25-40 cm)
      {"TWO_OBJS",    "OBJ",    (uint8_t)(1u << SCAN_OBJECT_FOUND)},                                 // 9. 2 vật lệch góc (chọn vật gần hơn)
  };
  uint8_t total = (uint8_t)(sizeof(cases) / sizeof(cases[0]));
  uint8_t passCount = 0u;
  uint8_t validCases = 0u;
  bool hadOldLog = blackboxHasSavedLog();
  bool hadCompleteLog = blackboxHasCompleteLog();

  blackboxStartCapture();
  setEncoderIrq(false);
  for (uint8_t i = 0u; i < total; i++)
  {
    moveServoSmooth(SCAN_HOME_ANGLE);
    HAL_Delay(SCAN_SETTLE_MS);

    debugLog("SCAN_CASE_PREP id=%u/%u name=%s exp=%s wait=%us\r\n",
             (unsigned)(i + 1u), (unsigned)total, cases[i].name, cases[i].expectStr, (unsigned)SCAN_TEST_WAIT_SEC);
    bool skipWait = false;
    for (uint8_t sec = SCAN_TEST_WAIT_SEC; sec > 0u && !skipWait; sec--)
    {
      debugStepPreview((uint8_t)(i + 1u), total, cases[i].name, sec, true);
      for (uint8_t t = 0u; t < 50u; t++)
      {
        if (blackboxPollUartCmd())
        {
          skipWait = true;
          break;
        }
        HAL_Delay(20u);
      }
    }

    debugLog("SCAN_CASE_START id=%u/%u name=%s\r\n",
             (unsigned)(i + 1u), (unsigned)total, cases[i].name);
    float distCm = 0.0f;
    uint8_t angleDeg = SCAN_HOME_ANGLE;
    ScanStatus st = scanForObject(&distCm, &angleDeg);
    const char *resStr = (st == SCAN_OBJECT_FOUND)
                             ? "OBJ"
                             : ((st == SCAN_WALL_FOUND) ? "WALL" : "NONE");
    bool pass = (cases[i].expectMask & (uint8_t)(1u << (uint8_t)st)) != 0u;
    if (pass)
      passCount++;

    debugLog("SCAN_CASE_END id=%u/%u name=%s res=%s exp=%s pass=%u ang=%u d=%s\r\n",
             (unsigned)(i + 1u), (unsigned)total, cases[i].name,
             resStr, cases[i].expectStr, pass ? 1u : 0u, angleDeg, debugF1(distCm));
    if (!(st == SCAN_WALL_FOUND && distCm == ECHO_ERROR_TOO_CLOSE))
      validCases++;
    if (validCases > 0u && !hadCompleteLog && (!hadOldLog || i >= 2u))
      blackboxFlush();
  }

  moveServoSmooth(SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);
  servoStop();
  setEncoderIrq(true);
  debugState("DONE", "SCAN_TEST");
  debugLog("SCAN_SUITE_DONE pass=%u/%u\r\n", (unsigned)passCount, (unsigned)total);
  if (validCases > 0u && (!hadCompleteLog || validCases >= 3u))
    blackboxFlushComplete();
  blackboxStopCapture();
  done = true;
}
