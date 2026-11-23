#include "agv.h"
#include "blackbox.h"
#include "project_config.h"
#include "debug.h"
#include "delay.h"
#include "destination.h"
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
static uint8_t inspectSteps = 0u;
static uint8_t expandSpinCount = 0u;
static int16_t prevExpandSpin = 0;
static bool justEscapedWall = false;
static uint8_t wallEscapeCount = 0u;
static bool hadCompleteLog = false;

static float normalizeDeg(float deg)
{
  while (deg > 180.0f) deg -= 360.0f;
  while (deg <= -180.0f) deg += 360.0f;
  return deg;
}

// Tính bước tiến để đưa tường/vật nghi ngờ về cự ly kiểm chứng ~20 cm
static int16_t computeInspectDriveCm(float wallDistCm)
{
  int16_t step = (int16_t)lroundf(wallDistCm - AGV_WALL_INSPECT_TARGET_CM);
  if (step < AGV_INSPECT_MIN_STEP_CM)
    return AGV_INSPECT_MIN_STEP_CM;
  if (step > AGV_INSPECT_MAX_STEP_CM)
    return AGV_INSPECT_MAX_STEP_CM;
  return step;
}

// Kiểm tra cự ly trước đầu dò có nằm trong cửa sổ hạ càng gắp trúng vật hay không
static bool isInGrabRange(float d05)
{
  return (d05 >= AGV_GRAB_DIST_MIN_CM) && (d05 <= AGV_GRAB_DIST_MAX_CM);
}

// Xác nhận tường thật gần mũi để không nhầm vật sát tường hoặc vật vượt quá mũi thành tường
static bool isCloseWallSample(float d05, float d04)
{
  if ((scanClassifyTarget(d05, d04) == TARGET_OBSTACLE) && (d05 <= AGV_WALL_CONFIRM_DIST_CM))
    return true;
  if (d05 >= 0.0f && d05 < SCAN_MIN_RANGE_CM && d04 >= 0.0f &&
      fabsf((d04 - SENSOR_DEPTH_OFFSET_CM) - d05) <= SCAN_OBSTACLE_TOLERANCE_CM)
    return true;
  if (d05 == ECHO_ERROR_TOO_CLOSE && d04 >= 0.0f &&
      d04 <= (2.0f + SENSOR_DEPTH_OFFSET_CM + SCAN_OBSTACLE_TOLERANCE_CM))
    return true;
  if (d05 == ECHO_ERROR_TIMEOUT && (d04 <= SCAN_CLOSE_WALL_SRF04_CM || d04 < 0.0f))
    return true;
  return false;
}

static void spinAndTrackPose(int16_t spinCmdDeg)
{
  if (spinCmdDeg == 0)
    return;

  float prevTheta = robotPose.thetaDeg;
  servoStop();
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
  float deltaMathDeg = normalizeDeg(robotPose.thetaDeg - prevTheta);
  destinationRecordSpin((int16_t)lroundf(-deltaMathDeg), prevTheta);
}

static void drivePidAndTrackPose(int16_t driveCm)
{
  if (driveCm == 0)
    return;

  float prevTheta = robotPose.thetaDeg;
  servoStop();
  motionDriveDistancePid(driveCm, PID_TARGET_DRIVE_RPM);

  uint32_t avgPulses = (encoderGetPulses(MOTOR_LEFT) + encoderGetPulses(MOTOR_RIGHT)) / 2u;
  float actualDistCm = ((float)avgPulses * WHEEL_CIRCUMFERENCE_CM) / (float)ENC_PULSES_PER_REV;
  float sign = (driveCm > 0) ? 1.0f : -1.0f;
  float rad = robotPose.thetaDeg * (MATH_PI / 180.0f);
  robotPose.xCm += sign * actualDistCm * cosf(rad);
  robotPose.yCm += sign * actualDistCm * sinf(rad);
  if (imuIsReady())
    robotPose.thetaDeg = imuGetHeadingDeg();
  destinationRecordDrive((int16_t)lroundf(sign * actualDistCm), prevTheta);
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
    bool isWall = (type == TARGET_OBSTACLE && d05RawCm <= AGV_WALL_CONFIRM_DIST_CM) ||
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
  servoStop();
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
    if (fineDist > AGV_WALL_CONFIRM_DIST_CM && fineDist <= SCAN_MAX_RANGE_CM)
    {
      // Tường nghi ngờ ở xa > 24 cm: tiến lại ~20 cm rồi quét tinh lại trước khi kết luận
      int16_t inspCm = computeInspectDriveCm(fineDist);
      debugLog("FINE_INSPECT d=%s drive=%dcm\r\n", debugF1(fineDist), inspCm);
      drivePidAndTrackPose(inspCm);
      setEncoderIrq(false);
      fineStatus = scanFine(&fineDist, &fineAngle);
      setEncoderIrq(true);
      fineFound = (fineStatus == SCAN_OBJECT_FOUND);
      debugLog("SCAN_FINE found=%d fine_ang=%u fine_d=%s\r\n",
               fineFound ? 1 : 0, fineAngle, debugF1(fineDist));
      if (fineStatus == SCAN_WALL_FOUND)
      {
        debugAlign(coarseCmd, fineAngle, fineSpinDeg, fineDist, false);
        debugLog("ABORT_DRIVE fine_is_wall d=%s\r\n", debugF1(fineDist));
        handleWall(fineDist);
        return false;
      }
    }
    else
    {
      debugAlign(coarseCmd, fineAngle, fineSpinDeg, fineDist, false);
      debugLog("ABORT_DRIVE fine_is_wall d=%s\r\n", debugF1(fineDist));
      if (coarseCmd != 0)
        spinAndTrackPose((int16_t)(-coarseCmd));
      else
        handleWall(fineDist);
      return false;
    }
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
      if (chk05 > AGV_WALL_CONFIRM_DIST_CM && chk05 <= SCAN_MAX_RANGE_CM)
      {
        int16_t inspCm = computeInspectDriveCm(chk05);
        debugLog("PRE_DRV_INSPECT d=%s drive=%dcm\r\n", debugF1(chk05), inspCm);
        drivePidAndTrackPose(inspCm);
        setEncoderIrq(false);
        measureDualDistanceMedianCM(&chk05, &chk04);
        setEncoderIrq(true);
        if (scanClassifyTarget(chk05, chk04) == TARGET_OBSTACLE)
        {
          debugLog("ABORT_DRIVE pre_drive_wall d05=%s d04=%s\r\n", debugF1(chk05), debugF1(chk04));
          handleWall(chk05);
          return false;
        }
        if (chk05 < 0.0f || chk05 > SCAN_MAX_RANGE_CM)
        {
          debugLog("ABORT_DRIVE pre_drive_lost d05=%s d04=%s\r\n", debugF1(chk05), debugF1(chk04));
          return false;
        }
        fineDist = chk05;
      }
      else
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
  }
  debugAlign(coarseCmd, fineAngle, fineSpinDeg, fineDist, fineFound);
  objectCenter = computeObjectCenter(SCAN_HOME_ANGLE, fineDist);

  // Tiến tới tọa độ tâm vật bằng PID + 2 encoder; luôn chốt cự ly trước mũi trong tầm hạ càng gắp
  float stopOffsetCm = SERVO_PIVOT_OFFSET_CM + SENSOR_FACE_OFFSET_CM + OBJ_CENTER_OFFSET_CM + PID_DIST_TARGET_CM;
  uint32_t driveStartMs = HAL_GetTick();
  float lastD05 = fineDist;
  float lastD04 = ECHO_ERROR_TIMEOUT;
  if (isInGrabRange(lastD05))
  {
    debugDriveEnd(HAL_GetTick() - driveStartMs, lastD05, lastD04, 0u, "DRV_DONE");
    return true;
  }

  for (uint8_t step = 0u; step < AGV_APPROACH_MAX_STEPS; step++)
  {
    dx = objectCenter.xCm - robotPose.xCm;
    dy = objectCenter.yCm - robotPose.yCm;
    float distToCenterCm = sqrtf(dx * dx + dy * dy);
    int16_t driveCm = 0;
    if (lastD05 == ECHO_ERROR_TOO_CLOSE)
    {
      driveCm = (int16_t)(-AGV_GRAB_BACKUP_CM);
    }
    else if (lastD05 >= 0.0f && lastD05 < AGV_GRAB_DIST_MIN_CM)
    {
      driveCm = (int16_t)lroundf(lastD05 - PID_DIST_TARGET_CM);
      if (driveCm > -2)
        driveCm = -2;
    }
    else
    {
      driveCm = (int16_t)lroundf(distToCenterCm - stopOffsetCm);
      if (lastD05 > AGV_GRAB_DIST_MAX_CM && driveCm < 2)
        driveCm = 2;
    }

    debugLog("OBJ_TARGET step=%u obj=(%s,%s) pos=(%s,%s,%s) dist_center=%s drive=%dcm\r\n",
             step + 1u, debugF1(objectCenter.xCm), debugF1(objectCenter.yCm),
             debugF1(robotPose.xCm), debugF1(robotPose.yCm), debugF1(robotPose.thetaDeg),
             debugF1(distToCenterCm), driveCm);

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

    if (isCloseWallSample(d05, d04))
    {
      debugDriveEnd(HAL_GetTick() - driveStartMs, d05, d04, PID_LOST_MAX, "DRV_WALL");
      handleWall(d05);
      return false;
    }
    if (scanClassifyTarget(d05, d04) == TARGET_OBSTACLE)
    {
      debugDriveEnd(HAL_GetTick() - driveStartMs, d05, d04, PID_LOST_MAX, "DRV_WALL_FAR");
      return false;
    }

    if (isInGrabRange(d05))
    {
      debugDriveEnd(HAL_GetTick() - driveStartMs, d05, d04, 0u, "DRV_DONE");
      return true;
    }

    if (d05 == ECHO_ERROR_TOO_CLOSE || (d05 >= 0.0f && d05 < AGV_GRAB_DIST_MIN_CM))
    {
      debugLog("DRV_TOO_CLOSE step=%u d05=%s -> backup\r\n", step + 1u, debugF1(d05));
      continue;
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
  }
  debugDriveEnd(HAL_GetTick() - driveStartMs, lastD05, lastD04, PID_LOST_MAX, "DRV_RANGE");
  return false;
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
  servoStop();
  grabberInit();
  agvState = AGV_STATE_INIT_CALIB;
  corridorSteps = 0u;
  inspectSteps = 0u;
  expandSpinCount = 0u;
  prevExpandSpin = 0;
  justEscapedWall = false;
  wallEscapeCount = 0u;
  hadCompleteLog = false;
  robotPose.xCm = 0.0f;
  robotPose.yCm = 0.0f;
  robotPose.thetaDeg = 0.0f;
  objectCenter = robotPose;
  destinationReset();
}

void agvRun(void)
{
  blackboxPollUartCmd();

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
    destinationReset();
    hadCompleteLog = blackboxHasCompleteLog();
    blackboxStartCapture();
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
      inspectSteps = 0u;
      expandSpinCount = 0u;
      prevExpandSpin = 0;
      justEscapedWall = false;
      wallEscapeCount = 0u;
      if (goToDetectedObject(distanceCm, angleDeg))
      {
        // Ngắt hẳn động cơ, servo quét và ngắt ngoài encoder để tay gắp hoạt động độc lập
        motorStopAll();
        moveServoSmooth(SCAN_HOME_ANGLE);
        HAL_Delay(SCAN_SETTLE_MS);
        servoStop();
        setEncoderIrq(false);
        grabberPick();
        setEncoderIrq(true);

        // Quét lại ngay trước mũi để xác nhận vật đã thật sự rời khỏi sàn
        float chkDistCm = 0.0f;
        uint8_t chkAngleDeg = SCAN_HOME_ANGLE;
        setEncoderIrq(false);
        ScanStatus pickChkSt = scanForObject(&chkDistCm, &chkAngleDeg);
        setEncoderIrq(true);
        moveServoSmooth(SCAN_HOME_ANGLE);
        HAL_Delay(SCAN_SETTLE_MS);
        servoStop();

        bool stillAtSpot = (pickChkSt == SCAN_OBJECT_FOUND) &&
                           (chkDistCm <= (AGV_GRAB_DIST_MAX_CM + GRABBER_VERIFY_DIST_TOL_CM));
        bool pickOk = !stillAtSpot;
        debugLog("AGV_PICK result=%s chk_st=%d chk_ang=%u chk_d=%s\r\n",
                 pickOk ? "PICK_OK" : "PICK_FAIL",
                 (int)pickChkSt, chkAngleDeg, debugF1(chkDistCm));

        if (pickOk)
        {
          debugLog("PICK_OK_RETURN obj=(%s,%s) pick_pos=(%s,%s,%s) steps=%u\r\n",
                   debugF1(objectCenter.xCm), debugF1(objectCenter.yCm),
                   debugF1(robotPose.xCm), debugF1(robotPose.yCm), debugF1(robotPose.thetaDeg),
                   destinationGetCount());

          destinationReturnHome(&robotPose);

          // Cô lập hệ thống và hạ cần nhả vật tại đúng điểm xuất phát
          motorStopAll();
          servoStop();
          setEncoderIrq(false);
          grabberRelease();
          setEncoderIrq(true);

          debugLog("DELIVERED obj=(%s,%s) final_pos=(%s,%s,%s)\r\n",
                   debugF1(objectCenter.xCm), debugF1(objectCenter.yCm),
                   debugF1(robotPose.xCm), debugF1(robotPose.yCm), debugF1(robotPose.thetaDeg));
          debugState("STOP", "DELIVERED");
          blackboxFlushComplete();
          blackboxStopCapture();
          agvState = AGV_STATE_ARRIVED;
        }
        else
        {
          motorStopAll();
          servoStop();
          setEncoderIrq(false);
          grabberMoveGripSmooth(GRABBER_GRIP_OPEN_DEG);
          HAL_Delay(GRABBER_ACTION_SETTLE_MS);
          grabberStopAll();
          setEncoderIrq(true);
          if (!hadCompleteLog)
            blackboxFlush();
        }
      }
      else if (!hadCompleteLog)
      {
        blackboxFlush();
      }
    }
    else if (status == SCAN_WALL_FOUND)
    {
      corridorSteps = 0u;
      prevExpandSpin = 0;
      expandSpinCount = 0u;
      justEscapedWall = false;
      if (distanceCm > AGV_WALL_CONFIRM_DIST_CM &&
          distanceCm <= SCAN_MAX_RANGE_CM &&
          inspectSteps < AGV_MAX_INSPECT_STEPS)
      {
        // Vật/tường ở xa > 24 cm dễ gây loe tia SRF04: tiến tới ~20 cm rồi quét lại chốt quyết định
        inspectSteps++;
        wallEscapeCount = 0u;
        int16_t stepCm = computeInspectDriveCm(distanceCm);
        debugLog("WALL_INSPECT step=%u d=%s drive=%dcm\r\n", inspectSteps, debugF1(distanceCm), stepCm);
        drivePidAndTrackPose(stepCm);
        if (!hadCompleteLog)
          blackboxFlush();
      }
      else
      {
        inspectSteps = 0u;
        wallEscapeCount++;
        handleWall(distanceCm);
        if (!hadCompleteLog && distanceCm != ECHO_ERROR_TOO_CLOSE)
          blackboxFlush();
      }
    }
    else
    {
      inspectSteps = 0u;
      bool frontClear = false;
      bool leftWall = false;
      bool rightWall = false;
      scanGetLastCorridorHint(&frontClear, &leftWall, &rightWall);
      if (frontClear && leftWall && rightWall && corridorSteps < AGV_MAX_CORRIDOR_STEPS)
      {
        corridorSteps++;
        prevExpandSpin = 0;
        expandSpinCount = 0u;
        wallEscapeCount = 0u;
        debugLog("SCAN_ADVANCE step=%u dist=%dcm\r\n", corridorSteps, AGV_CORRIDOR_STEP_CM);
        drivePidAndTrackPose(AGV_CORRIDOR_STEP_CM);
      }
      else if (justEscapedWall)
      {
        justEscapedWall = false;
        prevExpandSpin = 0;
        corridorSteps = 0u;
        expandSpinCount = 0u;
        wallEscapeCount = 0u;
        debugLog("SCAN_FLIP_BACK spin=%d deg\r\n", WALL_TURN_BACK_DEG);
        spinAndTrackPose(WALL_TURN_BACK_DEG);
      }
      else if (frontClear && expandSpinCount >= AGV_MAX_EXPAND_SPINS)
      {
        expandSpinCount = 0u;
        prevExpandSpin = 0;
        wallEscapeCount = 0u;
        debugLog("SCAN_ADVANCE_OPEN dist=%dcm\r\n", AGV_CORRIDOR_STEP_CM);
        drivePidAndTrackPose(AGV_CORRIDOR_STEP_CM);
      }
      else
      {
        wallEscapeCount = 0u;
        expandSpinCount++;
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
    blackboxPollUartCmd();
    HAL_Delay(20u);
    break;
  }
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

  // Đưa đầu quét về thẳng mũi, ngắt PWM servo quét và động cơ trước khi hạ tay gắp
  motorStopAll();
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
    // Gắp trượt: mở ngàm, ngắt toàn bộ PWM tay gắp rồi quét lại ở chu kỳ sau
    grabberMoveGripSmooth(GRABBER_GRIP_OPEN_DEG);
    HAL_Delay(GRABBER_ACTION_SETTLE_MS);
    grabberStopAll();
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
