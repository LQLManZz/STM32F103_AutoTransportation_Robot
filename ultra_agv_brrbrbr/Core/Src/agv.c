#include "agv.h"
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
static uint8_t scannedSectorMask = 0u;
static bool didBackupLastIter = false;

static uint8_t headingToSectorBit(float thetaDeg)
{
  float shifted = destinationNormalizeDeg(thetaDeg) + 180.0f + 0.5f * AGV_EXPAND_SECTOR_DEG;
  while (shifted >= 360.0f) shifted -= 360.0f;
  while (shifted < 0.0f) shifted += 360.0f;
  uint8_t idx = (uint8_t)(shifted / AGV_EXPAND_SECTOR_DEG);
  return (uint8_t)(1u << (idx & (AGV_EXPAND_SECTORS - 1u)));
}

static int16_t chooseUnscannedSpinDeg(float curThetaDeg, bool leftWall, bool rightWall)
{
  int16_t first = (leftWall && !rightWall)
                      ? (int16_t)(-AGV_SCAN_EXPAND_SPIN_DEG)
                      : (int16_t)AGV_SCAN_EXPAND_SPIN_DEG;
  int16_t cands[2] = {first, (int16_t)(-first)};
  for (uint8_t i = 0u; i < 2u; i++)
  {
    float nextTheta = destinationNormalizeDeg(curThetaDeg - (float)cands[i]);
    if ((scannedSectorMask & headingToSectorBit(nextTheta)) == 0u)
      return cands[i];
  }
  return first;
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

// Kiểm tra góc và cự ly có nằm trong vùng tolerance cho phép gắp ngay hay không
static bool isInGrabWindow(float d05, uint8_t angleDeg)
{
  int16_t err = (int16_t)angleDeg - (int16_t)SCAN_HOME_ANGLE;
  if (err < 0)
    err = -err;
  return isInGrabRange(d05) && (err <= (int16_t)AGV_GRAB_ANGLE_TOL_DEG);
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
  destinationExecSpin(spinCmdDeg, &robotPose, true);
}

static void drivePidAndTrackPose(int16_t driveCm)
{
  if (destinationExecDrive(driveCm, &robotPose, true) >= (float)AGV_SECTOR_RESET_DIST_CM)
    scannedSectorMask = 0u;
}

// Chuyển đổi các đoạn tường cục bộ từ lượt quét gần nhất sang tọa độ 2D toàn cục
static void recordScannedWallsToMap(void)
{
  ScanWallSegLocal segs[SCAN_MAX_WALL_SEGS_LOCAL];
  uint8_t n = scanGetLastWallSegments(segs, SCAN_MAX_WALL_SEGS_LOCAL);
  if (n == 0u)
    return;

  float thRad = robotPose.thetaDeg * (MATH_PI / 180.0f);
  float pivX = robotPose.xCm + SERVO_PIVOT_OFFSET_CM * cosf(thRad);
  float pivY = robotPose.yCm + SERVO_PIVOT_OFFSET_CM * sinf(thRad);

  for (uint8_t i = 0u; i < n; i++)
  {
    float r1Rad = (robotPose.thetaDeg + (float)segs[i].startDeg - (float)SCAN_HOME_ANGLE) * (MATH_PI / 180.0f);
    float r2Rad = (robotPose.thetaDeg + (float)segs[i].endDeg - (float)SCAN_HOME_ANGLE) * (MATH_PI / 180.0f);
    float d1 = SENSOR_FACE_OFFSET_CM + segs[i].startDistCm;
    float d2 = SENSOR_FACE_OFFSET_CM + segs[i].endDistCm;
    destinationAddWallSeg(pivX + d1 * cosf(r1Rad), pivY + d1 * sinf(r1Rad),
                          pivX + d2 * cosf(r2Rad), pivY + d2 * sinf(r2Rad),
                          segs[i].startDeg, segs[i].endDeg);
  }
}

// Tính cự ly lùi an toàn không va vào tường đã biết phía sau lưng xe
static int16_t computeSafeBackupCm(void)
{
  float rad = robotPose.thetaDeg * (MATH_PI / 180.0f);
  float cosB = -cosf(rad);
  float sinB = -sinf(rad);
  // Dịch điểm xuất phát 5 cm theo hướng lùi để tường áp sát phía trước/một bên không khóa nhầm đường lùi thoát
  float sx = robotPose.xCm + 5.0f * cosB;
  float sy = robotPose.yCm + 5.0f * sinB;
  for (int16_t d = WALL_BACKUP_DIST_CM; d >= WALL_MIN_SAFE_BACKUP_CM; d -= WALL_BACKUP_STEP_CM)
  {
    float bx = robotPose.xCm + (float)d * cosB;
    float by = robotPose.yCm + (float)d * sinB;
    if (destinationIsPathClear(sx, sy, bx, by, DEST_BODY_RADIUS_CM))
      return d;
  }
  return 0;
}

static void canTurnSides2D(bool *outCanLeft, bool *outCanRight)
{
  float rad = robotPose.thetaDeg * (MATH_PI / 180.0f);
  float c = cosf(rad);
  float s = sinf(rad);
  // Dịch gốc tia kiểm tra 6 cm sang hướng cần rẽ để tường sát bên đối diện không khóa nhầm hướng thoáng
  float lsx = robotPose.xCm - 6.0f * s;
  float lsy = robotPose.yCm + 6.0f * c;
  float lx = robotPose.xCm - AGV_SIDE_CLEAR_CHECK_CM * s;
  float ly = robotPose.yCm + AGV_SIDE_CLEAR_CHECK_CM * c;
  float rsx = robotPose.xCm + 6.0f * s;
  float rsy = robotPose.yCm - 6.0f * c;
  float rx = robotPose.xCm + AGV_SIDE_CLEAR_CHECK_CM * s;
  float ry = robotPose.yCm - AGV_SIDE_CLEAR_CHECK_CM * c;
  *outCanLeft = destinationIsPathClear(lsx, lsy, lx, ly, DEST_BODY_RADIUS_CM);
  *outCanRight = destinationIsPathClear(rsx, rsy, rx, ry, DEST_BODY_RADIUS_CM);
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
  obj.thetaDeg = destinationNormalizeDeg(rayAngleDeg);
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

// Xử lý tường/ngõ cụt: lùi an toàn theo bản đồ 2D, vòng qua đầu mút tường mở hoặc rẽ nhánh thông thoáng
static void handleWall(float wallDistCm)
{
  if (wallDistCm < WALL_BACKUP_TRIGGER_CM)
  {
    int16_t backCm = computeSafeBackupCm();
    if (backCm >= WALL_MIN_SAFE_BACKUP_CM)
    {
      debugLog("WALL_BACKUP dist=-%dcm\r\n", backCm);
      drivePidAndTrackPose((int16_t)(-backCm));
    }
  }

  NavPose bypassPose;
  float lookHeadingDeg = 0.0f;
  if (destinationPlanWallBypass(&robotPose, &bypassPose, &lookHeadingDeg))
  {
    float bx = bypassPose.xCm - robotPose.xCm;
    float by = bypassPose.yCm - robotPose.yCm;
    float bHead = atan2f(by, bx) * (180.0f / MATH_PI);
    float bTurnMath = destinationNormalizeDeg(bHead - robotPose.thetaDeg);
    if (fabsf(bTurnMath) > (float)AGV_FINE_DEADBAND_DEG)
      spinAndTrackPose((int16_t)lroundf(-bTurnMath));

    float bdx = bypassPose.xCm - robotPose.xCm;
    float bdy = bypassPose.yCm - robotPose.yCm;
    int16_t dMoveCm = (int16_t)lroundf(sqrtf(bdx * bdx + bdy * bdy));
    float aheadDist = ECHO_ERROR_TIMEOUT;
    bool wallAhead = checkWallAhead(&aheadDist);
    bool pathBlocked = wallAhead &&
                       (aheadDist == ECHO_ERROR_TOO_CLOSE ||
                        (aheadDist >= 0.0f && aheadDist <= AGV_WALL_CONFIRM_DIST_CM &&
                         aheadDist < (float)(dMoveCm - AGV_BYPASS_BLOCK_MARGIN_CM)));
    if (!pathBlocked && dMoveCm >= DEST_MIN_DRIVE_CM)
    {
      debugLog("WALL_BYPASS d=%dcm\r\n", dMoveCm);
      drivePidAndTrackPose(dMoveCm);
      float lookTurn = destinationNormalizeDeg(lookHeadingDeg - robotPose.thetaDeg);
      if (fabsf(lookTurn) > DEST_BYPASS_LOOK_MIN_DEG)
        spinAndTrackPose((int16_t)lroundf(-lookTurn));
      return;
    }
  }

  bool frontClear = false;
  bool leftWall = false;
  bool rightWall = false;
  scanGetLastCorridorHint(&frontClear, &leftWall, &rightWall);
  bool canLeft2D = false;
  bool canRight2D = false;
  canTurnSides2D(&canLeft2D, &canRight2D);

  int16_t chosenTurn = WALL_TURN_BACK_DEG;
  if (rightWall && !leftWall && canLeft2D)
    chosenTurn = (int16_t)WALL_TURN_LEFT_DEG;
  else if (canRight2D && canLeft2D)
  {
    bool rNew = (scannedSectorMask & headingToSectorBit(robotPose.thetaDeg - (float)WALL_TURN_RIGHT_DEG)) == 0u;
    bool lNew = (scannedSectorMask & headingToSectorBit(robotPose.thetaDeg - (float)WALL_TURN_LEFT_DEG)) == 0u;
    chosenTurn = (!rNew && lNew) ? (int16_t)WALL_TURN_LEFT_DEG : (int16_t)WALL_TURN_RIGHT_DEG;
  }
  else if (canRight2D)
    chosenTurn = (int16_t)WALL_TURN_RIGHT_DEG;
  else if (canLeft2D)
    chosenTurn = (int16_t)WALL_TURN_LEFT_DEG;

  debugLog("WALL_TURN_2D spin=%d\r\n", chosenTurn);
  spinAndTrackPose(chosenTurn);
  debugWallTurn(!canLeft2D, ECHO_ERROR_TIMEOUT, !canRight2D, ECHO_ERROR_TIMEOUT, chosenTurn);
  HAL_Delay(WALL_HOLD_MS);
}

// Chỉ xoay khi chạy thẳng sẽ làm tâm vật lệch ngang quá dung sai ngàm gắp (bỏ xoay góc quá nhỏ ở gần)
static bool needTurnToObj(float errDeg, float distCm)
{
  float absErr = fabsf(errDeg);
  if (absErr >= 90.0f)
    return true;
  if (absErr < (float)AGV_MIN_SPIN_DEG && distCm <= AGV_CLOSE_NO_FINE_DIST_CM)
    return false;
  return distCm * sinf(absErr * (MATH_PI / 180.0f)) > AGV_OBJ_LATERAL_TOL_CM;
}

// Vật quá sát mũi (d05 == -2 hoặc < 2 cm) nhưng cảm biến trên đo xa/bình thường -> chắc chắn là vật cần lùi ra
static bool isTooCloseObj(float d05, float d04)
{
  bool d05TooClose = (d05 == ECHO_ERROR_TOO_CLOSE) || (d05 >= 0.0f && d05 < AGV_GRAB_DIST_MIN_CM);
  if (!d05TooClose)
    return false;
  return (d04 > SRF04_MAX_RANGE_CM) ||
         (d04 >= SCAN_MIN_RANGE_CM &&
          d04 > (2.0f + SENSOR_DEPTH_OFFSET_CM + SCAN_OBSTACLE_TOLERANCE_CM));
}

// Xác nhận 1 mẫu đo nằm trong tầm gắp là vật (tránh gắp khi d05 == -2 hoặc < AGV_GRAB_DIST_MIN_CM)
static bool isGrabSampleObj(float d05, float d04, bool wallBackedObj)
{
  if (!isInGrabRange(d05))
    return false;
  if (scanClassifyTarget(d05, d04) == TARGET_OBJECT)
    return true;
  // Vật sát tường: SRF05 trên phải thấy tường lùi sau mặt vật, nếu cùng mặt phẳng là tường trống
  if (wallBackedObj && d04 >= SCAN_MIN_RANGE_CM && d04 <= SRF04_MAX_RANGE_CM)
    return ((d04 - SENSOR_DEPTH_OFFSET_CM) - d05) >= AGV_WALL_OBJ_MIN_GAP_CM;
  if (d04 == ECHO_ERROR_TIMEOUT)
    return false;
  return (d04 > SRF04_MAX_RANGE_CM) ||
         (d04 >= SCAN_MIN_RANGE_CM && (d04 - SENSOR_DEPTH_OFFSET_CM) > (d05 + SCAN_OBSTACLE_TOLERANCE_CM));
}

// Tính độ chênh giữa cảm biến trên (đã bù lùi 7 cm) và cảm biến dưới để làm căn cứ log vật/tường
static float calcSensorDiffCm(float d05, float d04)
{
  if (d05 < SCAN_MIN_RANGE_CM || d05 > SCAN_MAX_RANGE_CM)
    return 0.0f;
  if (d04 >= SCAN_MIN_RANGE_CM && d04 <= SRF04_MAX_RANGE_CM)
    return (d04 - SENSOR_DEPTH_OFFSET_CM) - d05;
  if (d04 > SRF04_MAX_RANGE_CM)
    return (SRF04_MAX_RANGE_CM - SENSOR_DEPTH_OFFSET_CM) - d05;
  return 0.0f;
}

// Xuất log UART chuẩn kèm tọa độ xe (X,Y,H) và thông số 2 cảm biến chính diện 90°
static void logPoseSensor(const char *tag, float d05, float d04, int flag)
{
  debugLog("%s pose=(%s,%s,%s) d05=%s d04=%s diff=%s f=%d\r\n",
           tag, debugF1(robotPose.xCm), debugF1(robotPose.yCm), debugF1(robotPose.thetaDeg),
           debugF1(d05), debugF1(d04), debugF1(calcSensorDiffCm(d05, d04)), flag);
}

// Kiểm tra vật thẳng chính diện 90° (không xoay servo sau khi đã khóa mục tiêu)
static bool checkFrontGrabObj(bool wallBackedObj, float *outD05, float *outD04)
{
  float d05 = ECHO_ERROR_TIMEOUT;
  float d04 = ECHO_ERROR_TIMEOUT;
  measureDualDistanceMedianCM(&d05, &d04);
  *outD05 = d05;
  *outD04 = d04;
  return isGrabSampleObj(d05, d04, wallBackedObj);
}

// Chốt cự ly ổn định [2.0..4.5 cm] ở góc cố định 90°; nếu quá sát (-2/<2 cm) mà SRF05 trên thoáng thì tự lùi tối đa 2 nhịp
static bool confirmSafeGrabDistance(bool wallBackedObj, float *ioD05, float *ioD04)
{
  for (uint8_t retry = 0u; retry <= 2u; retry++)
  {
    float d05 = ECHO_ERROR_TIMEOUT;
    float d04 = ECHO_ERROR_TIMEOUT;
    setEncoderIrq(false);
    bool inGrab = checkFrontGrabObj(wallBackedObj, &d05, &d04);
    setEncoderIrq(true);
    *ioD05 = d05;
    *ioD04 = d04;
    if (inGrab)
      return true;
    if (isTooCloseObj(d05, d04) && retry < 2u)
    {
      logPoseSensor("GRAB_BACKUP", d05, d04, (int)(retry + 1u));
      drivePidAndTrackPose((int16_t)(-AGV_GRAB_BACKUP_CM));
      continue;
    }
    return false;
  }
  return false;
}

// Khóa servo cố định 90°, xoay thân xe về tọa độ tâm vật và chỉ dùng góc chính diện 90° để kiểm chứng/tiếp cận
static bool goToDetectedObject(float distanceCm, uint8_t angleDeg, bool wallBackedObj)
{
  moveServoSmooth(SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);
  servoStop();

  // Lối tắt: vật đã nằm trong vùng gắp -> kiểm tra chính diện 90° không quá sát (-2) rồi chốt gắp ngay
  if (isInGrabWindow(distanceCm, angleDeg))
  {
    float chk05 = distanceCm, chk04 = ECHO_ERROR_TIMEOUT;
    if (confirmSafeGrabDistance(wallBackedObj, &chk05, &chk04))
    {
      logPoseSensor("GRAB_FAST", chk05, chk04, (int)angleDeg);
      debugDriveEnd(0u, chk05, chk04, 0u, "DRV_DONE");
      return true;
    }
  }

  objectCenter = computeObjectCenter(angleDeg, distanceCm);
  float dx = objectCenter.xCm - robotPose.xCm;
  float dy = objectCenter.yCm - robotPose.yCm;
  float distToObjCm = sqrtf(dx * dx + dy * dy);
  debugLog("OBJ_COORD pose=(%s,%s,%s) obj=(%s,%s) ang=%u d=%s\r\n",
           debugF1(robotPose.xCm), debugF1(robotPose.yCm), debugF1(robotPose.thetaDeg),
           debugF1(objectCenter.xCm), debugF1(objectCenter.yCm), angleDeg, debugF1(distanceCm));

  // Xoay tâm trục xe hướng thẳng về tọa độ tâm vật vừa tính
  float headingToObj = atan2f(dy, dx) * (180.0f / MATH_PI);
  float turnMathDeg = destinationNormalizeDeg(headingToObj - robotPose.thetaDeg);
  int16_t coarseCmd = 0;
  if (needTurnToObj(turnMathDeg, distToObjCm))
  {
    coarseCmd = (int16_t)lroundf(-turnMathDeg);
    spinAndTrackPose(coarseCmd);
  }

  // Nếu sau xoay thô hướng xe vẫn lệch quá AGV_POST_SPIN_MAX_ERR_DEG so với vật thì hủy để quét lại
  float postSpinErr = fabsf(destinationNormalizeDeg(headingToObj - robotPose.thetaDeg));
  if (postSpinErr > AGV_POST_SPIN_MAX_ERR_DEG)
  {
    debugLog("ABORT_DRIVE spin_err=%s\r\n", debugF1(postSpinErr));
    return false;
  }

  // Giữ servo cố định 90° đo xác nhận chính diện sau khi xoay thân xe; thấy tường hoặc lệch hẳn thì trả quyền xoay lại
  float lastD05 = distanceCm;
  float lastD04 = ECHO_ERROR_TIMEOUT;
  if (coarseCmd != 0)
  {
    float f05 = ECHO_ERROR_TIMEOUT;
    float f04 = ECHO_ERROR_TIMEOUT;
    setEncoderIrq(false);
    measureDualDistanceMedianCM(&f05, &f04);
    setEncoderIrq(true);
    ScanTargetType fType = scanClassifyTarget(f05, f04);
    logPoseSensor("FRONT_ALIGN", f05, f04, (int)fType);

    if (!wallBackedObj && isCloseWallSample(f05, f04))
    {
      debugAlign(coarseCmd, SCAN_HOME_ANGLE, 0, f05, false);
      handleWall(f05);
      return false;
    }
    if (!wallBackedObj && fType == TARGET_OBSTACLE)
    {
      debugAlign(coarseCmd, SCAN_HOME_ANGLE, 0, f05, false);
      return false;
    }
    if (isTooCloseObj(f05, f04))
    {
      lastD05 = f05;
      lastD04 = f04;
    }
    else if (fType == TARGET_OBJECT ||
             (wallBackedObj && f05 >= SCAN_MIN_RANGE_CM && f05 <= SCAN_MAX_RANGE_CM))
    {
      lastD05 = f05;
      lastD04 = f04;
      objectCenter = computeObjectCenter(SCAN_HOME_ANGLE, f05);
    }
    else if (distanceCm > AGV_CLOSE_NO_FINE_DIST_CM)
    {
      debugAlign(coarseCmd, SCAN_HOME_ANGLE, 0, f05, false);
      return false;
    }
  }
  debugAlign(coarseCmd, SCAN_HOME_ANGLE, 0, lastD05, true);

  // Tiến tới tọa độ tâm vật bằng PID + 2 encoder; luôn chốt cự ly ở góc cố định 90°
  float stopOffsetCm = SERVO_PIVOT_OFFSET_CM + SENSOR_FACE_OFFSET_CM + OBJ_CENTER_OFFSET_CM + PID_DIST_TARGET_CM;
  uint32_t driveStartMs = HAL_GetTick();
  if (isInGrabRange(lastD05))
  {
    if (confirmSafeGrabDistance(wallBackedObj, &lastD05, &lastD04))
    {
      debugDriveEnd(HAL_GetTick() - driveStartMs, lastD05, lastD04, 0u, "DRV_DONE");
      return true;
    }
    if (!wallBackedObj && isCloseWallSample(lastD05, lastD04))
      return false;
  }

  for (uint8_t step = 0u; step < AGV_APPROACH_MAX_STEPS; step++)
  {
    dx = objectCenter.xCm - robotPose.xCm;
    dy = objectCenter.yCm - robotPose.yCm;
    float distToCenterCm = sqrtf(dx * dx + dy * dy);
    int16_t driveCm = 0;
    bool midStep = false;
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
      // Chỉ dừng đúng 1 lần trước vật (chừa ~12 cm cuối) để chốt cự ly góc 90° rồi tiến chậm vào hạ tay gắp
      if (step == 0u && driveCm > AGV_PRE_GRAB_SPLIT_CM)
      {
        driveCm -= (int16_t)AGV_PRE_GRAB_RESERVE_CM;
        midStep = true;
      }
    }

    debugLog("OBJ_TARGET s=%u pose=(%s,%s,%s) obj=(%s,%s) dist=%s drive=%dcm\r\n",
             step + 1u, debugF1(robotPose.xCm), debugF1(robotPose.yCm), debugF1(robotPose.thetaDeg),
             debugF1(objectCenter.xCm), debugF1(objectCenter.yCm),
             debugF1(distToCenterCm), driveCm);

    drivePidAndTrackPose(driveCm);

    if (midStep)
    {
      float midD05 = ECHO_ERROR_TIMEOUT;
      float midD04 = ECHO_ERROR_TIMEOUT;
      setEncoderIrq(false);
      measureDualDistanceMedianCM(&midD05, &midD04);
      setEncoderIrq(true);
      ScanTargetType midType = scanClassifyTarget(midD05, midD04);
      logPoseSensor("FRONT_MID", midD05, midD04, (int)midType);

      if (isTooCloseObj(midD05, midD04))
      {
        logPoseSensor("DRV_TOO_CLOSE", midD05, midD04, (int)(step + 1u));
        drivePidAndTrackPose((int16_t)(-AGV_GRAB_BACKUP_CM));
        if (confirmSafeGrabDistance(wallBackedObj, &midD05, &midD04))
        {
          debugDriveEnd(HAL_GetTick() - driveStartMs, midD05, midD04, 0u, "DRV_DONE");
          return true;
        }
        lastD05 = midD05;
        lastD04 = midD04;
        continue;
      }
      if (!wallBackedObj && isCloseWallSample(midD05, midD04))
      {
        handleWall(midD05);
        return false;
      }
      bool midIsObj = (midType == TARGET_OBJECT) ||
                      (wallBackedObj && midD05 >= SCAN_MIN_RANGE_CM && midD05 <= SCAN_MAX_RANGE_CM);
      if (!midIsObj || midD05 > (lastD05 - AGV_DRIVE_MISS_RATIO * (float)driveCm))
      {
        debugLog("MID_MISSED prev=%s got=%s type=%u\r\n",
                 debugF1(lastD05), debugF1(midD05), (unsigned)midType);
        return false;
      }
      if (isGrabSampleObj(midD05, midD04, wallBackedObj))
      {
        debugDriveEnd(HAL_GetTick() - driveStartMs, midD05, midD04, 0u, "DRV_DONE");
        return true;
      }
      objectCenter = computeObjectCenter(SCAN_HOME_ANGLE, midD05);
      lastD05 = midD05;
      lastD04 = midD04;
      continue;
    }

    // Đo kiểm tra thẳng chính diện 90° sau khi dừng
    float d05 = ECHO_ERROR_TIMEOUT;
    float d04 = ECHO_ERROR_TIMEOUT;
    setEncoderIrq(false);
    bool inGrab = checkFrontGrabObj(wallBackedObj, &d05, &d04);
    setEncoderIrq(true);
    float prevD05 = lastD05;
    lastD05 = d05;
    lastD04 = d04;
    logPoseSensor("AFTER_DRIVE", d05, d04, inGrab ? 1 : 0);

    if (inGrab)
    {
      debugDriveEnd(HAL_GetTick() - driveStartMs, d05, d04, 0u, "DRV_DONE");
      return true;
    }

    if (isTooCloseObj(d05, d04))
    {
      logPoseSensor("DRV_TOO_CLOSE", d05, d04, (int)(step + 1u));
      drivePidAndTrackPose((int16_t)(-AGV_GRAB_BACKUP_CM));
      if (confirmSafeGrabDistance(wallBackedObj, &d05, &d04))
      {
        debugDriveEnd(HAL_GetTick() - driveStartMs, d05, d04, 0u, "DRV_DONE");
        return true;
      }
      lastD05 = d05;
      lastD04 = d04;
      continue;
    }

    bool blindFault = (d05 == ECHO_ERROR_TIMEOUT && (d04 <= SCAN_CLOSE_WALL_SRF04_CM || d04 < 0.0f));
    if ((!wallBackedObj && isCloseWallSample(d05, d04)) || (wallBackedObj && blindFault))
    {
      debugDriveEnd(HAL_GetTick() - driveStartMs, d05, d04, PID_LOST_MAX, "DRV_WALL");
      handleWall(d05);
      return false;
    }
    if (!wallBackedObj && scanClassifyTarget(d05, d04) == TARGET_OBSTACLE)
    {
      debugDriveEnd(HAL_GetTick() - driveStartMs, d05, d04, PID_LOST_MAX, "DRV_WALL_FAR");
      return false;
    }

    if (scanClassifyTarget(d05, d04) != TARGET_OBJECT &&
        !(wallBackedObj && d05 >= SCAN_MIN_RANGE_CM && d05 <= SCAN_MAX_RANGE_CM))
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
  imuInit();
  motionInit();
  scanInit();
  servoStop();
  grabberInit();
  agvState = AGV_STATE_INIT_CALIB;
  corridorSteps = 0u;
  inspectSteps = 0u;
  scannedSectorMask = 0u;
  didBackupLastIter = false;
  robotPose.xCm = 0.0f;
  robotPose.yCm = 0.0f;
  robotPose.thetaDeg = 0.0f;
  objectCenter = robotPose;
  destinationReset();
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
    scannedSectorMask = 0u;
    didBackupLastIter = false;
    destinationReset();
    agvState = AGV_STATE_SCAN;
    break;

  case AGV_STATE_SCAN:
  {
    setEncoderIrq(false);
    ScanStatus status = scanForObject(&distanceCm, &angleDeg);
    setEncoderIrq(true);
    ScanExtraInfo extra;
    scanGetLastExtraInfo(&extra);
    recordScannedWallsToMap();
    scannedSectorMask |= headingToSectorBit(robotPose.thetaDeg);
    bool frontClear = false;
    bool leftWall = false;
    bool rightWall = false;
    scanGetLastCorridorHint(&frontClear, &leftWall, &rightWall);

    if (status == SCAN_OBJECT_FOUND)
    {
      corridorSteps = 0u;
      didBackupLastIter = false;
      if (goToDetectedObject(distanceCm, angleDeg, extra.isWallBump))
      {
        inspectSteps = 0u;
        // Ngắt hẳn động cơ, servo quét và ngắt ngoài encoder để tay gắp hoạt động độc lập
        motorStopAll();
        moveServoSmooth(SCAN_HOME_ANGLE);
        HAL_Delay(SCAN_SETTLE_MS);
        servoStop();
        // Đo cự ly mặt vật chính diện ngay trước khi gắp làm mốc so sánh
        float pre05 = ECHO_ERROR_TIMEOUT;
        float pre04 = ECHO_ERROR_TIMEOUT;
        setEncoderIrq(false);
        measureDualDistanceMedianCM(&pre05, &pre04);
        // Mẫu lỗi/mất sóng thì kẹp về cửa sổ gắp vì vừa chốt cự ly trong cửa sổ này
        if (pre05 < AGV_GRAB_DIST_MIN_CM)
          pre05 = AGV_GRAB_DIST_MIN_CM;
        if (pre05 > AGV_GRAB_DIST_MAX_CM)
          pre05 = AGV_GRAB_DIST_MAX_CM;
        grabberPick();
        setEncoderIrq(true);

        // Giữ servo cố định 90° đo chính diện để xác nhận vật đã rời khỏi sàn (không xoay scanner)
        // Đã nâng vật thì mặt phía trước phải lùi xa ra (thấy tường sau hoặc khoảng trống); vẫn sát như cũ là gắp hụt/gắp nhầm tường
        // Đo 2 lượt, cả 2 cùng thoáng mới chốt để 1 lượt mất sóng không bị hiểu nhầm là đã gắp
        float chk05 = ECHO_ERROR_TIMEOUT;
        float chk04 = ECHO_ERROR_TIMEOUT;
        bool frontCleared = true;
        for (uint8_t i = 0u; i < 2u && frontCleared; i++)
        {
          setEncoderIrq(false);
          measureDualDistanceMedianCM(&chk05, &chk04);
          setEncoderIrq(true);
          frontCleared = (chk05 >= (pre05 + AGV_PICK_MIN_GAIN_CM));
        }

        bool tooCloseAfterPick = isTooCloseObj(chk05, chk04);
        bool objStillFront = (scanClassifyTarget(chk05, chk04) == TARGET_OBJECT) &&
                             (chk05 <= (AGV_GRAB_DIST_MAX_CM + GRABBER_VERIFY_DIST_TOL_CM));
        bool pickOk = frontCleared && !(tooCloseAfterPick || objStillFront);
        logPoseSensor("AGV_PICK", chk05, chk04, pickOk ? 1 : 0);

        if (pickOk)
        {
          debugLog("PICK_OK steps=%u\r\n", destinationGetCount());
          destinationReturnHome(&robotPose);

          // Cô lập hệ thống và hạ cần nhả vật tại đúng điểm xuất phát
          motorStopAll();
          servoStop();
          setEncoderIrq(false);
          grabberRelease();
          setEncoderIrq(true);

          debugLog("DELIVERED pose=(%s,%s,%s)\r\n",
                   debugF1(robotPose.xCm), debugF1(robotPose.yCm), debugF1(robotPose.thetaDeg));
          debugState("STOP", "DELIVERED");
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
          if (tooCloseAfterPick || (chk05 >= 0.0f && chk05 < AGV_GRAB_DIST_MIN_CM))
          {
            logPoseSensor("PICK_TOO_CLOSE", chk05, chk04, 1);
            drivePidAndTrackPose((int16_t)(-AGV_GRAB_BACKUP_CM));
            setEncoderIrq(false);
            measureDualDistanceMedianCM(&chk05, &chk04);
            setEncoderIrq(true);
            if (isTooCloseObj(chk05, chk04))
              drivePidAndTrackPose((int16_t)(-AGV_GRAB_BACKUP_CM));
          }
        }
      }
    }
    else if (status == SCAN_WALL_FOUND &&
             !(extra.isConfirmedWideWall && frontClear && distanceCm > SCAN_EXPAND_CLEAR_CM))
    {
      corridorSteps = 0u;
      bool canLeft2D = false;
      bool canRight2D = false;
      canTurnSides2D(&canLeft2D, &canRight2D);
      bool allowInspect = (!extra.isConfirmedWideWall &&
                           distanceCm > AGV_WALL_CONFIRM_DIST_CM &&
                           distanceCm <= SCAN_MAX_RANGE_CM) ||
                          (extra.isConfirmedWideWall &&
                           distanceCm > AGV_WALL_CONFIRM_DIST_CM &&
                           distanceCm <= SCAN_EXPAND_CLEAR_CM &&
                           !canLeft2D && !canRight2D);
      if (allowInspect && inspectSteps < AGV_MAX_INSPECT_STEPS)
      {
        // Vật/tường nghi ngờ ở xa hoặc đáy hành lang cụt 2 bên: tiến lại ~20 cm chốt quyết định
        didBackupLastIter = false;
        inspectSteps++;
        int16_t stepCm = computeInspectDriveCm(distanceCm);
        debugLog("WALL_INSPECT d=%dcm\r\n", stepCm);
        drivePidAndTrackPose(stepCm);
      }
      else
      {
        int16_t backCm = (!didBackupLastIter && distanceCm < WALL_BACKUP_TRIGGER_CM)
                             ? computeSafeBackupCm()
                             : 0;
        inspectSteps = 0u;
        if (backCm >= WALL_MIN_SAFE_BACKUP_CM)
        {
          didBackupLastIter = true;
          debugLog("WALL_BACKUP dist=-%dcm\r\n", backCm);
          drivePidAndTrackPose((int16_t)(-backCm));
        }
        else
        {
          didBackupLastIter = false;
          handleWall(WALL_BACKUP_TRIGGER_CM);
        }
      }
    }
    else
    {
      didBackupLastIter = false;
      if (extra.hasSuspect && inspectSteps < AGV_MAX_INSPECT_STEPS)
      {
        inspectSteps++;
        corridorSteps = 0u;
        int16_t suspSpin = (int16_t)SCAN_HOME_ANGLE - (int16_t)extra.suspectAngleDeg;
        if (needTurnToObj((float)suspSpin, SENSOR_FACE_OFFSET_CM + extra.suspectDistCm + OBJ_CENTER_OFFSET_CM))
          spinAndTrackPose(suspSpin);
        int16_t stepCm = computeInspectDriveCm(extra.suspectDistCm);
        debugLog("SUSP_INSPECT a=%u d=%dcm\r\n", extra.suspectAngleDeg, stepCm);
        drivePidAndTrackPose(stepCm);
      }
      else
      {
        inspectSteps = 0u;
        if (frontClear && leftWall && rightWall && corridorSteps < AGV_MAX_CORRIDOR_STEPS)
        {
          corridorSteps++;
          debugLog("SCAN_ADVANCE %u\r\n", corridorSteps);
          drivePidAndTrackPose(AGV_CORRIDOR_STEP_CM);
        }
        else if ((scannedSectorMask & 0xFFu) != 0xFFu)
        {
          int16_t expandSpin = chooseUnscannedSpinDeg(robotPose.thetaDeg, leftWall, rightWall);
          debugLog("SCAN_EXPAND spin=%d\r\n", expandSpin);
          spinAndTrackPose(expandSpin);
        }
        else if (frontClear)
        {
          scannedSectorMask = 0u;
          debugLog("SCAN_ADVANCE open\r\n");
          drivePidAndTrackPose(AGV_CORRIDOR_STEP_CM);
        }
        else
        {
          spinAndTrackPose((leftWall && !rightWall)
                               ? (int16_t)(-AGV_SCAN_EXPAND_SPIN_DEG)
                               : (int16_t)AGV_SCAN_EXPAND_SPIN_DEG);
        }
      }
    }
    break;
  }

  case AGV_STATE_ARRIVED:
    HAL_Delay(20u);
    break;
  }
}

void agvTestScanGrabInit(void)
{
  delayInit();
  debugInit();
  motorStopAll();
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

  setEncoderIrq(false);
  for (uint8_t i = 0u; i < total; i++)
  {
    moveServoSmooth(SCAN_HOME_ANGLE);
    HAL_Delay(SCAN_SETTLE_MS);

    debugLog("SCAN_CASE_PREP id=%u/%u name=%s exp=%s wait=%us\r\n",
             (unsigned)(i + 1u), (unsigned)total, cases[i].name, cases[i].expectStr, (unsigned)SCAN_TEST_WAIT_SEC);
    for (uint8_t sec = SCAN_TEST_WAIT_SEC; sec > 0u; sec--)
    {
      debugStepPreview((uint8_t)(i + 1u), total, cases[i].name, sec, true);
      HAL_Delay(1000u);
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
  }

  moveServoSmooth(SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);
  servoStop();
  setEncoderIrq(true);
  debugState("DONE", "SCAN_TEST");
  debugLog("SCAN_SUITE_DONE pass=%u/%u\r\n", (unsigned)passCount, (unsigned)total);
  done = true;
}
