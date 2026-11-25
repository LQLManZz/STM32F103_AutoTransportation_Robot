#include "agvbench.h"
#include "project_config.h"
#include "agv.h"
#include "debug.h"
#include "destination.h"
#include "imu.h"
#include "motion.h"
#include "motor.h"
#include "scanner.h"
#include "servo.h"
#include "srf05.h"
#include <stdbool.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart1;

// Chờ đủ 1 dòng lệnh kết thúc bằng '\n' (bỏ '\r', cắt bớt nếu quá dài)
static void readLine(char *buf, uint8_t maxLen)
{
  uint8_t len = 0u;
  __HAL_UART_CLEAR_OREFLAG(&huart1);
  while (1)
  {
    uint8_t ch = 0u;
    if (HAL_UART_Receive(&huart1, &ch, 1u, AGV_BENCH_RX_TIMEOUT_MS) != HAL_OK)
      continue;
    if (ch == '\r')
      continue;
    if (ch == '\n')
      break;
    if (len < (uint8_t)(maxLen - 1u))
      buf[len++] = (char)ch;
  }
  buf[len] = '\0';
}

// Bật/tắt ngắt encoder để ISR không làm sai độ rộng xung Echo khi đo
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

// Đo trung vị cặp SRF05 ở góc servo hiện tại
static void measureFront(float *d05Cm, float *d04Cm)
{
  *d05Cm = ECHO_ERROR_TIMEOUT;
  *d04Cm = ECHO_ERROR_TIMEOUT;
  setEncoderIrq(false);
  measureDualDistanceMedianCM(d05Cm, d04Cm);
  setEncoderIrq(true);
}

// Quét toàn cung tìm vật (tắt ngắt encoder trong lúc quét)
static ScanStatus scanOnce(float *distCm, uint8_t *angleDeg)
{
  *distCm = 0.0f;
  *angleDeg = SCAN_HOME_ANGLE;
  setEncoderIrq(false);
  ScanStatus st = scanForObject(distCm, angleDeg);
  setEncoderIrq(true);
  return st;
}

static const char *statusText(ScanStatus st)
{
  if (st == SCAN_OBJECT_FOUND)
    return "OBJ";
  return (st == SCAN_WALL_FOUND) ? "WALL" : "NONE";
}

// Chờ đặt xe xuống sàn, calib IMU khi đứng yên rồi đặt lại tọa độ (0, 0, 0°)
static void prepareFloor(void)
{
  debugState("WAIT", "AGV_TEST");
  HAL_Delay(MOTOR_TEST_START_DELAY_MS);
  if (imuIsReady())
    imuCalibrate();
  agvBenchReset();
}

// H ms (kê cao): đo 90° trước, sau khi motor chạy tiến/lùi lúc servo tắt xung, và sau khi cấp xung lại
static void runHold(int ms)
{
  if (ms < 1)
    return;
  float b05 = 0.0f, b04 = 0.0f, o05 = 0.0f, o04 = 0.0f, r05 = 0.0f, r04 = 0.0f;
  if (imuIsReady())
    imuCalibrate();
  moveServoSmooth(SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);
  servoStop();
  measureFront(&b05, &b04);
  motionDriveTimePid(MOTOR_FORWARD, (uint32_t)ms, PID_TARGET_DRIVE_RPM);
  motionDriveTimePid(MOTOR_BACKWARD, (uint32_t)ms, PID_TARGET_DRIVE_RPM);
  measureFront(&o05, &o04);
  moveServoSmooth(SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);
  measureFront(&r05, &r04);
  debugLog("HOLD ms=%d d_before=%s d_off=%s d_reassert=%s top_before=%s top_off=%s top_reassert=%s\r\n",
           ms, debugF1(b05), debugF1(o05), debugF1(r05), debugF1(b04), debugF1(o04), debugF1(r04));
}

// D cm (dưới sàn): đo 90° trước, chạy PID như agv, rồi đo liên tục 1 s sau phanh để so với encoder
static void runDrive(int cm)
{
  if (cm == 0)
    return;
  prepareFloor();
  moveServoSmooth(SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);
  float b05 = 0.0f, b04 = 0.0f;
  measureFront(&b05, &b04);

  NavPose pose = {0.0f, 0.0f, 0.0f};
  float encCm = destinationExecDrive((int16_t)cm, &pose, false);

  uint32_t t0 = HAL_GetTick();
  setEncoderIrq(false);
  while ((HAL_GetTick() - t0) < AGV_BENCH_TRACE_MS)
  {
    float d05 = ECHO_ERROR_TIMEOUT;
    float d04 = ECHO_ERROR_TIMEOUT;
    uint32_t tMs = HAL_GetTick() - t0;
    measureDualDistanceCM(&d05, &d04);
    debugLog("DRV_T t=%lu d05=%s d04=%s\r\n", tMs, debugF1(d05), debugF1(d04));
  }
  setEncoderIrq(true);
  debugLog("DRV_SUM cmd=%d enc_cm=%s d_before=%s top_before=%s yaw=%s\r\n",
           cm, debugF1(encCm), debugF1(b05), debugF1(b04), debugF1(pose.thetaDeg));
}

// T (dưới sàn): quét -> xoay về vật như agv -> quét tinh đo góc còn lệch
static void runTurn(void)
{
  prepareFloor();
  float distCm = 0.0f;
  uint8_t angleDeg = SCAN_HOME_ANGLE;
  ScanStatus st = scanOnce(&distCm, &angleDeg);
  if (st != SCAN_OBJECT_FOUND)
  {
    debugLog("TURN st=%s\r\n", statusText(st));
    return;
  }

  int16_t cmd = agvTurnCmdToObject(angleDeg, distCm);
  NavPose pose = {0.0f, 0.0f, 0.0f};
  destinationExecSpin(cmd, &pose, false);

  float fineCm = 0.0f;
  uint8_t fineDeg = SCAN_HOME_ANGLE;
  setEncoderIrq(false);
  ScanStatus fineSt = scanFine(&fineCm, &fineDeg);
  setEncoderIrq(true);
  // Quy ước lệnh xoay: > 0 quay phải, heading IMU dương là quay trái
  debugLog("TURN st=OBJ scan_ang=%u scan_d=%s cmd=%d imu_deg=%s fine_st=%s fine_ang=%u fine_d=%s\r\n",
           angleDeg, debugF1(distCm), cmd, debugF1(-pose.thetaDeg), statusText(fineSt), fineDeg, debugF1(fineCm));
}

// G (dưới sàn): quét -> tiếp cận vật như agv (không gắp) -> đo chốt cự ly cuối
static void runApproach(void)
{
  prepareFloor();
  float distCm = 0.0f;
  uint8_t angleDeg = SCAN_HOME_ANGLE;
  ScanStatus st = scanOnce(&distCm, &angleDeg);
  ScanExtraInfo extra;
  scanGetLastExtraInfo(&extra);

  bool ok = false;
  if (st == SCAN_OBJECT_FOUND)
    ok = agvApproachObject(distCm, angleDeg, extra.isWallBump);
  motorStopAll();

  float f05 = 0.0f, f04 = 0.0f;
  measureFront(&f05, &f04);
  NavPose pose;
  NavPose obj;
  agvGetPose(&pose, &obj);
  debugLog("APPROACH st=%s ang=%u d=%s wall=%u ok=%u d05=%s d04=%s pose=(%s,%s,%s) obj=(%s,%s)\r\n",
           statusText(st), angleDeg, debugF1(distCm), extra.isWallBump ? 1u : 0u, ok ? 1u : 0u,
           debugF1(f05), debugF1(f04), debugF1(pose.xCm), debugF1(pose.yCm), debugF1(pose.thetaDeg),
           debugF1(obj.xCm), debugF1(obj.yCm));
}

// N (dưới sàn): chạy đúng 1 lượt trạng thái quét của agvRun (có thể gắp nếu thấy vật gần)
static void runStep(void)
{
  prepareFloor();
  agvRun();
  motorStopAll();
  NavPose pose;
  agvGetPose(&pose, NULL);
  debugLog("STEP pose=(%s,%s,%s)\r\n", debugF1(pose.xCm), debugF1(pose.yCm), debugF1(pose.thetaDeg));
}

void agvBenchRun(void)
{
  char line[AGV_BENCH_LINE_MAX];

  readLine(line, AGV_BENCH_LINE_MAX);
  if (line[0] == '\0')
    return;

  if (line[0] == 'P')
    debugLog("BENCH_READY\r\n");
  else if (line[0] == 'H')
    runHold(atoi(&line[1]));
  else if (line[0] == 'D')
    runDrive(atoi(&line[1]));
  else if (line[0] == 'T')
    runTurn();
  else if (line[0] == 'G')
    runApproach();
  else if (line[0] == 'N')
    runStep();
  else
    debugLog("BENCH_ERR cmd=%s\r\n", line);

  motorStopAll();
  debugLog("BENCH_DONE\r\n");
}
