#include "motorbench.h"
#include "project_config.h"
#include "debug.h"
#include "encoder.h"
#include "imu.h"
#include "motion.h"
#include "motor.h"
#include <stdio.h>
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
    if (HAL_UART_Receive(&huart1, &ch, 1u, MOTOR_BENCH_RX_TIMEOUT_MS) != HAL_OK)
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

// Giới hạn duty trong 0..trần an toàn motor
static uint16_t clampDuty(int duty)
{
  if (duty < 0)
    return 0u;
  if (duty > MOTOR_PWM_MAX_DUTY)
    return MOTOR_PWM_MAX_DUTY;
  return (uint16_t)duty;
}

// Quy đổi số xung trong khoảng thời gian sang vòng/phút
static float pulsesToRpm(uint32_t pulses, uint32_t ms)
{
  if (ms == 0u)
    return 0.0f;
  return ((float)pulses * 60000.0f) / ((float)ENC_PULSES_PER_REV * (float)ms);
}

// D duty ms: chạy lần lượt từng bánh tiến/lùi để kiểm chiều quay và ánh xạ encoder
static void runDir(int duty, int ms)
{
  static const struct
  {
    MotorId id;
    MotorDir dir;
    const char *code;
  } steps[] = {
      {MOTOR_LEFT, MOTOR_FORWARD, "LFWD"},
      {MOTOR_LEFT, MOTOR_BACKWARD, "LBWD"},
      {MOTOR_RIGHT, MOTOR_FORWARD, "RFWD"},
      {MOTOR_RIGHT, MOTOR_BACKWARD, "RBWD"},
  };
  uint16_t d = clampDuty(duty);
  if (ms < 1)
    ms = 1;

  for (uint8_t i = 0u; i < (uint8_t)(sizeof(steps) / sizeof(steps[0])); i++)
  {
    encoderResetPulses();
    motorSetDutyRaw(steps[i].id, steps[i].dir, d);
    debugMotorPwmTest(d, motorGetSpeedPercent(steps[i].id), steps[i].code);
    HAL_Delay((uint32_t)ms);
    motorStopAll();
    HAL_Delay(MOTOR_BENCH_STEP_PAUSE_MS);
    debugLog("DIR wheel=%c dir=%s duty=%u ms=%d pL=%lu pR=%lu\r\n",
             (steps[i].id == MOTOR_LEFT) ? 'L' : 'R',
             (steps[i].dir == MOTOR_FORWARD) ? "FWD" : "BWD", d, ms,
             encoderGetPulses(MOTOR_LEFT), encoderGetPulses(MOTOR_RIGHT));
  }
}

// E wheel pulses duty: chạy 1 bánh tới đủ số xung rồi phanh, đo vọt lố
static void runEnc(char wheel, int pulses, int duty)
{
  MotorId id = (wheel == 'R') ? MOTOR_RIGHT : MOTOR_LEFT;
  uint32_t target = (pulses < 1) ? 1u : (uint32_t)pulses;
  uint16_t d = clampDuty(duty);

  encoderResetPulses();
  debugMotorPwmTest(d, 0u, (id == MOTOR_LEFT) ? "ENCL" : "ENCR");
  uint32_t startMs = HAL_GetTick();
  motorSetDutyRaw(id, MOTOR_FORWARD, d);
  while (encoderGetPulses(id) < target && (HAL_GetTick() - startMs) < MOTOR_BENCH_ENC_TIMEOUT_MS)
  {
  }
  uint32_t runMs = HAL_GetTick() - startMs;
  uint32_t stopPulses = encoderGetPulses(id);
  motorBrakeAll();
  HAL_Delay(MOTOR_BENCH_BRAKE_SETTLE_MS);
  motorStopAll();

  debugLog("ENC wheel=%c duty=%u target=%lu stop=%lu final=%lu other=%lu ms=%lu\r\n",
           (id == MOTOR_LEFT) ? 'L' : 'R', d, target, stopPulses, encoderGetPulses(id),
           encoderGetPulses((id == MOTOR_LEFT) ? MOTOR_RIGHT : MOTOR_LEFT), runMs);
}

// K dir start end step holdMs: quét duty cả 2 bánh, so RPM đếm xung với RPM của ISR
static void runCurve(char dirCh, int startDuty, int endDuty, int stepDuty, int holdMs)
{
  MotorDir dir = (dirCh == 'B') ? MOTOR_BACKWARD : MOTOR_FORWARD;
  if (stepDuty < 1)
    stepDuty = 1;
  if (holdMs < (int)MOTOR_BENCH_RPM_SAMPLE_MS)
    holdMs = (int)MOTOR_BENCH_RPM_SAMPLE_MS;
  int stepSigned = (startDuty <= endDuty) ? stepDuty : -stepDuty;

  for (int duty = startDuty; (stepSigned > 0) ? (duty <= endDuty) : (duty >= endDuty); duty += stepSigned)
  {
    uint16_t d = clampDuty(duty);
    motorSetDutyRaw(MOTOR_LEFT, dir, d);
    motorSetDutyRaw(MOTOR_RIGHT, dir, d);
    debugMotorPwmTest(d, motorGetSpeedPercent(MOTOR_LEFT), (dir == MOTOR_FORWARD) ? "KFWD" : "KBWD");
    HAL_Delay(MOTOR_BENCH_SETTLE_MS);

    uint32_t p0L = encoderGetPulses(MOTOR_LEFT);
    uint32_t p0R = encoderGetPulses(MOTOR_RIGHT);
    uint32_t sumL = 0u, sumR = 0u, n = 0u;
    uint32_t t0 = HAL_GetTick();
    uint32_t lastMs = t0;
    while ((HAL_GetTick() - t0) < (uint32_t)holdMs)
    {
      uint32_t nowMs = HAL_GetTick();
      if ((nowMs - lastMs) >= MOTOR_BENCH_RPM_SAMPLE_MS)
      {
        lastMs = nowMs;
        sumL += encoderGetRpm(MOTOR_LEFT);
        sumR += encoderGetRpm(MOTOR_RIGHT);
        n++;
      }
    }
    uint32_t dtMs = HAL_GetTick() - t0;
    uint32_t dpL = encoderGetPulses(MOTOR_LEFT) - p0L;
    uint32_t dpR = encoderGetPulses(MOTOR_RIGHT) - p0R;

    debugLog("CURVE dir=%s duty=%u ms=%lu dpL=%lu dpR=%lu cntL=%s cntR=%s isrL=%s isrR=%s\r\n",
             (dir == MOTOR_FORWARD) ? "FWD" : "BWD", d, dtMs, dpL, dpR,
             debugF1(pulsesToRpm(dpL, dtMs)), debugF1(pulsesToRpm(dpR, dtMs)),
             debugF1(n ? (float)sumL / (float)n : 0.0f), debugF1(n ? (float)sumR / (float)n : 0.0f));
  }
  motorStopAll();
}

// V dir rpm ms: giữ tốc độ PID theo thời gian (log TEST mỗi 100 ms do motion in sẵn)
static void runVelocity(char dirCh, int rpm, int ms)
{
  if (rpm < 1 || ms < 1)
    return;
  // Calib bias gyro khi xe đứng yên, tránh yaw trôi làm vòng giữ hướng lái lệch 2 bánh
  if (imuIsReady())
    imuCalibrate();
  motionDriveTimePid((dirCh == 'B') ? MOTOR_BACKWARD : MOTOR_FORWARD, (uint32_t)ms, (uint16_t)rpm);
}

// Chờ đặt xe xuống sàn rồi calib IMU khi xe đứng yên
static void prepareFloor(void)
{
  debugState("WAIT", "MTR_TEST");
  HAL_Delay(MOTOR_TEST_START_DELAY_MS);
  if (imuIsReady())
    imuCalibrate();
}

// M cm rpm: chạy thẳng PID dưới sàn, báo quãng đường theo encoder để so với thước
static void runMove(int cm, int rpm)
{
  if (cm == 0 || rpm < 1)
    return;
  prepareFloor();
  motionDriveDistancePid((int16_t)cm, (uint16_t)rpm);
  HAL_Delay(NAV_STEP_PAUSE_MS);

  uint32_t pL = encoderGetPulses(MOTOR_LEFT);
  uint32_t pR = encoderGetPulses(MOTOR_RIGHT);
  float encCm = ((float)(pL + pR) * 0.5f * WHEEL_CIRCUMFERENCE_CM) / (float)ENC_PULSES_PER_REV;
  if (cm < 0)
    encCm = -encCm;
  debugLog("MOVE cmd=%d rpm=%d pL=%lu pR=%lu enc_cm=%s yaw=%s imu=%d\r\n",
           cm, rpm, pL, pR, debugF1(encCm), debugF1(imuGetYawDeg()), imuIsReady() ? 1 : 0);
}

// A deg rpm: xoay PID dưới sàn (>0 phải, <0 trái), báo góc IMU và góc theo encoder
static void runSpin(int deg, int rpm)
{
  if (deg == 0 || rpm < 1)
    return;
  prepareFloor();
  motionSpinAnglePid((int16_t)deg, (uint16_t)rpm);
  HAL_Delay(NAV_STEP_PAUSE_MS);
  if (imuIsReady())
    imuUpdate();

  uint32_t pL = encoderGetPulses(MOTOR_LEFT);
  uint32_t pR = encoderGetPulses(MOTOR_RIGHT);
  float encDeg = ((float)(pL + pR) * 0.5f * 360.0f) / (float)ENC_SPIN_360_PULSES;
  if (deg < 0)
    encDeg = -encDeg;
  // Quy ước motion: >0 quay phải (IMU yaw âm)
  debugLog("SPIN cmd=%d rpm=%d pL=%lu pR=%lu enc_deg=%s imu_deg=%s imu=%d\r\n",
           deg, rpm, pL, pR, debugF1(encDeg), debugF1(-imuGetYawDeg()), imuIsReady() ? 1 : 0);
}

void motorBenchRun(void)
{
  char line[MOTOR_BENCH_LINE_MAX];
  char ch = 0;
  int a = 0, b = 0, c = 0, d = 0;

  readLine(line, MOTOR_BENCH_LINE_MAX);
  if (line[0] == '\0')
    return;

  if (line[0] == 'P')
    debugLog("BENCH_READY\r\n");
  else if (line[0] == 'D' && sscanf(&line[1], "%d %d", &a, &b) == 2)
    runDir(a, b);
  else if (line[0] == 'E' && sscanf(&line[1], " %c %d %d", &ch, &a, &b) == 3)
    runEnc(ch, a, b);
  else if (line[0] == 'K' && sscanf(&line[1], " %c %d %d %d %d", &ch, &a, &b, &c, &d) == 5)
    runCurve(ch, a, b, c, d);
  else if (line[0] == 'V' && sscanf(&line[1], " %c %d %d", &ch, &a, &b) == 3)
    runVelocity(ch, a, b);
  else if (line[0] == 'M' && sscanf(&line[1], "%d %d", &a, &b) == 2)
    runMove(a, b);
  else if (line[0] == 'A' && sscanf(&line[1], "%d %d", &a, &b) == 2)
    runSpin(a, b);
  else
    debugLog("BENCH_ERR cmd=%s\r\n", line);

  motorStopAll();
  debugLog("BENCH_DONE\r\n");
}
