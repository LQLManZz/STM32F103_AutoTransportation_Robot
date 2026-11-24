#include "scanbench.h"
#include "project_config.h"
#include "debug.h"
#include "scanner.h"
#include "servo.h"
#include "srf05.h"
#include <stdio.h>

extern UART_HandleTypeDef huart1;

// Chờ đủ 1 dòng lệnh kết thúc bằng '\n' (bỏ '\r', cắt bớt nếu quá dài)
static void readLine(char *buf, uint8_t maxLen)
{
  uint8_t len = 0u;
  __HAL_UART_CLEAR_OREFLAG(&huart1);
  while (1)
  {
    uint8_t ch = 0u;
    if (HAL_UART_Receive(&huart1, &ch, 1u, SCAN_BENCH_RX_TIMEOUT_MS) != HAL_OK)
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

// Giới hạn số mẫu mỗi góc trong 1..SCAN_BENCH_MAX_SAMPLES
static uint8_t clampSamples(int n)
{
  if (n < 1)
    return 1u;
  if (n > (int)SCAN_BENCH_MAX_SAMPLES)
    return SCAN_BENCH_MAX_SAMPLES;
  return (uint8_t)n;
}

// Đo n cặp thô (không lọc 2/3) tại góc servo hiện tại
static void measureRawAt(const char *tag, uint8_t angleDeg, uint8_t samples)
{
  for (uint8_t i = 0u; i < samples; i++)
  {
    float d05RawCm = ECHO_ERROR_TIMEOUT;
    float d04RawCm = ECHO_ERROR_TIMEOUT;
    measureDualDistanceCM(&d05RawCm, &d04RawCm);
    debugBench(tag, i, angleDeg, d05RawCm, d04RawCm);
  }
}

// R ang n: đo thô tại 1 góc cố định
static void runRaw(int angleDeg, int n)
{
  moveServoSmooth((uint8_t)angleDeg);
  HAL_Delay(SCAN_SETTLE_MS);
  measureRawAt("RAW", getServoAngle(), clampSamples(n));
}

// S start end step n: quét thô từng góc, mỗi góc n mẫu
static void runSweep(int startDeg, int endDeg, int stepDeg, int n)
{
  if (stepDeg < 1)
    stepDeg = 1;
  int dir = (startDeg <= endDeg) ? stepDeg : -stepDeg;
  uint8_t samples = clampSamples(n);

  moveServoSmooth((uint8_t)startDeg);
  HAL_Delay(SCAN_SETTLE_MS);
  for (int a = startDeg; (dir > 0) ? (a <= endDeg) : (a >= endDeg); a += dir)
  {
    if (a != startDeg)
    {
      moveServoSmooth((uint8_t)a);
      HAL_Delay(SCAN_STEP_SETTLE_MS);
    }
    measureRawAt("SWP", getServoAngle(), samples);
  }
}

// C: chạy nguyên thuật toán scanForObject 1 lần
static void runClassify(void)
{
  float distCm = 0.0f;
  uint8_t angleDeg = SCAN_HOME_ANGLE;
  ScanStatus st = scanForObject(&distCm, &angleDeg);
  const char *resStr = (st == SCAN_OBJECT_FOUND) ? "OBJ" : ((st == SCAN_WALL_FOUND) ? "WALL" : "NONE");
  ScanExtraInfo extra;
  scanGetLastExtraInfo(&extra);
  debugLog("BENCH_RESULT st=%s ang=%u d=%s suspect=%u s_ang=%u s_d=%s\r\n", resStr, angleDeg, debugF1(distCm),
           extra.hasSuspect ? 1u : 0u, extra.suspectAngleDeg, debugF1(extra.suspectDistCm));
}

void scanBenchRun(void)
{
  char line[SCAN_BENCH_LINE_MAX];
  int a = 0, b = 0, c = 0, d = 0;

  // Tắt ngắt encoder để ISR không làm sai độ rộng xung Echo khi dời xe bằng tay
  HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
  readLine(line, SCAN_BENCH_LINE_MAX);
  if (line[0] == '\0')
    return;

  if (line[0] == 'P')
    debugLog("BENCH_READY\r\n");
  else if (line[0] == 'R' && sscanf(&line[1], "%d %d", &a, &b) == 2)
    runRaw(a, b);
  else if (line[0] == 'S' && sscanf(&line[1], "%d %d %d %d", &a, &b, &c, &d) == 4)
    runSweep(a, b, c, d);
  else if (line[0] == 'C')
    runClassify();
  else
    debugLog("BENCH_ERR cmd=%s\r\n", line);

  debugLog("BENCH_DONE\r\n");
}
