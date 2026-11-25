// Phát lại CSV đo thật qua đúng scanner.c trên PC.
// gcc -I../../ultra_agv_brrbrbr/tests/scanner_host_test/stubs -I../../ultra_agv_brrbrbr/Core/Inc replay.c -o replay -lm
// replay sweep <file.csv>   : chạy scanForObject() với dữ liệu quét (góc gần nhất trong CSV)
// replay static <file.csv>  : phân loại toàn bộ mẫu 1 góc bằng classifySamples()
#include "stm32f1xx_hal.h"
#include "main.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../../ultra_agv_brrbrbr/Core/Src/scanner.c"

#define MAX_ROWS 2000
static int rowAng[MAX_ROWS];
static float rowBot[MAX_ROWS], rowTop[MAX_ROWS];
static int rowCount = 0;
static int angCursor[181];
static uint8_t servoAng = SCAN_HOME_ANGLE;

void HAL_Delay(uint32_t ms) { (void)ms; }
void servoInit(void) { servoAng = SCAN_HOME_ANGLE; }
void setServoAngle(uint8_t a) { servoAng = a; }
void moveServoSmooth(uint8_t a) { servoAng = a; }
uint8_t getServoAngle(void) { return servoAng; }
void servoStop(void) {}

const char *debugF1(float v)
{
  static char bufs[8][16];
  static int idx = 0;
  char *b = bufs[idx++ & 7];
  snprintf(b, 16, "%.1f", v);
  return b;
}
void debugLog(const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  vprintf(fmt, ap);
  va_end(ap);
}
void debugDecision(const char *d) { (void)d; }
void debugScan(float d05, float d04, uint8_t ang, uint8_t hit, ScanTargetType t)
{
  (void)hit;
  printf("  SCAN ang=%3u bot=%5.1f top=%5.1f type=%d\n", ang, d05, d04, (int)t);
}
void debugWallDetected(float d05, float d04) { printf("WALL_DETECT bot=%.1f top=%.1f\n", d05, d04); }

// Lấy mẫu kế tiếp ở góc ghi trong CSV gần góc servo nhất
static void nextSample(float *bot, float *top)
{
  int best = -1;
  for (int i = 0; i < rowCount; i++)
    if (best < 0 || abs(rowAng[i] - servoAng) < abs(rowAng[best] - servoAng))
      best = i;
  int ang = rowAng[best];
  int k = 0, pick = best;
  for (int i = 0; i < rowCount; i++)
    if (rowAng[i] == ang && k++ == angCursor[ang] % 3)
      pick = i;
  angCursor[ang]++;
  *bot = rowBot[pick];
  *top = rowTop[pick];
}

void measureDualDistanceCM(float *d05, float *d04)
{
  float b, t;
  nextSample(&b, &t);
  if (d05) *d05 = b;
  if (d04) *d04 = t;
}

static float median3(float a, float b, float c)
{
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

void measureDualDistanceConfirmCM(float firstD05, float firstD04, float *d05, float *d04)
{
  float b1, t1, b2, t2;
  measureDualDistanceCM(&b1, &t1);
  measureDualDistanceCM(&b2, &t2);
  *d05 = median3(firstD05, b1, b2);
  *d04 = median3(firstD04, t1, t2);
}

static void loadCsv(const char *path)
{
  FILE *f = fopen(path, "r");
  if (!f) { perror(path); exit(1); }
  char line[256];
  fgets(line, sizeof line, f);
  while (fgets(line, sizeof line, f) && rowCount < MAX_ROWS)
  {
    char tag[8], bst[16], tst[16];
    int i, ang;
    float b, t;
    if (sscanf(line, "%7[^,],%d,%d,%f,%15[^,],%f,%15s", tag, &i, &ang, &b, bst, &t, tst) == 7)
    {
      rowAng[rowCount] = ang;
      rowBot[rowCount] = b;
      rowTop[rowCount] = t;
      rowCount++;
    }
  }
  fclose(f);
}

int main(int argc, char **argv)
{
  if (argc < 3) { printf("usage: replay sweep|static file.csv\n"); return 1; }
  loadCsv(argv[2]);
  if (strcmp(argv[1], "static") == 0)
  {
    float diff = 0.0f;
    ScanTargetType t = classifySamples(rowBot, rowTop, (uint8_t)rowCount, &diff);
    if (t == TARGET_NONE)
    {
      float b[MAX_ROWS];
      memcpy(b, rowBot, sizeof(float) * (size_t)rowCount);
      t = (computeMedian(b, (uint8_t)rowCount) <= SCAN_TOP_BLIND_MAX_CM) ? TARGET_OBJECT : TARGET_OBSTACLE;
    }
    printf("STATIC n=%d diff=%.1f type=%s\n", rowCount, diff,
           t == TARGET_OBJECT ? "OBJ" : (t == TARGET_OBSTACLE ? "WALL" : "NONE"));
    return 0;
  }
  float d = 0.0f;
  uint8_t a = 0;
  ScanStatus st = scanForObject(&d, &a);
  ScanExtraInfo ex;
  scanGetLastExtraInfo(&ex);
  printf("RESULT st=%s ang=%u d=%.1f suspect=%d sang=%u sd=%.1f\n",
         st == SCAN_OBJECT_FOUND ? "OBJ" : (st == SCAN_WALL_FOUND ? "WALL" : "NONE"),
         a, d, ex.hasSuspect, ex.suspectAngleDeg, ex.suspectDistCm);
  return 0;
}
