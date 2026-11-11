#include "debug.h"
#include "project_config.h"
#include "oled.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart1;

static const char *targetTypeToStr(ScanTargetType type)
{
  if (type == TARGET_OBJECT) return "OBJ_FOUND";
  if (type == TARGET_OBSTACLE) return "OBSTACLE";
  return "OBJ_NOTFND";
}

int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void debugInit(void)
{
  oledInit();
}

void debugScan(float d05RawCm, float d04RawCm, uint8_t angleDeg, uint8_t hitCount, ScanTargetType type)
{
  const char *status = targetTypeToStr(type);
  float d04AdjCm = (d04RawCm >= SCAN_MIN_RANGE_CM) ? (d04RawCm - SENSOR_DEPTH_OFFSET_CM) : d04RawCm;
  float diffCm = (d05RawCm >= SCAN_MIN_RANGE_CM && d04RawCm >= SCAN_MIN_RANGE_CM) ? (d04AdjCm - d05RawCm) : 0.0f;

  printf("SCAN ang=%3u d05_raw=%.1f d04_raw=%.1f d04_adj=%.1f diff=%.1f hit=%u/%u status=%s\r\n",
         angleDeg, d05RawCm, d04RawCm, d04AdjCm, diffCm, hitCount, SCAN_CONFIRM_COUNT, status);
  oledPrintLine(0, "A:%3ud C:%1u", angleDeg, hitCount);
  oledPrintLine(1, "S5:%5.1fcm", d05RawCm);
  oledPrintLine(2, "S4:%5.1fcm", d04RawCm);
  oledPrintLine(3, "%s", status);
}

void debugScanFine(uint8_t angleDeg, float d05RawCm, float d04RawCm, ScanTargetType type,
                   float bestDistCm, uint8_t bestAngleDeg)
{
  const char *status = targetTypeToStr(type);
  float d04AdjCm = (d04RawCm >= SCAN_MIN_RANGE_CM) ? (d04RawCm - SENSOR_DEPTH_OFFSET_CM) : d04RawCm;

  printf("FINE ang=%3u d05_raw=%.1f d04_raw=%.1f d04_adj=%.1f status=%s best_d=%.1f best_ang=%u\r\n",
         angleDeg, d05RawCm, d04RawCm, d04AdjCm, status, bestDistCm, bestAngleDeg);
  oledPrintLine(0, "A:%3ud FIN", angleDeg);
  oledPrintLine(1, "S5:%5.1fcm", d05RawCm);
  oledPrintLine(2, "S4:%5.1fcm", d04RawCm);
  oledPrintLine(3, "%s", status);
}

void debugAlign(int16_t coarseSpinDeg, uint8_t fineAngleDeg, int16_t fineSpinDeg, float bestDistCm, bool fineFound)
{
  printf("ALIGN coarse_spin=%d fine_ang=%u fine_spin=%d best_d=%.1f status=%s\r\n",
         coarseSpinDeg, fineAngleDeg, fineSpinDeg, bestDistCm, fineFound ? "FINE_OK" : "FINE_SKIP");
}

void debugDrive(uint32_t elapsedMs, float d05RawCm, float d04RawCm, float errorCm,
                float pTerm, float iTerm, float dTerm, uint8_t pwmPercent, uint8_t lostCount)
{
  printf("DRV t=%4lums d05_raw=%.1f d04_raw=%.1f err=%.2f P=%.2f I=%.2f D=%.2f pwm=%u lost=%u/%u status=DRV_PID\r\n",
         elapsedMs, d05RawCm, d04RawCm, errorCm, pTerm, iTerm, dTerm, pwmPercent, lostCount, PID_LOST_MAX);
  oledPrintLine(0, "P%2u%% E%4.1f", pwmPercent, errorCm);
  oledPrintLine(1, "S5:%5.1fcm", d05RawCm);
  oledPrintLine(2, "S4:%5.1fcm", d04RawCm);
  oledPrintLine(3, "DRV_PID");
}

void debugDriveEnd(uint32_t elapsedMs, float d05RawCm, float d04RawCm, uint8_t lostCount, const char *status)
{
  printf("DRV_END t=%4lums d05_raw=%.1f d04_raw=%.1f lost=%u/%u status=%s\r\n",
         elapsedMs, d05RawCm, d04RawCm, lostCount, PID_LOST_MAX, status);
  oledPrintLine(0, "T:%5lums", elapsedMs);
  oledPrintLine(1, "S5:%5.1fcm", d05RawCm);
  oledPrintLine(2, "S4:%5.1fcm", d04RawCm);
  oledPrintLine(3, "%s", status);
}

void debugGripReady(float d05RawCm, float d04RawCm, int16_t spinDeg, int16_t advCm)
{
  printf("GRIP d05_raw=%.1f d04_raw=%.1f spin_deg=%d adv_cm=%d status=GRIP_RDY\r\n",
         d05RawCm, d04RawCm, spinDeg, advCm);
  oledPrintLine(0, "G:%4d deg", spinDeg);
  oledPrintLine(1, "S5:%5.1fcm", d05RawCm);
  oledPrintLine(2, "S4:%5.1fcm", d04RawCm);
  oledPrintLine(3, "GRIP_RDY");
}

void debugMotorTest(const char *step)
{
  printf("TEST step=%s\r\n", step);
  oledPrintLine(0, "MOTOR TEST");
  oledPrintLine(3, "%s", step);
}

void debugMotorPwmTest(uint16_t duty, uint8_t percent, const char *status)
{
  printf("TEST_PWM Duty=%4u (PWM=%2u%%) - %s\r\n", duty, percent, status);
  oledPrintLine(0, (strncmp(status, "SPIN", 4) == 0) ? "TEST SPIN" : "TEST PWM");
  oledPrintLine(1, "DUTY:%4u", duty);
  oledPrintLine(2, "PWM : %2u%%", percent);
  oledPrintLine(3, "%-10s", status);
}
