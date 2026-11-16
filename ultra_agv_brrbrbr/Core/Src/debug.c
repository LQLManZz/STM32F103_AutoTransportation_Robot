#include "debug.h"
#include "project_config.h"
#include "motor.h"
#include "oled.h"
#include <stdio.h>

extern UART_HandleTypeDef huart1;

static const char *targetTypeToStr(ScanTargetType type)
{
  if (type == TARGET_OBJECT) return "OBJ_FOUND";
  if (type == TARGET_OBSTACLE) return "OBSTACLE_FND";
  return "OBJ_NOTFND";
}

static void oledPrintCurrentPwm(void)
{
  oledPrintLine(3, "L:%2u%% | R:%2u%%",
                motorGetSpeedPercent(MOTOR_LEFT), motorGetSpeedPercent(MOTOR_RIGHT));
}

static void oledUpdate(uint8_t angleDeg, float d05RawCm, float d04RawCm,
                       const char *decision, const char *status)
{
  oledPrintLine(0, "A:%3u | %.1f-%.1fcm", angleDeg, d05RawCm, d04RawCm);
  if (decision)
    oledPrintLine(1, "%s", decision);
  if (status)
    oledPrintLine(2, "%s", status);
  oledPrintCurrentPwm();
}

int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void debugInit(void)
{
  oledInit();
  oledUpdate(SCAN_HOME_ANGLE, 0.0f, 0.0f, "STOP", "INIT");
}

void debugDecision(const char *decision)
{
  oledPrintLine(1, "%s", decision);
  oledPrintCurrentPwm();
}

void debugScan(float d05RawCm, float d04RawCm, uint8_t angleDeg, uint8_t hitCount, ScanTargetType type)
{
  const char *status = targetTypeToStr(type);
  float d04AdjCm = (d04RawCm >= SCAN_MIN_RANGE_CM) ? (d04RawCm - SENSOR_DEPTH_OFFSET_CM) : d04RawCm;
  float diffCm = (d05RawCm >= SCAN_MIN_RANGE_CM && d04RawCm >= SCAN_MIN_RANGE_CM) ? (d04AdjCm - d05RawCm) : 0.0f;

  printf("SCAN ang=%3u d05_raw=%.1f d04_raw=%.1f d04_adj=%.1f diff=%.1f hit=%u/%u status=%s\r\n",
         angleDeg, d05RawCm, d04RawCm, d04AdjCm, diffCm, hitCount, SCAN_CONFIRM_COUNT, status);
  oledUpdate(angleDeg, d05RawCm, d04RawCm, "SCANNING", status);
}

void debugScanFine(uint8_t angleDeg, float d05RawCm, float d04RawCm, ScanTargetType type,
                   float bestDistCm, uint8_t bestAngleDeg)
{
  const char *status = targetTypeToStr(type);
  float d04AdjCm = (d04RawCm >= SCAN_MIN_RANGE_CM) ? (d04RawCm - SENSOR_DEPTH_OFFSET_CM) : d04RawCm;

  printf("FINE ang=%3u d05_raw=%.1f d04_raw=%.1f d04_adj=%.1f status=%s best_d=%.1f best_ang=%u\r\n",
         angleDeg, d05RawCm, d04RawCm, d04AdjCm, status, bestDistCm, bestAngleDeg);
  oledUpdate(angleDeg, d05RawCm, d04RawCm, "SCANNING", status);
}

void debugAlign(int16_t coarseSpinDeg, uint8_t fineAngleDeg, int16_t fineSpinDeg, float bestDistCm, bool fineFound)
{
  const char *status = fineFound ? "FINE_OK" : "FINE_SKIP";

  printf("ALIGN coarse_spin=%d fine_ang=%u fine_spin=%d best_d=%.1f status=%s\r\n",
         coarseSpinDeg, fineAngleDeg, fineSpinDeg, bestDistCm, status);
  oledPrintLine(2, "%s", status);
  oledPrintCurrentPwm();
}

void debugApproach(uint32_t elapsedMs, float d05RawCm, float d04RawCm, float targetDistCm, bool wallMode)
{
  uint8_t leftPct = motorGetSpeedPercent(MOTOR_LEFT);
  uint8_t rightPct = motorGetSpeedPercent(MOTOR_RIGHT);
  const char *status = wallMode ? "OBSTACLE_FND" : "OBJ_FOUND";

  printf("%s t=%4lums d05_raw=%.1f d04_raw=%.1f target=%.1f L=%u%% R=%u%%\r\n",
         wallMode ? "WALL_DRV" : "DRV", elapsedMs, d05RawCm, d04RawCm, targetDistCm, leftPct, rightPct);
  oledUpdate(SCAN_HOME_ANGLE, d05RawCm, d04RawCm, "FORWARD", status);
}

void debugDriveEnd(uint32_t elapsedMs, float d05RawCm, float d04RawCm, uint8_t lostCount, const char *status)
{
  printf("DRV_END t=%4lums d05_raw=%.1f d04_raw=%.1f lost=%u/%u status=%s\r\n",
         elapsedMs, d05RawCm, d04RawCm, lostCount, PID_LOST_MAX, status);
  oledUpdate(SCAN_HOME_ANGLE, d05RawCm, d04RawCm, NULL, status);
}

void debugWallDetected(float d05RawCm, float d04RawCm)
{
  printf("WALL_DETECT d05_raw=%.1f d04_raw=%.1f status=OBSTACLE_FND\r\n", d05RawCm, d04RawCm);
  oledUpdate(SCAN_HOME_ANGLE, d05RawCm, d04RawCm, "FORWARD", "OBSTACLE_FND");
}

void debugWallTurn(bool leftHasWall, float leftDistCm, bool rightHasWall, float rightDistCm, int16_t turnDeg)
{
  const char *dir = (turnDeg < 0) ? "TURN-90L" : ((turnDeg == WALL_TURN_BACK_DEG) ? "TURN-180R" : "TURN-90R");

  printf("WALL_TURN left=%s(%.1f) right=%s(%.1f) turn=%s(%d)\r\n",
         leftHasWall ? "WALL" : "FREE", leftDistCm,
         rightHasWall ? "WALL" : "FREE", rightDistCm,
         dir, turnDeg);
  oledPrintLine(1, "%s", dir);
  oledPrintLine(2, "L:%s R:%s", leftHasWall ? "WALL" : "FREE", rightHasWall ? "WALL" : "FREE");
  oledPrintCurrentPwm();
}

void debugMotorTest(const char *step)
{
  printf("TEST step=%s\r\n", step);
  oledPrintLine(1, "%s", step);
  oledPrintLine(2, "MOTOR_TEST");
  oledPrintCurrentPwm();
}

void debugMotorPwmTest(uint16_t duty, uint8_t percent, const char *status)
{
  printf("TEST_PWM Duty=%4u (PWM=%2u%%) - %s\r\n", duty, percent, status);
  oledPrintLine(0, "DUTY:%4u PWM:%2u%%", duty, percent);
  oledPrintLine(1, "%s", status);
  oledPrintLine(2, "PWM_TEST");
  oledPrintCurrentPwm();
}
