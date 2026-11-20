#include "debug.h"
#include "project_config.h"
#include "blackbox.h"
#include "encoder.h"
#include "imu.h"
#include "motor.h"
#include "oled.h"
#include "srf05.h"
#include <math.h>
#include <stdarg.h>
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
  oledPrintLine(3, "L:%3u | R:%3u rpm",
                encoderGetRpm(MOTOR_LEFT), encoderGetRpm(MOTOR_RIGHT));
}

static void uartPrintMotorTelemetry(const char *step)
{
  printf("TEST step=%s L=%urpm R=%urpm pL=%lu pR=%lu dL=%u dR=%u H=%s\r\n",
         step,
         encoderGetRpm(MOTOR_LEFT), encoderGetRpm(MOTOR_RIGHT),
         encoderGetPulses(MOTOR_LEFT), encoderGetPulses(MOTOR_RIGHT),
         motorGetDutyRaw(MOTOR_LEFT), motorGetDutyRaw(MOTOR_RIGHT),
         debugF1(imuGetHeadingDeg()));
}

static void oledUpdate(uint8_t angleDeg, float d05RawCm, float d04RawCm,
                       const char *decision, const char *status)
{
  oledPrintLine(0, "A:%3u | %s-%scm", angleDeg, debugF1(d05RawCm), debugF1(d04RawCm));
  if (decision)
    oledPrintLine(1, "%s", decision);
  if (status)
    oledPrintLine(2, "%s", status);
  oledPrintCurrentPwm();
}

const char *debugF1(float v)
{
  static char bufs[8][16];
  static uint8_t idx = 0u;
  char *b = bufs[idx++ & 7u];
  int32_t t = (int32_t)lroundf(v * 10.0f);
  if (t < 0)
  {
    t = -t;
    snprintf(b, 16u, "-%ld.%ld", (long)(t / 10), (long)(t % 10));
  }
  else
  {
    snprintf(b, 16u, "%ld.%ld", (long)(t / 10), (long)(t % 10));
  }
  return b;
}

int __io_putchar(int ch)
{
  blackboxPutChar((char)ch);
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void debugLog(const char *format, ...)
{
  va_list args;
  va_start(args, format);
  vprintf(format, args);
  va_end(args);
}

void debugInit(void)
{
  oledInit();
  oledUpdate(SCAN_HOME_ANGLE, 0.0f, 0.0f, "STOP", "INIT");
}

void debugDecision(const char *decision)
{
  uartPrintMotorTelemetry(decision);
  oledPrintLine(1, "%s", decision);
  oledPrintCurrentPwm();
}

void debugScan(float d05RawCm, float d04RawCm, uint8_t angleDeg, uint8_t hitCount, ScanTargetType type)
{
  const char *status = targetTypeToStr(type);
  bool d05InRange = (d05RawCm >= SCAN_MIN_RANGE_CM && d05RawCm <= SRF05_MAX_RANGE_CM);
  bool d04InRange = (d04RawCm >= SCAN_MIN_RANGE_CM && d04RawCm <= SRF04_MAX_RANGE_CM);
  float d04AdjCm = d04InRange ? (d04RawCm - SENSOR_DEPTH_OFFSET_CM) : d04RawCm;
  float diffCm = (d05InRange && d04InRange) ? (d04AdjCm - d05RawCm) : 0.0f;

  printf("SCAN ang=%3u SRF05_bot=%s(%s) SRF04_top=%s(%s) srf04_adj=%s diff=%s hit=%u/%u status=%s\r\n",
         angleDeg, debugF1(d05RawCm), srf05GetStateStr(false),
         debugF1(d04RawCm), srf05GetStateStr(true),
         debugF1(d04AdjCm), debugF1(diffCm),
         hitCount, SCAN_CONFIRM_COUNT, status);
  oledUpdate(angleDeg, d05RawCm, d04RawCm, "SCANNING", status);
}

void debugScanFine(uint8_t angleDeg, float d05RawCm, float d04RawCm, ScanTargetType type,
                   float bestDistCm, uint8_t bestAngleDeg)
{
  const char *status = targetTypeToStr(type);
  bool d04InRange = (d04RawCm >= SCAN_MIN_RANGE_CM && d04RawCm <= SRF04_MAX_RANGE_CM);
  float d04AdjCm = d04InRange ? (d04RawCm - SENSOR_DEPTH_OFFSET_CM) : d04RawCm;

  printf("FINE ang=%3u SRF05_bot=%s(%s) SRF04_top=%s(%s) srf04_adj=%s status=%s best_d=%s best_ang=%u\r\n",
         angleDeg, debugF1(d05RawCm), srf05GetStateStr(false),
         debugF1(d04RawCm), srf05GetStateStr(true),
         debugF1(d04AdjCm), status, debugF1(bestDistCm), bestAngleDeg);
  oledUpdate(angleDeg, d05RawCm, d04RawCm, "SCANNING", status);
}

void debugAlign(int16_t coarseSpinDeg, uint8_t fineAngleDeg, int16_t fineSpinDeg, float bestDistCm, bool fineFound)
{
  const char *status = fineFound ? "FINE_OK" : "FINE_SKIP";

  printf("ALIGN coarse_spin=%d fine_ang=%u fine_spin=%d best_d=%s status=%s\r\n",
         coarseSpinDeg, fineAngleDeg, fineSpinDeg, debugF1(bestDistCm), status);
  oledPrintLine(2, "%s", status);
  oledPrintCurrentPwm();
}

void debugApproach(uint32_t elapsedMs, float d05RawCm, float d04RawCm, float targetDistCm, bool wallMode)
{
  uint16_t leftRpm = encoderGetRpm(MOTOR_LEFT);
  uint16_t rightRpm = encoderGetRpm(MOTOR_RIGHT);
  const char *status = wallMode ? "OBSTACLE_FND" : "OBJ_FOUND";

  printf("%s t=%4lums SRF05_bot=%s SRF04_top=%s target=%s L=%urpm R=%urpm\r\n",
         wallMode ? "WALL_DRV" : "DRV", elapsedMs,
         debugF1(d05RawCm), debugF1(d04RawCm), debugF1(targetDistCm), leftRpm, rightRpm);
  oledUpdate(SCAN_HOME_ANGLE, d05RawCm, d04RawCm, "FORWARD", status);
}

void debugDriveEnd(uint32_t elapsedMs, float d05RawCm, float d04RawCm, uint8_t lostCount, const char *status)
{
  printf("DRV_END t=%4lums SRF05_bot=%s SRF04_top=%s lost=%u/%u status=%s\r\n",
         elapsedMs, debugF1(d05RawCm), debugF1(d04RawCm), lostCount, PID_LOST_MAX, status);
  oledUpdate(SCAN_HOME_ANGLE, d05RawCm, d04RawCm, NULL, status);
}

void debugWallDetected(float d05RawCm, float d04RawCm)
{
  printf("WALL_DETECT SRF05_bot=%s SRF04_top=%s status=OBSTACLE_FND\r\n",
         debugF1(d05RawCm), debugF1(d04RawCm));
  oledUpdate(SCAN_HOME_ANGLE, d05RawCm, d04RawCm, "FORWARD", "OBSTACLE_FND");
}

void debugWallTurn(bool leftHasWall, float leftDistCm, bool rightHasWall, float rightDistCm, int16_t turnDeg)
{
  const char *dir = (turnDeg < 0) ? "TURN-90L" : ((turnDeg == WALL_TURN_BACK_DEG) ? "TURN-180R" : "TURN-90R");
  const char *leftStr = leftHasWall ? "WALL" : ((leftDistCm < 0.0f) ? "SKIP" : "FREE");
  const char *rightStr = rightHasWall ? "WALL" : ((rightDistCm < 0.0f) ? "SKIP" : "FREE");

  printf("WALL_TURN left=%s(%s) right=%s(%s) turn=%s(%d)\r\n",
         leftStr, debugF1(leftDistCm),
         rightStr, debugF1(rightDistCm),
         dir, turnDeg);
  oledPrintLine(1, "%s", dir);
  oledPrintLine(2, "L:%s R:%s", leftStr, rightStr);
  oledPrintCurrentPwm();
}

void debugMotorTest(const char *step)
{
  uartPrintMotorTelemetry(step);
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

void debugState(const char *decision, const char *status)
{
  oledPrintLine(0, "H:%sdeg", debugF1(imuGetHeadingDeg()));
  if (decision)
    oledPrintLine(1, "%s", decision);
  if (status)
    oledPrintLine(2, "%s", status);
  oledPrintCurrentPwm();
}

void debugNavWaypoint(uint8_t idx, uint8_t total, float curX, float curY, float curThetaDeg,
                      float targetX, float targetY, float targetThetaDeg, const char *phase)
{
  printf("NAV wp=%u/%u phase=%s cur=(%s,%s,%s) target=(%s,%s,%s)\r\n",
         idx, total, phase,
         debugF1(curX), debugF1(curY), debugF1(curThetaDeg),
         debugF1(targetX), debugF1(targetY), debugF1(targetThetaDeg));
  oledPrintLine(0, "WP%u/%u T:%d,%d,%dd", idx, total,
                (int)lroundf(targetX), (int)lroundf(targetY), (int)lroundf(targetThetaDeg));
  oledPrintLine(1, "%s", phase);
  oledPrintLine(2, "CUR:%d,%d %dd",
                (int)lroundf(curX), (int)lroundf(curY), (int)lroundf(curThetaDeg));
  oledPrintCurrentPwm();
}

void debugNavPreview(uint8_t idx, uint8_t total, float curX, float curY, float curThetaDeg,
                     float targetX, float targetY, float targetThetaDeg,
                     const char *nextAction, uint8_t secLeft, bool imuReady)
{
  if (secLeft == (uint8_t)(NAV_STEP_PREVIEW_MS / 1000u))
  {
    printf("NAV_NEXT wp=%u/%u next=%s (%us) cur=(%s,%s,%s) target=(%s,%s,%s) imu=%d\r\n",
           idx, total, nextAction, secLeft,
           debugF1(curX), debugF1(curY), debugF1(curThetaDeg),
           debugF1(targetX), debugF1(targetY), debugF1(targetThetaDeg),
           imuReady ? 1 : 0);
  }
  oledPrintLine(0, "WP%u/%u T:%d,%d,%dd", idx, total,
                (int)lroundf(targetX), (int)lroundf(targetY), (int)lroundf(targetThetaDeg));
  oledPrintLine(1, "%s %us", nextAction, secLeft);
  oledPrintLine(2, "CUR:%d,%d %dd",
                (int)lroundf(curX), (int)lroundf(curY), (int)lroundf(curThetaDeg));
  oledPrintCurrentPwm();
}

void debugStepPreview(uint8_t stepIdx, uint8_t stepTotal, const char *nextAction,
                      uint8_t secLeft, bool imuReady)
{
  if (secLeft == (uint8_t)(NAV_STEP_PREVIEW_MS / 1000u))
    printf("TEST_NEXT step=%u/%u action=%s (%us)\r\n", stepIdx, stepTotal, nextAction, secLeft);

  oledPrintLine(0, "H:%s | S:%u/%u", debugF1(imuGetHeadingDeg()), stepIdx, stepTotal);
  oledPrintLine(1, "%s %us", nextAction, secLeft);
  oledPrintLine(2, "%s", imuReady ? "IMU_OK" : "ENC_ONLY");
  oledPrintCurrentPwm();
}
