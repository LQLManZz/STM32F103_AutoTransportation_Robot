#include "debug.h"
#include "project_config.h"
#include "encoder.h"
#include "imu.h"
#include "lcd.h"
#include "motor.h"
#include "srf05.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart1;

// Nhãn cắt ngắn bị mất nghĩa -> mã riêng (còn lại cắt còn 4 ô quyết định / 8 ô trạng thái)
static const char *const lcdShortCodes[][2] = {
    {"FORWARD", "FWD"}, {"TRIM-L", "TR-L"}, {"TRIM-R", "TR-R"}, {"GRAB_PICK", "PICK"},
    {"GRAB_DROP", "DROP"}, {"OBJ_FOUND", "OBJ_FND"}, {"OBJ_NOTFND", "NO_OBJ"},
    {"DRV_WALL_FAR", "DRV_WFAR"}, {"RETURN", "RET"}, {"FINE_SKIP", "FINE_SKP"},
    {"DELIVERED", "DELIVER"}, {"GRIP_CLOSE", "GRIP_CLS"}};

// 4 ô nội dung LCD 16x2:
// dòng 0: [góc 4][vị trí vật 7] [quyết định 4]  vd " 90° 12/ 19 SCAN"
// dòng 1: [RPM trái|phải 7] [trạng thái 8]      vd " 65| 64 OBJ_FND"
static char lcdAngle[LCD_ANGLE_CHARS + 1];
static char lcdPos[LCD_POS_CHARS + 1];
static char lcdDecision[LCD_DECISION_CHARS + 1];
static char lcdStatus[LCD_STATUS_CHARS + 1];

static const char *targetTypeToStr(ScanTargetType type)
{
  if (type == TARGET_OBJECT) return "OBJ_FOUND";
  if (type == TARGET_OBSTACLE) return "OBSTACLE_FND";
  return "OBJ_NOTFND";
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

// Rút gọn nhãn vào ô LCD: TURN-90L -> L90, nhãn dài tra bảng, còn lại cắt bớt
static void lcdSetCode(char *dst, uint8_t width, const char *label)
{
  if (label == NULL)
    return;

  size_t n = strlen(label);
  if (n > 5u && strncmp(label, "TURN", 4u) == 0)
  {
    snprintf(dst, width + 1u, "%c%.*s", label[n - 1u], (int)(n - 6u), &label[5]);
    return;
  }

  for (uint8_t i = 0u; i < sizeof(lcdShortCodes) / sizeof(lcdShortCodes[0]); i++)
  {
    if (strcmp(label, lcdShortCodes[i][0]) == 0)
    {
      label = lcdShortCodes[i][1];
      break;
    }
  }
  snprintf(dst, width + 1u, "%s", label);
}

// Cự ly 3 ô: số cm nguyên, " <2" quá gần, "ERR" mất sóng, "---" ngoài tầm
static const char *lcdDist(float cm)
{
  static char bufs[2][4];
  static uint8_t idx = 0u;
  char *b = bufs[idx++ & 1u];

  if (cm < ECHO_ERROR_TIMEOUT) return " <2";
  if (cm < 0.0f) return "ERR";
  if (cm > SRF04_MAX_RANGE_CM) return "---";
  snprintf(b, 4u, "%3ld", lroundf(cm));
  return b;
}

// Ô góc servo (kèm °) và ô cự ly SRF05 dưới/trên
static void lcdSetScan(uint8_t angleDeg, float d05RawCm, float d04RawCm)
{
  snprintf(lcdAngle, sizeof(lcdAngle), "%3u%c", angleDeg, LCD_DEGREE_CHAR);
  snprintf(lcdPos, sizeof(lcdPos), "%s/%s", lcdDist(d05RawCm), lcdDist(d04RawCm));
}

// Ô góc hiển thị heading IMU có dấu (không kèm ° để phân biệt góc servo), xóa cự ly cũ
static void lcdSetHeading(float headingDeg)
{
  snprintf(lcdAngle, sizeof(lcdAngle), "%4d", (int)lroundf(headingDeg));
  lcdPos[0] = '\0';
}

// Ô góc = hướng xe, ô vị trí = tọa độ x,y hiện tại căn phải
static void lcdSetNavPose(float curX, float curY, float curThetaDeg)
{
  char xy[LCD_COLS + 1];

  lcdSetHeading(curThetaDeg);
  snprintf(xy, sizeof(xy), "%d,%d", (int)lroundf(curX), (int)lroundf(curY));
  snprintf(lcdPos, sizeof(lcdPos), "%*.*s", (int)LCD_POS_CHARS, (int)LCD_POS_CHARS, xy);
}

static void lcdRefresh(void)
{
  lcdPrintLine(0, "%-*s%-*s %s", (int)LCD_ANGLE_CHARS, lcdAngle, (int)LCD_POS_CHARS, lcdPos, lcdDecision);
  lcdPrintLine(1, "%3u|%3u %s", encoderGetRpm(MOTOR_LEFT), encoderGetRpm(MOTOR_RIGHT), lcdStatus);
}

static void lcdUpdate(const char *decision, const char *status)
{
  lcdSetCode(lcdDecision, LCD_DECISION_CHARS, decision);
  lcdSetCode(lcdStatus, LCD_STATUS_CHARS, status);
  lcdRefresh();
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
  uint8_t lcdAddr7 = 0u;
  bool lcdOk = lcdInit(&lcdAddr7);

  // NO_ACK: không module nào trả lời, WR_ERR: có địa chỉ nhưng gửi lệnh khởi tạo lỗi
  printf("LCD addr=0x%02X status=%s\r\n", lcdAddr7, lcdOk ? "OK" : (lcdAddr7 ? "WR_ERR" : "NO_ACK"));
  lcdSetScan(SCAN_HOME_ANGLE, ECHO_OUT_OF_RANGE_CM, ECHO_OUT_OF_RANGE_CM);
  lcdUpdate("STOP", "INIT");
}

void debugDecision(const char *decision)
{
  uartPrintMotorTelemetry(decision);
  lcdUpdate(decision, NULL);
}

void debugScan(float d05RawCm, float d04RawCm, uint8_t angleDeg, uint8_t hitCount, ScanTargetType type)
{
  const char *status = targetTypeToStr(type);
  bool d05InRange = (d05RawCm >= SCAN_MIN_RANGE_CM && d05RawCm <= SRF05_MAX_RANGE_CM);
  bool d04InRange = (d04RawCm >= SCAN_MIN_RANGE_CM && d04RawCm <= SRF04_MAX_RANGE_CM);
  float d04AdjCm = d04InRange ? (d04RawCm - SENSOR_DEPTH_OFFSET_CM) : d04RawCm;
  float diffCm = (d05InRange && d04InRange) ? (d04AdjCm - d05RawCm) : 0.0f;

  printf("SCAN ang=%3u SRF05_bot=%s(%s) SRF05_top=%s(%s) top_adj=%s diff=%s hit=%u/%u status=%s\r\n",
         angleDeg, debugF1(d05RawCm), srf05GetStateStr(false),
         debugF1(d04RawCm), srf05GetStateStr(true),
         debugF1(d04AdjCm), debugF1(diffCm),
         hitCount, SCAN_CONFIRM_COUNT, status);
  lcdSetScan(angleDeg, d05RawCm, d04RawCm);
  lcdUpdate("SCANNING", status);
}

void debugScanFine(uint8_t angleDeg, float d05RawCm, float d04RawCm, ScanTargetType type,
                   float bestDistCm, uint8_t bestAngleDeg)
{
  const char *status = targetTypeToStr(type);
  bool d04InRange = (d04RawCm >= SCAN_MIN_RANGE_CM && d04RawCm <= SRF04_MAX_RANGE_CM);
  float d04AdjCm = d04InRange ? (d04RawCm - SENSOR_DEPTH_OFFSET_CM) : d04RawCm;

  printf("FINE ang=%3u SRF05_bot=%s(%s) SRF05_top=%s(%s) top_adj=%s status=%s best_d=%s best_ang=%u\r\n",
         angleDeg, debugF1(d05RawCm), srf05GetStateStr(false),
         debugF1(d04RawCm), srf05GetStateStr(true),
         debugF1(d04AdjCm), status, debugF1(bestDistCm), bestAngleDeg);
  lcdSetScan(angleDeg, d05RawCm, d04RawCm);
  lcdUpdate("SCANNING", status);
}

void debugAlign(int16_t coarseSpinDeg, uint8_t fineAngleDeg, int16_t fineSpinDeg, float bestDistCm, bool fineFound)
{
  const char *status = fineFound ? "FINE_OK" : "FINE_SKIP";

  printf("ALIGN coarse_spin=%d fine_ang=%u fine_spin=%d best_d=%s status=%s\r\n",
         coarseSpinDeg, fineAngleDeg, fineSpinDeg, debugF1(bestDistCm), status);
  lcdUpdate(NULL, status);
}

void debugApproach(uint32_t elapsedMs, float d05RawCm, float d04RawCm, float targetDistCm, bool wallMode)
{
  uint16_t leftRpm = encoderGetRpm(MOTOR_LEFT);
  uint16_t rightRpm = encoderGetRpm(MOTOR_RIGHT);
  const char *status = wallMode ? "OBSTACLE_FND" : "OBJ_FOUND";

  printf("%s t=%4lums SRF05_bot=%s SRF05_top=%s target=%s L=%urpm R=%urpm\r\n",
         wallMode ? "WALL_DRV" : "DRV", elapsedMs,
         debugF1(d05RawCm), debugF1(d04RawCm), debugF1(targetDistCm), leftRpm, rightRpm);
  lcdSetScan(SCAN_HOME_ANGLE, d05RawCm, d04RawCm);
  lcdUpdate("FORWARD", status);
}

void debugDriveEnd(uint32_t elapsedMs, float d05RawCm, float d04RawCm, uint8_t lostCount, const char *status)
{
  printf("DRV_END t=%4lums SRF05_bot=%s SRF05_top=%s lost=%u/%u status=%s\r\n",
         elapsedMs, debugF1(d05RawCm), debugF1(d04RawCm), lostCount, PID_LOST_MAX, status);
  lcdSetScan(SCAN_HOME_ANGLE, d05RawCm, d04RawCm);
  lcdUpdate(NULL, status);
}

void debugWallDetected(float d05RawCm, float d04RawCm)
{
  printf("WALL_DETECT SRF05_bot=%s SRF05_top=%s status=OBSTACLE_FND\r\n",
         debugF1(d05RawCm), debugF1(d04RawCm));
  lcdSetScan(SCAN_HOME_ANGLE, d05RawCm, d04RawCm);
  lcdUpdate("FORWARD", "OBSTACLE_FND");
}

void debugWallTurn(bool leftHasWall, float leftDistCm, bool rightHasWall, float rightDistCm, int16_t turnDeg)
{
  const char *dir = (turnDeg < 0) ? "TURN-90L" : ((turnDeg == WALL_TURN_BACK_DEG) ? "TURN-180R" : "TURN-90R");
  const char *leftStr = leftHasWall ? "WALL" : ((leftDistCm < 0.0f) ? "SKIP" : "FREE");
  const char *rightStr = rightHasWall ? "WALL" : ((rightDistCm < 0.0f) ? "SKIP" : "FREE");
  char status[LCD_STATUS_CHARS + 1];

  printf("WALL_TURN left=%s(%s) right=%s(%s) turn=%s(%d)\r\n",
         leftStr, debugF1(leftDistCm),
         rightStr, debugF1(rightDistCm),
         dir, turnDeg);
  // Trạng thái 2 bên rút còn 1 chữ: W tường, F trống, S bỏ qua
  snprintf(status, sizeof(status), "L:%c R:%c", leftStr[0], rightStr[0]);
  lcdUpdate(dir, status);
}

void debugMotorTest(const char *step)
{
  uartPrintMotorTelemetry(step);
  lcdSetHeading(imuGetHeadingDeg());
  lcdUpdate(step, "MTR_TEST");
}

void debugMotorPwmTest(uint16_t duty, uint8_t percent, const char *status)
{
  printf("TEST_PWM Duty=%4u (PWM=%2u%%) - %s\r\n", duty, percent, status);
  // Ô góc thay bằng % PWM, ô vị trí thay bằng duty (0..999)
  snprintf(lcdAngle, sizeof(lcdAngle), "%3u%%", percent);
  snprintf(lcdPos, sizeof(lcdPos), " D%u", duty);
  lcdUpdate(status, "PWM_TEST");
}

void debugState(const char *decision, const char *status)
{
  lcdSetHeading(imuGetHeadingDeg());
  lcdUpdate(decision, status);
}

void debugNavWaypoint(uint8_t idx, uint8_t total, float curX, float curY, float curThetaDeg,
                      float targetX, float targetY, float targetThetaDeg, const char *phase)
{
  char status[LCD_COLS + 1];

  printf("NAV wp=%u/%u phase=%s cur=(%s,%s,%s) target=(%s,%s,%s)\r\n",
         idx, total, phase,
         debugF1(curX), debugF1(curY), debugF1(curThetaDeg),
         debugF1(targetX), debugF1(targetY), debugF1(targetThetaDeg));
  lcdSetNavPose(curX, curY, curThetaDeg);
  snprintf(status, sizeof(status), "WP%u/%u", idx, total);
  lcdUpdate(phase, status);
}

void debugNavPreview(uint8_t idx, uint8_t total, float curX, float curY, float curThetaDeg,
                     float targetX, float targetY, float targetThetaDeg,
                     const char *nextAction, uint8_t secLeft, bool imuReady)
{
  char status[LCD_COLS + 1];

  if (secLeft == (uint8_t)(NAV_STEP_PREVIEW_MS / 1000u))
  {
    printf("NAV_NEXT wp=%u/%u next=%s (%us) cur=(%s,%s,%s) target=(%s,%s,%s) imu=%d\r\n",
           idx, total, nextAction, secLeft,
           debugF1(curX), debugF1(curY), debugF1(curThetaDeg),
           debugF1(targetX), debugF1(targetY), debugF1(targetThetaDeg),
           imuReady ? 1 : 0);
  }
  lcdSetNavPose(curX, curY, curThetaDeg);
  snprintf(status, sizeof(status), "WP%u/%u %us", idx, total, secLeft);
  lcdUpdate(nextAction, status);
}

void debugStepPreview(uint8_t stepIdx, uint8_t stepTotal, const char *nextAction,
                      uint8_t secLeft, bool imuReady)
{
  char status[LCD_COLS + 1];

  if (secLeft == (uint8_t)(NAV_STEP_PREVIEW_MS / 1000u))
    printf("TEST_NEXT step=%u/%u action=%s (%us)\r\n", stepIdx, stepTotal, nextAction, secLeft);

  lcdSetHeading(imuGetHeadingDeg());
  // Trạng thái: bước/tổng và giây đếm ngược (IMU đã báo ở bước calib)
  (void)imuReady;
  snprintf(status, sizeof(status), "S%u/%u %us", stepIdx, stepTotal, secLeft);
  lcdUpdate(nextAction, status);
}

void debugGrabber(uint8_t gripDeg, uint8_t liftDeg, const char *action, const char *status)
{
  printf("GRAB action=%s grip=%u lift=%u status=%s\r\n", action, gripDeg, liftDeg, status);
  // Ô góc = góc servo kẹp, ô vị trí = góc servo nâng
  snprintf(lcdAngle, sizeof(lcdAngle), "%3u%c", gripDeg, LCD_DEGREE_CHAR);
  snprintf(lcdPos, sizeof(lcdPos), "%4u%c", liftDeg, LCD_DEGREE_CHAR);
  lcdUpdate(action, status);
}
