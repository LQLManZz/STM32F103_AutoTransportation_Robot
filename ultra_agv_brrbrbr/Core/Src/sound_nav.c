/**
  ******************************************************************************
  * @file    sound_nav.c
  * @brief   Điều hướng đến vị trí nguồn âm, căn chỉnh siêu âm và tiếp cận chính xác
  ******************************************************************************
  */

#include "sound_nav.h"
#include "target_parser.h"
#include "motor.h"
#include "oled.h"
#include "lcd_i2c.h"
#include "srf05.h"
#include "scanner.h"
#include "servo.h"
#include "project_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define PI_CONST 3.1415926535f

extern TIM_HandleTypeDef htim2;

static void showScreen(const char *l0, const char *l1, const char *l2, const char *l3)
{
  oledPrintLine(0, "%-10s", l0);
  oledPrintLine(1, "%-10s", l1);
  oledPrintLine(2, "%-10s", l2);
  oledPrintLine(3, "%-10s", l3);

  char buf[24];
  snprintf(buf, sizeof(buf), "%-16s", l0);
  LCD_PrintLine(0, buf);
  snprintf(buf, sizeof(buf), "%-7s %-8s", l1, l3);
  LCD_PrintLine(1, buf);
}

// Xoay thân xe tại chỗ theo góc lệch (độ) ở tốc độ thấp nhất (Duty 350)
static void SoundNav_SpinAngleMinSpeed(float angleDeg)
{
  if (fabsf(angleDeg) <= 2.5f)
  {
    return;
  }

  MotorDir dirLeft  = (angleDeg > 0.0f) ? MOTOR_FORWARD : MOTOR_BACKWARD;
  MotorDir dirRight = (angleDeg > 0.0f) ? MOTOR_BACKWARD : MOTOR_FORWARD;

  // Xung mồi kickstart 40ms phá ma sát tĩnh ban đầu
  motorSetDutyRaw(MOTOR_LEFT, dirLeft, SOUND_KICK_DUTY);
  motorSetDutyRaw(MOTOR_RIGHT, dirRight, SOUND_KICK_DUTY);
  HAL_Delay(SOUND_KICK_TIME_MS);

  // Duy trì ở tốc độ xoay thấp nhất có thể (Duty 350)
  motorSetDutyRaw(MOTOR_LEFT, dirLeft, SOUND_SPIN_DUTY_MIN);
  motorSetDutyRaw(MOTOR_RIGHT, dirRight, SOUND_SPIN_DUTY_MIN);

  // Tính thời gian quay dựa trên thông số đã được tinh chỉnh (4.6 ms/độ)
  uint32_t spinMs = (uint32_t)(fabsf(angleDeg) * SOUND_SPIN_MS_PER_DEG + SOUND_SPIN_OFFSET_MS);
  if (spinMs > SOUND_KICK_TIME_MS)
  {
    spinMs -= SOUND_KICK_TIME_MS;
  }
  else
  {
    spinMs = 0;
  }
  HAL_Delay(spinMs);

  // Phanh cứng dừng xoay ngay lập tức
  motorBrakeAll();
  HAL_Delay(150);
}

// Chạy thẳng một quãng đường chính xác (cm) ở tốc độ thấp nhất rồi phanh dừng
static void SoundNav_DriveExactDistance(float distanceCm)
{
  if (distanceCm <= 0.5f)
  {
    motorBrakeAll();
    return;
  }

  printf("DRIVE_EXACT: Chay theo khoang cach da tinh = %.1f cm o Duty %u...\r\n",
         distanceCm, SOUND_DRIVE_DUTY_MIN);

  // 1. Xung mồi kickstart 40ms để xe lăn bánh từ trạng thái dừng
  motorSetDutyRaw(MOTOR_LEFT, MOTOR_FORWARD, SOUND_KICK_DUTY);
  motorSetDutyRaw(MOTOR_RIGHT, MOTOR_FORWARD, SOUND_KICK_DUTY);
  HAL_Delay(SOUND_KICK_TIME_MS);

  // 2. Chạy đều ở tốc độ thấp nhất (Duty 270)
  motorSetDutyRaw(MOTOR_LEFT, MOTOR_FORWARD, SOUND_DRIVE_DUTY_MIN);
  motorSetDutyRaw(MOTOR_RIGHT, MOTOR_FORWARD, SOUND_DRIVE_DUTY_MIN);

  uint32_t driveMs = (uint32_t)(distanceCm * SOUND_DRIVE_MS_PER_CM);
  if (driveMs > SOUND_KICK_TIME_MS)
  {
    driveMs -= SOUND_KICK_TIME_MS;
  }
  else
  {
    driveMs = 0;
  }

  // Chạy đủ thời gian tính toán
  HAL_Delay(driveMs);

  // 3. TỚI NƠI -> LẬP TỨC PHANH CỨNG KHÓA BÁNH XE!
  motorBrakeAll();
  HAL_Delay(200); // Chờ xe đứng yên hoàn toàn và triệt tiêu quán tính
}

// Quét vi chỉnh bằng siêu âm quanh góc 90° để kiểm tra mũi xe đã thẳng tắp với vật chưa
// Nếu chưa thẳng (xoay quá hoặc non góc) -> tự động xoay bù vi chỉnh cho thẳng tuyệt đối
bool SoundNav_VerifyAndAlignWithObject(float *outDistanceCm)
{
  printf("VERIFY_ALIGN: Kiem tra do thang huong voi vat the bang sieu am...\r\n");
  showScreen("VERIFY", "CHECK ALIGN", "DANG QUET..", "CHECKING");

  // Đưa servo về 90 độ trước
  moveServoSmooth(&htim2, SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);

  // Quét góc hẹp quanh 90 độ (70°..110°) với bước 5°
  const uint8_t testAngles[] = {70, 75, 80, 85, 90, 95, 100, 105, 110};
  const uint8_t numAngles = sizeof(testAngles) / sizeof(testAngles[0]);

  uint8_t bestAngle = SCAN_HOME_ANGLE;
  float minDistance = 999.0f;
  bool foundValid = false;

  for (uint8_t i = 0; i < numAngles; i++)
  {
    uint8_t ang = testAngles[i];
    setServoAngle(&htim2, ang);
    HAL_Delay(70);

    // Đo 2 lần để lọc nhiễu sóng phản hồi
    float d1 = measureDistanceCM();
    HAL_Delay(50);
    float d2 = measureDistanceCM();
    float d = (d1 > 0.0f && d2 > 0.0f) ? ((d1 + d2) / 2.0f) : ((d1 > 0.0f) ? d1 : d2);

    printf("  Angle %3u deg -> Dist = %.1f cm\r\n", ang, d);

    if (d >= SCAN_MIN_RANGE_CM && d <= SCAN_MAX_RANGE_CM)
    {
      if (d < minDistance)
      {
        minDistance = d;
        bestAngle = ang;
        foundValid = true;
      }
    }
  }

  // Đưa servo về lại vị trí thẳng 90 độ
  moveServoSmooth(&htim2, SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);

  if (!foundValid)
  {
    printf("VERIFY_ALIGN: Khong tim thay vat the trong dai goc hep 70..110 deg!\r\n");
    return false;
  }

  // Tính góc lệch vi chỉnh:
  // 90 độ là thẳng, nếu bestAngle = 80 deg (<90) -> lệch phải -> xe xoay phải +10 deg
  // nếu bestAngle = 100 deg (>90) -> lệch trái -> xe xoay trái -10 deg
  int16_t fineOffset = (int16_t)SCAN_HOME_ANGLE - (int16_t)bestAngle;

  printf("VERIFY_ALIGN: Tim thay vat o goc %u deg (lech %+d deg), culy = %.1f cm\r\n",
         bestAngle, fineOffset, minDistance);

  char l1[16], l2[16];
  snprintf(l1, sizeof(l1), "BEST:%3u DEG", bestAngle);
  snprintf(l2, sizeof(l2), "OFFSET:%+2d D", fineOffset);

  // Nếu góc lệch > 2 độ, tiến hành nhích xoay vi chỉnh bù lại
  if (abs(fineOffset) > 2)
  {
    showScreen("FINE ALIGN", l1, l2, "ADJUSTING");
    printf("VERIFY_ALIGN: Xoay vi chinh %+d deg de chieu thang vao vat...\r\n", fineOffset);
    SoundNav_SpinAngleMinSpeed((float)fineOffset);
  }
  else
  {
    showScreen("VERIFY OK", l1, l2, "PERFECT!");
    printf("VERIFY_ALIGN: Mui xe da THANG TUP voi vat the!\r\n");
    HAL_Delay(250);
  }

  // Đo lại khoảng cách sau khi xe đã xoay thẳng
  float finalD = measureDistanceCM();
  if (finalD < SCAN_MIN_RANGE_CM || finalD > SCAN_MAX_RANGE_CM)
  {
    finalD = minDistance;
  }

  if (outDistanceCm)
  {
    *outDistanceCm = finalD;
  }

  return true;
}

// Chạy đúng cự ly đã tính rồi dừng lại, sau khi dừng mới đo kiểm tra xem đã tới chưa
void SoundNav_DriveAndVerifyArrival(float targetDistanceCm)
{
  // Cự ly xe cần chạy = Khoảng cách đo được - Cự ly dừng mong muốn (3.0 cm)
  float driveDistCm = targetDistanceCm - SOUND_STOP_DIST_CM;
  if (driveDistCm < 0.0f)
  {
    driveDistCm = 0.0f;
  }

  printf("APPROACH: Khoang cach den vat = %.1f cm -> Chay dung %.1f cm roi dung lai\r\n",
         targetDistanceCm, driveDistCm);

  char l1[16], l2[16];
  snprintf(l1, sizeof(l1), "CALC:%.1fCM", driveDistCm);
  snprintf(l2, sizeof(l2), "STOP:%.1fCM", SOUND_STOP_DIST_CM);
  showScreen("DRIVE CALC", l1, l2, "RUNNING..");

  // BƯỚC 1: Chạy đúng khoảng cách đã tính rồi DỪNG HẲN
  SoundNav_DriveExactDistance(driveDistCm);

  // BƯỚC 2: SAU KHI ĐÃ DỪNG HẲN -> MỚI ĐO KIỂM TRA XEM ĐÃ TỚI CHƯA
  showScreen("CHECK STOP", "XE DA DUNG", "DANG DO LAI", "CHECKING");
  printf("CHECK_ARRIVAL: Xe da dung! Dang dung sieu am do lai xem da toi chua...\r\n");

  // Đảm bảo servo ở vị trí 90 độ thẳng phía trước
  setServoAngle(&htim2, SCAN_HOME_ANGLE);
  HAL_Delay(100);

  // Đo 3 lần lấy trung bình để chính xác
  float d1 = measureDistanceCM(); HAL_Delay(50);
  float d2 = measureDistanceCM(); HAL_Delay(50);
  float d3 = measureDistanceCM();

  float postDist = -1.0f;
  uint8_t validCount = 0;
  float sum = 0.0f;
  if (d1 > 0.0f) { sum += d1; validCount++; }
  if (d2 > 0.0f) { sum += d2; validCount++; }
  if (d3 > 0.0f) { sum += d3; validCount++; }
  if (validCount > 0)
  {
    postDist = sum / (float)validCount;
  }
  else if (d1 == -2.0f || d2 == -2.0f || d3 == -2.0f)
  {
    postDist = 2.0f; // Quá gần (< 2cm)
  }

  printf("CHECK_ARRIVAL: Khoang cach thuc te sau khi dung = %.1f cm (d1=%.1f, d2=%.1f, d3=%.1f)\r\n",
         postDist, d1, d2, d3);

  // BƯỚC 3: ĐÁNH GIÁ KẾT QUẢ VÀ VI CHỈNH NẾU DỪNG NON
  // Trường hợp 1: Nếu cự ly đo được <= 4.5 cm (hoặc < 2cm): ĐÃ TỚI ĐÍCH CHUẨN XÁC!
  if (postDist > 0.0f && postDist <= (SOUND_STOP_DIST_CM + 1.5f))
  {
    snprintf(l1, sizeof(l1), "DIST:%.1fCM", postDist);
    showScreen("ARRIVED!", l1, "CHUAN XAC", "SUCCESS!");
    printf("CHECK_ARRIVAL: DA DUNG CHUAN XAC CACH VAT %.1f cm!\r\n", postDist);
  }
  // Trường hợp 2: Nếu xe bị dừng non (vẫn còn cách xa > 4.5 cm, ví dụ do trượt bánh nhẹ trên sàn)
  else if (postDist > (SOUND_STOP_DIST_CM + 1.5f) && postDist <= 20.0f)
  {
    float remainDist = postDist - SOUND_STOP_DIST_CM;
    printf("CHECK_ARRIVAL: Xe dung hoi xa (con %.1f cm) -> Nhich them %.1f cm...\r\n",
           postDist, remainDist);
    snprintf(l1, sizeof(l1), "CON:%.1fCM", remainDist);
    showScreen("FINE TUNE", l1, "NHICH THEM", "CREEPING");

    // Nhích thêm cự ly còn thiếu
    SoundNav_DriveExactDistance(remainDist);

    // Kiểm tra lại lần cuối
    HAL_Delay(100);
    float finalCheck = measureDistanceCM();
    snprintf(l1, sizeof(l1), "DIST:%.1fCM", finalCheck > 0.0f ? finalCheck : 3.0f);
    showScreen("ARRIVED!", l1, "HOAN THANH", "SUCCESS!");
    printf("CHECK_ARRIVAL: Da nhich xong, culy cuoi cung = %.1f cm!\r\n", finalCheck);
  }
  else
  {
    // Trường hợp không đo được (vật quá sát khiến sóng âm tịt < 2cm) hoặc đã chạm vật
    snprintf(l1, sizeof(l1), "DIST:%.1fCM", postDist > 0.0f ? postDist : 3.0f);
    showScreen("ARRIVED!", l1, "DA TIEP CAN", "SUCCESS!");
    printf("CHECK_ARRIVAL: Da tiep can vat the thanh cong!\r\n");
  }

  // Khóa phanh chắc chắn
  motorBrakeAll();
}

void SoundNav_Init(void)
{
  TargetParser_Init();
  scanInit(); // Khởi tạo PWM servo trên TIM2 CH2 và đưa servo về HOME (90°)
}

// Xoay servo quét tìm vật thể khi phía trước không thấy vật
bool SoundNav_ScanAndApproach(void)
{
  printf("SCAN_SERVO: Bat dau xoay servo quet 0..180 deg tim vat...\r\n");
  showScreen("SCAN SERVO", "DANG QUET", "0..180 DEG", "SCANNING..");

  float scanDist = 0.0f;
  uint8_t scanAngle = SCAN_HOME_ANGLE;

  if (scanForObject(&scanDist, &scanAngle) == SCAN_OBJECT_FOUND)
  {
    printf("SCAN_SERVO: TIM THAY VAT tai goc=%u deg, culy=%.1f cm\r\n", scanAngle, scanDist);

    // Tính góc lệch: 90° là thẳng, < 90° lệch phải, > 90° lệch trái
    int16_t offsetDeg = (int16_t)SCAN_HOME_ANGLE - (int16_t)scanAngle;

    char l1[16], l2[16];
    snprintf(l1, sizeof(l1), "ANG:%3u DEG", scanAngle);
    snprintf(l2, sizeof(l2), "DST:%3.0f CM", scanDist);
    showScreen("OBJECT FND", l1, l2, "ALIGNING..");

    // Đưa servo về lại vị trí thẳng 90°
    moveServoSmooth(&htim2, SCAN_HOME_ANGLE);
    HAL_Delay(SCAN_SETTLE_MS);

    // 1. Xoay thân xe sơ bộ về hướng vật thể ở tốc độ thấp nhất (Duty 350)
    if (abs(offsetDeg) > 3)
    {
      SoundNav_SpinAngleMinSpeed((float)offsetDeg);
    }

    // 2. DÙNG CẢM BIẾN SIÊU ÂM QUÉT LẠI ĐỂ KIỂM TRA ĐÃ THẲNG VẬT CHƯA
    float verifiedDist = scanDist;
    if (SoundNav_VerifyAndAlignWithObject(&verifiedDist))
    {
      // 3. Chạy theo khoảng cách đã tính rồi dừng lại, sau đó mới đo tính xem đã tới chưa
      SoundNav_DriveAndVerifyArrival(verifiedDist);
    }
    else
    {
      // Nếu quét hẹp không thấy, dùng cự ly ban đầu để chạy tới
      SoundNav_DriveAndVerifyArrival(scanDist);
    }

    return true;
  }
  else
  {
    printf("SCAN_SERVO: Khong tim thay vat trong ban kinh quet!\r\n");
    moveServoSmooth(&htim2, SCAN_HOME_ANGLE);
    HAL_Delay(SCAN_SETTLE_MS);
    return false;
  }
}

// Di chuyển tới tọa độ (x, y) với tốc độ thấp nhất có thể
void SoundNav_GoTo(float x, float y)
{
  float headingDeg = atan2f(x, y) * (180.0f / PI_CONST);
  float totalDistCm = sqrtf(x * x + y * y);

  printf("SOUND_NAV: Tgt X=%.1f, Y=%.1f -> Ang=%.1f deg, Dist=%.1f cm\r\n",
         x, y, headingDeg, totalDistCm);

  char l1[16], l2[16];
  snprintf(l1, sizeof(l1), "X:%3.0f Y:%3.0f", x, y);
  snprintf(l2, sizeof(l2), "A:%3.0f D:%3.0f", headingDeg, totalDistCm);

  // 1. BƯỚC 1: Xoay hướng xe ở tốc độ thấp nhất (Duty 350)
  if (fabsf(headingDeg) > 2.5f)
  {
    showScreen("SOUND NAV", l1, l2, "ALIGNING..");
    SoundNav_SpinAngleMinSpeed(headingDeg);
  }

  // 2. BƯỚC 2: QUÉT LẠI BẰNG CẢM BIẾN SIÊU ÂM ĐỂ KIỂM TRA MŨI XE ĐÃ THẲNG VẬT CHƯA
  float verifiedDist = 0.0f;
  if (SoundNav_VerifyAndAlignWithObject(&verifiedDist))
  {
    // Đã căn chỉnh thẳng tắp với vật!
    // Chạy đúng cự ly đã tính rồi dừng lại, sau khi dừng mới đo kiểm tra xem đã tới chưa!
    SoundNav_DriveAndVerifyArrival(verifiedDist);
  }
  else
  {
    // Nếu góc nhìn hẹp chưa thấy (do tọa độ âm thanh ban đầu lệch xa), kích hoạt quét rộng 0..180°
    printf("SOUND_NAV: Quet hep chua thay vat -> Quet rong tim vat...\r\n");
    SoundNav_ScanAndApproach();
  }
}

// Vòng lặp kiểm tra và thực thi: nhận tọa độ âm thanh hoặc quét servo khi không phát hiện ra
void SoundNav_Process(void)
{
  static bool waitingShown = false;

  // 1. Nếu có tọa độ âm thanh mới từ UART1 -> Điều hướng tới mục tiêu
  if (TargetParser_HasNewTarget())
  {
    float x = 0.0f;
    float y = 0.0f;
    if (TargetParser_GetTarget(&x, &y))
    {
      SoundNav_GoTo(x, y);
      waitingShown = false;
      HAL_Delay(2000);
      return;
    }
  }

  // 2. Nếu chưa có tọa độ âm thanh:
  if (!waitingShown)
  {
    showScreen("SOUND NAV", "WAIT COORD", "RX: PA10", "SCAN IF NO");
    printf("SOUND_NAV: San sang nhan toa do UART1 (PA10) hoac quet servo tim vat...\r\n");
    waitingShown = true;
  }

  // 3. Nếu ở phía trước không phát hiện ra vật thể (khoảng cách > 50cm hoặc không có echo):
  // Xoay servo để quét tìm vật thể!
  float d = measureDistanceCM();
  if (d < SCAN_MIN_RANGE_CM || d > SCAN_MAX_RANGE_CM)
  {
    if (SoundNav_ScanAndApproach())
    {
      waitingShown = false;
      HAL_Delay(2000);
    }
  }
}
