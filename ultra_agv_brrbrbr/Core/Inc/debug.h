#ifndef DEBUG_H
#define DEBUG_H

#include "stm32f1xx_hal.h"
#include "scanner.h"
#include <stdbool.h>

// Khởi tạo màn hình OLED (serial dùng sẵn UART1)
void debugInit(void);

// Báo thông số thực tế mỗi lần đo khi quét thô
void debugScan(float d05RawCm, float d04RawCm, uint8_t angleDeg, uint8_t hitCount, ScanTargetType type);

// Báo thông số thực tế mỗi bước quét tinh (70°..110°)
void debugScanFine(uint8_t angleDeg, float d05RawCm, float d04RawCm, ScanTargetType type,
                   float bestDistCm, uint8_t bestAngleDeg);

// Báo kết quả xoay căn hướng thô và vi chỉnh tinh
void debugAlign(int16_t coarseSpinDeg, uint8_t fineAngleDeg, int16_t fineSpinDeg, float bestDistCm, bool fineFound);

// Báo thông số chi tiết từng chu kỳ chạy PID tiếp cận vật
void debugDrive(uint32_t elapsedMs, float d05RawCm, float d04RawCm, float errorCm,
                float pTerm, float iTerm, float dTerm, uint8_t pwmPercent, uint8_t lostCount);

// Báo kết thúc hành trình PID (tới đích, mất vật hoặc gặp vật cản)
void debugDriveEnd(uint32_t elapsedMs, float d05RawCm, float d04RawCm, uint8_t lostCount, const char *status);

// Báo trạng thái đã xoay bù đưa vật vào phạm vi tay gắp bên phải
void debugGripReady(float d05RawCm, float d04RawCm, int16_t spinDeg, int16_t advCm);

// Báo bước đang chạy của kịch bản test động cơ
void debugMotorTest(const char *step);

// Báo thông số khi test tìm ngưỡng PWM tối thiểu: duty (0-999), % PWM, trạng thái
void debugMotorPwmTest(uint16_t duty, uint8_t percent, const char *status);

#endif // DEBUG_H
