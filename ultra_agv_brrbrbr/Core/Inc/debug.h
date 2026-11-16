#ifndef DEBUG_H
#define DEBUG_H

#include "stm32f1xx_hal.h"
#include "scanner.h"
#include <stdbool.h>

// Khởi tạo màn hình OLED (serial dùng sẵn UART1)
void debugInit(void);

// Cập nhật dòng 2 quyết định hệ thống (FORWARD, BACKWARD, TURN-90R, SCANNING...) và dòng 4 PWM
void debugDecision(const char *decision);

// Báo thông số thực tế mỗi lần đo khi quét thô
void debugScan(float d05RawCm, float d04RawCm, uint8_t angleDeg, uint8_t hitCount, ScanTargetType type);

// Báo thông số thực tế mỗi bước quét tinh (70°..110°)
void debugScanFine(uint8_t angleDeg, float d05RawCm, float d04RawCm, ScanTargetType type,
                   float bestDistCm, uint8_t bestAngleDeg);

// Báo kết quả xoay căn hướng thô và vi chỉnh tinh
void debugAlign(int16_t coarseSpinDeg, uint8_t fineAngleDeg, int16_t fineSpinDeg, float bestDistCm, bool fineFound);

// Báo thông số khi xe chạy tiếp cận vật hoặc tường (không PID)
void debugApproach(uint32_t elapsedMs, float d05RawCm, float d04RawCm, float targetDistCm, bool wallMode);

// Báo kết thúc hành trình tiếp cận (tới đích, mất vật hoặc gặp vật cản)
void debugDriveEnd(uint32_t elapsedMs, float d05RawCm, float d04RawCm, uint8_t lostCount, const char *status);

// Báo khi phát hiện tường phía trước
void debugWallDetected(float d05RawCm, float d04RawCm);

// Báo kết quả kiểm tra tường trái/phải và hướng quay quyết định
void debugWallTurn(bool leftHasWall, float leftDistCm, bool rightHasWall, float rightDistCm, int16_t turnDeg);

// Báo bước đang chạy của kịch bản test động cơ
void debugMotorTest(const char *step);

// Báo thông số khi test tìm ngưỡng PWM tối thiểu: duty (0-999), % PWM, trạng thái
void debugMotorPwmTest(uint16_t duty, uint8_t percent, const char *status);

#endif // DEBUG_H
