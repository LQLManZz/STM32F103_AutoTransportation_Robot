#ifndef DEBUG_H
#define DEBUG_H

#include "stm32f1xx_hal.h"
#include "scanner.h"
#include <stdbool.h>

// Khởi tạo màn hình LCD 16x2 (serial dùng sẵn UART1)
void debugInit(void);

// Định dạng số thực 1 chữ số thập phân (không phụ thuộc cờ -u _printf_float)
const char *debugF1(float v);

// Xuất một dòng log dạng TAG key=value ra UART1
void debugLog(const char *format, ...);

// Cập nhật ô quyết định hệ thống (FORWARD, BACKWARD, TURN-90R, SCANNING...) và RPM trên LCD
void debugDecision(const char *decision);

// Báo thông số thực tế mỗi lần đo khi quét thô
void debugScan(float d05RawCm, float d04RawCm, uint8_t angleDeg, uint8_t hitCount, ScanTargetType type);

// Báo thông số thực tế mỗi bước quét tinh (60°..120°)
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

// Báo thông số khi test tìm ngưỡng PWM tối thiểu: duty (0-999), % PWM, trạng thái
void debugMotorPwmTest(uint16_t duty, uint8_t percent, const char *status);

// Cập nhật LCD: ô góc hiện heading IMU, kèm quyết định và trạng thái
void debugState(const char *decision, const char *status);

// Báo trạng thái điều hướng theo tọa độ Turn - Drive - Turn
void debugNavWaypoint(uint8_t idx, uint8_t total, float curX, float curY, float curThetaDeg,
                      float targetX, float targetY, float targetThetaDeg, const char *phase);

// Hiển thị đếm ngược hoạt động sắp thực hiện lên LCD trước khi xe chạy
void debugNavPreview(uint8_t idx, uint8_t total, float curX, float curY, float curThetaDeg,
                     float targetX, float targetY, float targetThetaDeg,
                     const char *nextAction, uint8_t secLeft, bool imuReady);

// Hiển thị đếm ngược bước tiếp theo của kịch bản test lên LCD
void debugStepPreview(uint8_t stepIdx, uint8_t stepTotal, const char *nextAction,
                      uint8_t secLeft, bool imuReady);

// Báo góc 2 servo tay gắp và trạng thái thao tác lên Serial + LCD
void debugGrabber(uint8_t gripDeg, uint8_t liftDeg, const char *action, const char *status);

// Báo 1 mẫu đo thô của bài test bench scanner (tag RAW/SWP) lên Serial + LCD
void debugBench(const char *tag, uint8_t sampleIdx, uint8_t angleDeg, float d05RawCm, float d04RawCm);

#endif // DEBUG_H
