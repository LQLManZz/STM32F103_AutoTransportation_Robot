#ifndef DEBUG_H
#define DEBUG_H

#include "stm32f1xx_hal.h"
#include "scanner.h"
#include <stdbool.h>

// Khởi tạo màn hình OLED (serial dùng sẵn UART1)
void debugInit(void);

// Định dạng số thực 1 chữ số thập phân (không phụ thuộc cờ -u _printf_float)
const char *debugF1(float v);

// Xuất một dòng log dạng TAG key=value ra UART1
void debugLog(const char *format, ...);

// Cập nhật dòng 1 quyết định hệ thống (FORWARD, BACKWARD, TURN-90R, SCANNING...) và dòng 3 RPM
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

// Báo bước đang chạy của kịch bản test động cơ
void debugMotorTest(const char *step);

// Báo thông số khi test tìm ngưỡng PWM tối thiểu: duty (0-999), % PWM, trạng thái
void debugMotorPwmTest(uint16_t duty, uint8_t percent, const char *status);

// Cập nhật OLED theo đúng bố cục 4 dòng chuẩn: dòng 0 góc hướng IMU, dòng 1 quyết định, dòng 2 trạng thái, dòng 3 RPM
void debugState(const char *decision, const char *status);

// Báo trạng thái điều hướng theo tọa độ Turn - Drive - Turn
void debugNavWaypoint(uint8_t idx, uint8_t total, float curX, float curY, float curThetaDeg,
                      float targetX, float targetY, float targetThetaDeg, const char *phase);

// Hiển thị đếm ngược hoạt động sắp thực hiện lên OLED trước khi xe chạy
void debugNavPreview(uint8_t idx, uint8_t total, float curX, float curY, float curThetaDeg,
                     float targetX, float targetY, float targetThetaDeg,
                     const char *nextAction, uint8_t secLeft, bool imuReady);

// Hiển thị đếm ngược bước tiếp theo của kịch bản test lên OLED
void debugStepPreview(uint8_t stepIdx, uint8_t stepTotal, const char *nextAction,
                      uint8_t secLeft, bool imuReady);

// Báo góc 2 servo tay gắp và trạng thái thao tác lên Serial + OLED
void debugGrabber(uint8_t gripDeg, uint8_t liftDeg, const char *action, const char *status);

#endif // DEBUG_H
