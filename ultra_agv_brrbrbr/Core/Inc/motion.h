#ifndef MOTION_H
#define MOTION_H

#include "motor.h"

void motionInit(void);

// Điều khiển chuyển động trực tiếp
void motionForward(uint8_t speedPercent);
void motionBackward(uint8_t speedPercent);
void motionBrake(void);
void motionStop(void);

// Xoay tại chỗ theo góc định lượng: > 0 sang phải, < 0 sang trái
void motionSpinAngle(int16_t angleDeg, uint8_t speedPercent);

// Chạy thẳng theo cự ly có tăng/giảm tốc: > 0 tiến, < 0 lùi
void motionDriveDistance(int16_t distanceCm, uint8_t speedPercent);

// Xoay tại chỗ theo góc dùng PID 2 encoder: > 0 sang phải, < 0 sang trái
void motionSpinAnglePid(int16_t angleDeg, uint16_t targetRpm);

// Chạy thẳng theo cự ly dùng PID 2 encoder: > 0 tiến, < 0 lùi
void motionDriveDistancePid(int16_t distanceCm, uint16_t targetRpm);

// Chạy thẳng hoặc lùi dùng PID 2 encoder theo thời gian (durationMs = 0: chạy liên tục)
void motionDriveTimePid(MotorDir dir, uint32_t durationMs, uint16_t targetRpm);

// Khởi tạo mốc giữ hướng bằng IMU + encoder trước khi tiến tiếp cận vật/tường
void motionForwardHoldStart(void);

// Cập nhật một bước chạy thẳng có giữ hướng bằng IMU (fallback encoder nếu chưa có IMU)
void motionForwardHoldStep(uint8_t speedPercent);

#endif // MOTION_H
