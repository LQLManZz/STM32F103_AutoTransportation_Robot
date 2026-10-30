#ifndef MOTION_H
#define MOTION_H

#include "motor.h"

void motionInit(void);

// Xoay tại chỗ theo góc: > 0 sang phải, < 0 sang trái
void motionSpinAngle(int16_t angleDeg, uint8_t speedPercent);

// Chạy thẳng theo cự ly có tăng/giảm tốc: > 0 tiến, < 0 lùi
void motionDriveDistance(int16_t distanceCm, uint8_t speedPercent);

#endif // MOTION_H
