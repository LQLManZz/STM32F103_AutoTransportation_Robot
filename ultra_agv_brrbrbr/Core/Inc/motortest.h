#ifndef MOTORTEST_H
#define MOTORTEST_H

#include <stdint.h>

// Chạy một lần kịch bản test động cơ: tiến, lùi, quay phải/trái 90°, quay phải/trái 180°
void motorTestRun(void);

// Chạy kịch bản test động cơ PID: tiến thẳng liên tục
void motorTestPidRun(void);

// Kịch bản test tìm ngưỡng PWM thấp nhất khi chạy tiến
void motorTestMinPwm(void);

// Kịch bản test tìm ngưỡng PWM thấp nhất khi xoay tại chỗ (đảo chiều mỗi chu kỳ)
void motorTestMinSpinPwm(void);

// Kịch bản test kích hoạt trực tiếp ở mức PWM_TEST_SPIN_MIN_DUTY từ trạng thái đứng yên
void motorTestDirectSpin350(void);

// Xoay liên tục không ngừng ở tốc độ thấp nhất có thể
void motorSpinContinuousLowestSpeed(void);

// Chạy thẳng liên tục không ngừng ở tốc độ PWM thấp nhất có thể
void motorDriveContinuousLowestSpeed(void);

// Chạy một lần kịch bản Turn - Drive - Turn qua danh sách tọa độ (x, y, z°)
void motorTestWaypointsRun(void);

// Kịch bản test chuẩn: tiến 1m, lùi 1m, xoay trái/phải 45°, 90°, 180° (có đếm ngược 3s trên LCD)
void motorTestMotionSuiteRun(void);

// Test PID xoay góc chuẩn (90, -90, 180, -180) và tổ hợp góc tùy ý (tổng về 0°)
void motorTestPidSpinRun(void);

// Test PID chạy thẳng tiến/lùi 50 cm về gốc
void motorTestPidDriveRun(void);

// Test PID xoay 1 góc tùy ý (>0 phải, <0 trái)
void motorTestPidSpinAngleRun(int16_t angleDeg);

#endif // MOTORTEST_H
