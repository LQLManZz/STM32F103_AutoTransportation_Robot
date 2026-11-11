#ifndef MOTORTEST_H
#define MOTORTEST_H

// Chạy một lần kịch bản test động cơ: tiến, lùi, quay phải/trái 90°, quay phải/trái 180°
void motorTestRun(void);

// Kịch bản test tìm ngưỡng PWM thấp nhất khi chạy tiến
void motorTestMinPwm(void);

// Kịch bản test tìm ngưỡng PWM thấp nhất khi xoay tại chỗ (đảo chiều mỗi chu kỳ)
void motorTestMinSpinPwm(void);

// Kịch bản test kích hoạt trực tiếp ở mức PWM_TEST_SPIN_MIN_DUTY từ trạng thái đứng yên
void motorTestDirectSpin350(void);

// Xoay liên tục không ngừng ở tốc độ thấp nhất có thể
void motorSpinContinuousLowestSpeed(void);

#endif // MOTORTEST_H
