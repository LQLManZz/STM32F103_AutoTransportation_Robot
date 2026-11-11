#ifndef MOTORTEST_H
#define MOTORTEST_H

// Kịch bản test tìm ngưỡng PWM thấp nhất khi chạy tiến
void motorTestMinPwm(void);

// Kịch bản test tìm ngưỡng PWM thấp nhất khi XOAY TẠI CHỖ (Spin in place):
// Xoay tròn liên tục từ Max PWM (650), giảm dần từng bước 10 duty (~1%)
// và luân phiên đảo chiều xoay giữa các chu kỳ để chống rối dây.
void motorTestMinSpinPwm(void);

// Kịch bản test kích hoạt trực tiếp ở mức Duty = 350 từ trạng thái đứng yên
// xem xe có thể tự phá ma sát nghỉ để xoay tại chỗ được không
void motorTestDirectSpin350(void);

// Xoay liên tục không ngừng ở tốc độ thấp nhất có thể (Duty = 350 ~ 35%)
void motorSpinContinuousLowestSpeed(void);

// Chạy một lần kịch bản test động cơ: tiến, lùi, quay phải/trái 90°, quay phải/trái 180°
void motorTestRun(void);

#endif // MOTORTEST_H
