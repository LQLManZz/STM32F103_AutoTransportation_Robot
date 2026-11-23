#ifndef AGV_H
#define AGV_H

// Khởi tạo toàn bộ module của xe
void agvInit(void);

// Một vòng hoạt động: quét tìm vật/tường -> xoay + vi chỉnh hướng -> chạy tới gần vật (hoặc né tường)
void agvRun(void);

// Khởi tạo riêng module scanner và grabber để chạy test tương tác (không chạy động cơ/IMU)
void agvTestScanGrabInit(void);

// Một chu kỳ test tương tác: quét tìm vật -> gắp -> quét xác minh đã gắp -> đặt xuống -> quét xác minh đã đặt
void agvTestScanGrabRun(void);

// Chạy bộ 9 bài test quét cung phân biệt vật/tường (tự chấm pass/fail, nghỉ 15s giữa các bài)
void agvTestScannerSuiteRun(void);

#endif // AGV_H
