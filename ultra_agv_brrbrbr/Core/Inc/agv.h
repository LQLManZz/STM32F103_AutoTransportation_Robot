#ifndef AGV_H
#define AGV_H

#include "nav.h"
#include <stdbool.h>
#include <stdint.h>

// Khởi tạo toàn bộ module của xe
void agvInit(void);

// Một vòng hoạt động: quét tìm vật/tường -> xoay + vi chỉnh hướng -> chạy tới gần vật (hoặc né tường)
void agvRun(void);

// Lệnh xoay thân xe về tâm vật từ góc servo + cự ly SRF05 dưới (> 0 quay phải, 0: không cần xoay)
int16_t agvTurnCmdToObject(uint8_t angleDeg, float distanceCm);

// Bench: đặt lại tọa độ (0, 0, 0°), bộ đếm và bản đồ, chuyển FSM sang trạng thái quét
void agvBenchReset(void);

// Bench: xoay + chạy PID tiếp cận vật đã quét được (không gắp), true khi chốt được cự ly gắp
bool agvApproachObject(float distanceCm, uint8_t angleDeg, bool wallBackedObj);

// Bench: lấy tọa độ xe và tọa độ tâm vật hiện tại
void agvGetPose(NavPose *outPose, NavPose *outObj);

// Khởi tạo riêng module scanner và grabber để chạy test tương tác (không chạy động cơ/IMU)
void agvTestScanGrabInit(void);

// Một chu kỳ test tương tác: quét tìm vật -> gắp -> quét xác minh đã gắp -> đặt xuống -> quét xác minh đã đặt
void agvTestScanGrabRun(void);

// Chạy bộ 9 bài test quét cung phân biệt vật/tường (tự chấm pass/fail, nghỉ 15s giữa các bài)
void agvTestScannerSuiteRun(void);

#endif // AGV_H
