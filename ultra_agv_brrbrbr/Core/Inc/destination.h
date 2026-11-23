#ifndef DESTINATION_H
#define DESTINATION_H

#include "nav.h"
#include <stdint.h>

// Xóa lịch sử hành động khi bắt đầu phiên mới tại (0, 0, 0°)
void destinationReset(void);

// Lưu (và gộp) bước xoay thực tế (>0 quay phải CW, <0 quay trái CCW) kèm hướng IMU trước bước xoay
void destinationRecordSpin(int16_t actualSpinDeg, float prevHeadingDeg);

// Lưu (và gộp) quãng đường thực tế (>0 tiến, <0 lùi) kèm hướng IMU trước bước chạy
void destinationRecordDrive(int16_t actualDriveCm, float prevHeadingDeg);

// Trả số bước hành động đã gộp đang có trong ngăn xếp lịch sử
uint8_t destinationGetCount(void);

// Chạy ngược ngăn xếp hành động về vị trí xuất phát (0, 0, 0°) và cập nhật pose
void destinationReturnHome(NavPose *pose);

#endif // DESTINATION_H
