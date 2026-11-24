#ifndef DESTINATION_H
#define DESTINATION_H

#include "nav.h"
#include <stdbool.h>
#include <stdint.h>

// Xóa lịch sử hành động, mốc tọa độ và bản đồ tường 2D khi bắt đầu phiên mới tại (0, 0, 0°)
void destinationReset(void);

// Lưu (và gộp) bước xoay thực tế (>0 quay phải CW, <0 quay trái CCW) kèm hướng IMU trước bước xoay
void destinationRecordSpin(int16_t actualSpinDeg, float prevHeadingDeg);

// Lưu (và gộp) quãng đường thực tế (>0 tiến, <0 lùi) kèm hướng IMU trước bước chạy
void destinationRecordDrive(int16_t actualDriveCm, float prevHeadingDeg);

// Ghi nhận mốc tọa độ xe dọc chiều đi (phục vụ rút gọn quỹ đạo quay về)
void destinationRecordPose(const NavPose *pose);

// Thêm đoạn tường phẳng 2D trong hệ tọa độ gốc xe vào bản đồ không gian nhớ tạm
void destinationAddWallSeg(float x1Cm, float y1Cm, float x2Cm, float y2Cm,
                           uint8_t startDeg, uint8_t endDeg);

// Kiểm tra đoạn thẳng (ax, ay) -> (bx, by) có cách mọi đoạn tường đã biết >= clearanceCm hay không
bool destinationIsPathClear(float axCm, float ayCm, float bxCm, float byCm, float clearanceCm);

// Tính điểm vòng 2D qua đầu mút mở của tường chắn trước mặt và góc nhìn sau khi vượt góc
bool destinationPlanWallBypass(const NavPose *pose, NavPose *outTarget, float *outLookHeadingDeg);

// Trả số bước hành động đã gộp đang có trong ngăn xếp lịch sử
uint8_t destinationGetCount(void);

// Rút gọn quỹ đạo 2D an toàn (hoặc lùi theo lịch sử nếu tràn bộ nhớ) về (0, 0, 0°) và cập nhật pose
void destinationReturnHome(NavPose *pose);

// Chuẩn hóa góc (độ) về dải (-180°, 180°]
float destinationNormalizeDeg(float deg);

// Xoay tại chỗ và cập nhật góc pose (tùy chọn lưu lịch sử đường đi)
void destinationExecSpin(int16_t spinCmdDeg, NavPose *pose, bool recordHistory);

// Chạy thẳng/lùi, cập nhật tọa độ pose theo góc trung bình và trả về cự ly thực (cm)
float destinationExecDrive(int16_t driveCm, NavPose *pose, bool recordHistory);

#endif // DESTINATION_H
