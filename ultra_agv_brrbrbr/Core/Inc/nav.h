#ifndef NAV_H
#define NAV_H

#include <stdint.h>

// Hệ tọa độ tại lúc Reset: (0, 0, 0°); +X tiến thẳng, +Y sang trái, +theta ngược chiều kim đồng hồ
typedef struct
{
  float xCm;
  float yCm;
  float thetaDeg;
} NavPose;

// Đặt lại tọa độ hiện tại về (0, 0, 0°)
void navResetPose(void);

// Lấy tọa độ ước lượng hiện tại của xe
NavPose navGetPose(void);

// Di chuyển từ tọa độ hiện tại tới (xCm, yCm, thetaDeg) theo trình tự Turn - Drive - Turn
void navGoToPose(const NavPose *target, uint8_t wpIndex, uint8_t wpTotal);

// Chạy tuần tự qua một danh sách điểm đích (x, y, z°)
void navFollowWaypoints(const NavPose *waypoints, uint8_t count);

#endif // NAV_H
