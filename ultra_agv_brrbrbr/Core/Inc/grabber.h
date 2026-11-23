#ifndef GRABBER_H
#define GRABBER_H

#include "stm32f1xx_hal.h"

// Bật PWM 2 servo tay gắp và đưa về vị trí chờ (cần nâng, ngàm mở)
void grabberInit(void);

// Đặt trực tiếp góc servo kẹp (0°..180°)
void grabberSetGripAngle(uint8_t angleDeg);

// Đặt trực tiếp góc servo nâng hạ (0°..180°)
void grabberSetLiftAngle(uint8_t angleDeg);

// Quay mượt servo kẹp tới góc mục tiêu
void grabberMoveGripSmooth(uint8_t targetDeg);

// Quay mượt servo nâng hạ tới góc mục tiêu
void grabberMoveLiftSmooth(uint8_t targetDeg);

// Đọc góc hiện tại của servo kẹp và servo nâng hạ
uint8_t grabberGetGripAngle(void);
uint8_t grabberGetLiftAngle(void);

// Quy trình gắp vật: hạ cần -> kẹp vật -> nâng lên
void grabberPick(void);

// Quy trình nhả vật: hạ cần -> mở ngàm -> nâng cần về vị trí chờ
void grabberRelease(void);

// Chạy 1 chu kỳ test gắp - giữ - nhả (tương đương vòng lặp test trong tay.gap)
void grabberTestCycle(void);

// Ngắt xung PWM servo kẹp khi ngàm đang mở không tải để giảm dòng giữ
void grabberGripStop(void);

#endif // GRABBER_H
