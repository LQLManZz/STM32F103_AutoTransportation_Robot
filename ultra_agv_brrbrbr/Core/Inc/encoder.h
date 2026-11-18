#ifndef ENCODER_H
#define ENCODER_H

#include "motor.h"

// Khởi tạo bộ đếm tốc độ encoder 2 bánh
void encoderInit(void);

// Đặt lại bộ đếm xung 2 bánh về 0
void encoderResetPulses(void);

// Lấy tổng số xung đã đếm của một bánh
uint32_t encoderGetPulses(MotorId motor);

// Lấy tốc độ hiện tại của một bánh (vòng/phút)
uint16_t encoderGetRpm(MotorId motor);

#endif // ENCODER_H
