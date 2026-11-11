#ifndef DEBUG_H
#define DEBUG_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>

// Khởi tạo màn hình OLED (serial dùng sẵn UART1)
void debugInit(void);

// Báo một lần đo khi quét: khoảng cách, góc servo, tọa độ, có thấy vật không
void debugScan(float distanceCm, uint8_t angleDeg, float xCm, float yCm, bool found);

// Báo khoảng cách đo được khi đang chạy tới vật
void debugDrive(float distanceCm);

// Báo mất vật khi đang chạy tới vật
void debugDriveLost(void);

// Báo bước đang chạy của kịch bản test động cơ
void debugMotorTest(const char *step);

// Báo thông số khi test tìm ngưỡng PWM tối thiểu: duty (0-999), % PWM, trạng thái / bước
void debugMotorPwmTest(uint16_t duty, uint8_t percent, const char *status);

#endif // DEBUG_H
