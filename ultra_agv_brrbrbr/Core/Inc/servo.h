#ifndef SERVO_H
#define SERVO_H

#include "stm32f1xx_hal.h"

// Bật PWM servo và đưa về góc HOME
void servoInit(void);

// Đặt góc servo trực tiếp (0°..180°)
void setServoAngle(uint8_t angleDeg);

// Quay servo dần tới góc mục tiêu, giảm rung cảm biến
void moveServoSmooth(uint8_t targetDeg);

// Đọc góc servo hiện tại từ độ rộng xung PWM
uint8_t getServoAngle(void);

// Ngắt xung PWM servo quét để cắt dòng giữ khi tay gắp hoạt động
void servoStop(void);

#endif // SERVO_H
