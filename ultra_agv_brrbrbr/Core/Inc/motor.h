#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f1xx_hal.h"

typedef enum
{
  MOTOR_LEFT = 0,
  MOTOR_RIGHT
} MotorId;

typedef enum
{
  MOTOR_STOP = 0, // Thả trôi
  MOTOR_FORWARD,
  MOTOR_BACKWARD,
  MOTOR_BRAKE // Phanh cứng
} MotorDir;

// Bật PWM TIM3 cho 2 bánh và dừng xe
void motorInit(void);

// Đặt chiều quay và tốc độ (0 - 100 %) cho một bánh
void motorSetSpeed(MotorId motor, MotorDir dir, uint8_t speedPercent);

// Đặt trực tiếp giá trị duty raw (0 - 999) bỏ qua ngưỡng tối thiểu, dùng cho việc test / calibrate
void motorSetDutyRaw(MotorId motor, MotorDir dir, uint16_t duty);

void motorStopAll(void);
void motorBrakeAll(void);

// Lấy mức % PWM hiện tại (0 - 100) của một bánh để hiển thị debug
uint8_t motorGetSpeedPercent(MotorId motor);

// Lấy giá trị duty raw hiện tại (0 - 999) của một bánh
uint16_t motorGetDutyRaw(MotorId motor);

// Lấy trạng thái chiều quay hiện tại của một bánh
MotorDir motorGetDir(MotorId motor);

#endif // MOTOR_H
