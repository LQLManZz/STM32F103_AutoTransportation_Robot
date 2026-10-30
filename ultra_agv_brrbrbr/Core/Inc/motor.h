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

void motorStopAll(void);
void motorBrakeAll(void);

#endif // MOTOR_H
