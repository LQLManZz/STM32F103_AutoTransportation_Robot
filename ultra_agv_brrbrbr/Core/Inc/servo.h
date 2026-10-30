#ifndef SERVO
#define SERVO

#include "stm32f1xx_hal.h"

typedef enum
{
  SERVO_FORWARD = 0,  
  SERVO_REVERSE       
} ServoDirection;

void setServoAngle(TIM_HandleTypeDef *htim, uint8_t angle);

void moveServoSmooth(TIM_HandleTypeDef *htim, uint8_t target);

uint8_t getServoAngle(TIM_HandleTypeDef *htim);

void setServoRange(TIM_HandleTypeDef *htim, uint8_t angleRange, ServoDirection direction);

#endif
