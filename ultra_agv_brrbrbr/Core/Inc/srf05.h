#ifndef SRF05_H
#define SRF05_H

#include "stm32f1xx_hal.h"

uint32_t pulseIn(GPIO_TypeDef *gpio, uint16_t gpioPin, GPIO_PinState state, uint32_t timeoutUs);

// Đo khoảng cách cảm biến dưới SRF05 (cm)
float measureDistanceCM(void);

// Đo đồng thời khoảng cách thực tế (cm) của cả SRF05 (dưới) và SRF04 (trên) trên 1 xung Trigger
void measureDualDistanceCM(float *dist05Cm, float *dist04Cm);

#endif // SRF05_H
