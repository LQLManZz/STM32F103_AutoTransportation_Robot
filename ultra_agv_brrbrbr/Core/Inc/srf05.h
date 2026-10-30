#ifndef SRF05_H
#define SRF05_H

#include "stm32f1xx_hal.h"

uint32_t pulseIn(GPIO_TypeDef *gpio, uint16_t gpioPin, GPIO_PinState state, uint32_t timeoutUs);

float measureDistanceCM(void);

#endif // SRF05_H
