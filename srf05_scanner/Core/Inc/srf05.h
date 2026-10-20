#ifndef SRF05_H
#define SRF05_H

#include "stm32f1xx_hal.h"

uint32_t pulseIn(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState state, uint32_t timeout_us);

float measureDistanceCM(void);

#endif // SRF05_H
