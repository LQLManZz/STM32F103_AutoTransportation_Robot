#ifndef DELAY
#define DELAY

#include "stm32f1xx_hal.h"

// Bật bộ đếm chu kỳ DWT cho delayUs
void delayInit(void);

void delayUs(uint32_t us);

#endif
