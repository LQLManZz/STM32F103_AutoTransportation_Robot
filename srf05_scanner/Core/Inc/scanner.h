#ifndef SCANNER_H
#define SCANNER_H

#include "stm32f1xx_hal.h"

typedef enum
{
  SCAN_OBJECT_FOUND = 0,
  ERROR_NO_OBJECTIVE
} ScanStatus;

// Quét xuôi từ HOME, về HOME rồi quét ngược; thấy vật thì trả tọa độ (x, y) cm, gốc là cảm biến
ScanStatus scanForObject(TIM_HandleTypeDef *htim, float *x_cm, float *y_cm);

#endif // SCANNER_H
