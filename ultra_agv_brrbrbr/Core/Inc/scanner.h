#ifndef SCANNER_H
#define SCANNER_H

#include <stdint.h>

typedef enum
{
  SCAN_OBJECT_FOUND = 0,
  ERROR_NO_OBJECTIVE
} ScanStatus;

// Bật PWM servo và đưa servo về HOME
void scanInit(void);

// Quét xuôi từ HOME, về HOME rồi quét ngược; thấy vật thì trả tọa độ cực: cự ly (cm) và góc servo (độ), gốc là cảm biến
ScanStatus scanForObject(float *distanceCm, uint8_t *angleDeg);

#endif // SCANNER_H
