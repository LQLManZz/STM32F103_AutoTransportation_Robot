#ifndef SCANNER_H
#define SCANNER_H

typedef enum
{
  SCAN_OBJECT_FOUND = 0,
  ERROR_NO_OBJECTIVE
} ScanStatus;

// Bật PWM servo và đưa servo về HOME
void scanInit(void);

// Quét xuôi từ HOME, về HOME rồi quét ngược; thấy vật thì trả tọa độ (x, y) cm, gốc là cảm biến
ScanStatus scanForObject(float *xCm, float *yCm);

#endif // SCANNER_H
