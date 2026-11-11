#ifndef SCANNER_H
#define SCANNER_H

#include <stdint.h>

typedef enum
{
  SCAN_OBJECT_FOUND = 0,
  ERROR_NO_OBJECTIVE
} ScanStatus;

typedef enum
{
  TARGET_NONE = 0, // Không có vật trong tầm quét (OBJ_NOTFND)
  TARGET_OBJECT,   // Chỉ cảm biến dưới SRF05 thấy -> Vật cần gắp (OBJ_FOUND)
  TARGET_OBSTACLE  // Cả 2 cảm biến cùng thấy -> Vật cản cao >= 11cm (OBSTACLE)
} ScanTargetType;

// Bật PWM servo và đưa servo về HOME
void scanInit(void);

// Phân loại mục tiêu từ khoảng cách thực tế của SRF05 (dưới) và SRF04 (trên, lùi 7cm)
ScanTargetType scanClassifyTarget(float d05RawCm, float d04RawCm);

// Quét xuôi từ HOME, về HOME rồi quét ngược; thấy vật cần gắp thì trả tọa độ cực: cự ly (cm) và góc servo (độ)
ScanStatus scanForObject(float *distanceCm, uint8_t *angleDeg);

// Quét góc hẹp quanh HOME (70°..110°) để vi chỉnh hướng mũi xe vào vật cần gắp gần nhất
ScanStatus scanFine(float *distanceCm, uint8_t *angleDeg);

#endif // SCANNER_H
