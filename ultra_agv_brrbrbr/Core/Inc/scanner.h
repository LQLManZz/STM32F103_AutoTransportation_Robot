#ifndef SCANNER_H
#define SCANNER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  SCAN_OBJECT_FOUND = 0,
  SCAN_WALL_FOUND,
  ERROR_NO_OBJECTIVE
} ScanStatus;

typedef enum
{
  TARGET_NONE = 0, // Không có vật trong tầm quét (OBJ_NOTFND)
  TARGET_OBJECT,   // Chỉ cảm biến dưới SRF05 thấy -> Vật cần gắp (OBJ_FOUND)
  TARGET_OBSTACLE  // Cả 2 cảm biến cùng thấy -> Vật cản cao >= 11cm (OBSTACLE)
} ScanTargetType;

typedef enum
{
  SCAN_SIDE_BOTH = 0, // Quét từ trái sang phải (135°..45°)
  SCAN_SIDE_LEFT,     // Chỉ quét nửa trái từ ngoài vào chính diện (135°..90°), dùng khi tường ở bên phải
  SCAN_SIDE_RIGHT     // Chỉ quét nửa phải từ chính diện sang phải (90°..45°), dùng khi tường ở bên trái
} ScanSide;

// Bật PWM servo và đưa servo về HOME
void scanInit(void);

// Phân loại mục tiêu từ khoảng cách thực tế của SRF05 (dưới) và SRF04 (trên, lùi 7cm)
ScanTargetType scanClassifyTarget(float d05RawCm, float d04RawCm);

// Quét toàn cung từ trái sang phải (135°..45°), lọc mép tường và xác minh cụm vật gần nhất
ScanStatus scanForObject(float *distanceCm, uint8_t *angleDeg);

// Quét một phía chỉ định theo chiều từ trái sang phải (tránh quét vào mặt tường đang men theo)
ScanStatus scanSideArc(ScanSide side, float *distanceCm, uint8_t *angleDeg);

// Quét góc hẹp từ trái sang phải quanh HOME (120°..60°) để vi chỉnh hướng mũi xe vào vật gần nhất
ScanStatus scanFine(float *distanceCm, uint8_t *angleDeg);

// Lấy gợi ý không gian từ lần quét toàn cung gần nhất (phía trước thoáng, có tường trái/phải)
void scanGetLastCorridorHint(bool *frontClear, bool *leftWall, bool *rightWall);

// Quét 1 lượt liên tục (giữ nguyên phân loại tường ban đầu): thấy vật thì quét hết vật rồi dừng tính trung bình, không quét lại lần 2
ScanStatus scanSimpleForObject(float *distanceCm, uint8_t *angleDeg);

#endif // SCANNER_H
