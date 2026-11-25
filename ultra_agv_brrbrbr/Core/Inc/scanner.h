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
  TARGET_OBSTACLE  // Cả 2 cảm biến cùng thấy cùng 1 mặt (chênh <= SCAN_OBSTACLE_TOLERANCE_CM) -> tường/vật cản cao (OBSTACLE)
} ScanTargetType;

typedef enum
{
  SCAN_SIDE_BOTH = 0, // Quét từ trái sang phải (135°..45°)
  SCAN_SIDE_LEFT,     // Chỉ quét nửa trái từ ngoài vào chính diện (135°..90°), dùng khi tường ở bên phải
  SCAN_SIDE_RIGHT     // Chỉ quét nửa phải từ chính diện sang phải (90°..45°), dùng khi tường ở bên trái
} ScanSide;

typedef struct
{
  uint8_t startDeg;
  uint8_t endDeg;
  float startDistCm;
  float endDistCm;
} ScanWallSegLocal;

typedef struct
{
  bool isConfirmedWideWall;
  bool isWallBump;
  bool hasSuspect;
  uint8_t suspectAngleDeg;
  float suspectDistCm;
} ScanExtraInfo;

// Bật PWM servo và đưa servo về HOME
void scanInit(void);

// Phân loại mục tiêu từ khoảng cách thực tế của SRF05 dưới và SRF05 trên (lùi 7cm)
ScanTargetType scanClassifyTarget(float d05RawCm, float d04RawCm);

// Quét toàn cung từ trái sang phải (135°..45°), lọc mép tường và xác minh cụm vật gần nhất
ScanStatus scanForObject(float *distanceCm, uint8_t *angleDeg);

// Quét một phía chỉ định theo chiều từ trái sang phải (tránh quét vào mặt tường đang men theo)
ScanStatus scanSideArc(ScanSide side, float *distanceCm, uint8_t *angleDeg);

// Quét góc hẹp từ trái sang phải quanh HOME (120°..60°) để vi chỉnh hướng mũi xe vào vật gần nhất
ScanStatus scanFine(float *distanceCm, uint8_t *angleDeg);

// Quét hẹp quanh HOME chỉ lấy góc + cự ly cụm gần nhất (không phân loại lại, vật đã chốt từ lượt quét toàn cung)
bool scanFineBearing(float *distanceCm, uint8_t *angleDeg);

// Lấy gợi ý không gian từ lần quét toàn cung gần nhất (phía trước thoáng, có tường trái/phải)
void scanGetLastCorridorHint(bool *frontClear, bool *leftWall, bool *rightWall);

// Lấy thông tin phân loại mở rộng (tường rộng, vật sát tường, cụm nghi ngờ xa) của lượt quét gần nhất
void scanGetLastExtraInfo(ScanExtraInfo *outInfo);

// Lấy danh sách đoạn tường cục bộ trích xuất từ lượt quét gần nhất
uint8_t scanGetLastWallSegments(ScanWallSegLocal *outSegs, uint8_t maxSegs);

#endif // SCANNER_H
