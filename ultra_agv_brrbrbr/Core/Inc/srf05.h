#ifndef SRF05_H
#define SRF05_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>

// Đo tuần tự 1 mẫu khoảng cách thực tế (cm) của SRF05 (dưới) và SRF04 (trên) bằng 2 chân Trigger riêng
void measureDualDistanceCM(float *dist05Cm, float *dist04Cm);

// Đo 3 mẫu có lọc trung vị đồng thuận 2/3 cho SRF05 (dưới) và SRF04 (trên)
void measureDualDistanceMedianCM(float *dist05Cm, float *dist04Cm);

// Lọc 2/3 dùng sẵn mẫu đầu và đo bổ sung 2 mẫu để xác nhận khi có tín hiệu nghi ngờ
void measureDualDistanceConfirmCM(float firstD05Cm, float firstD04Cm, float *dist05Cm, float *dist04Cm);

// Trả mã trạng thái đo gần nhất của SRF05 (isSrf04=false) hoặc SRF04 (isSrf04=true): OK/OUT_RNG/STUCK_HI/NO_RISE
const char *srf05GetStateStr(bool isSrf04);

#endif // SRF05_H
