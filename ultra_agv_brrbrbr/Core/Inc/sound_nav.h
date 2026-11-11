/**
  ******************************************************************************
  * @file    sound_nav.h
  * @brief   Điều hướng xe đến tọa độ nguồn âm và quét servo tìm vật ở tốc độ thấp nhất
  ******************************************************************************
  */

#ifndef SOUND_NAV_H
#define SOUND_NAV_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>

// --- Cấu hình vận tốc thấp nhất có thể của xe ---
#define SOUND_SPIN_DUTY_MIN    350u    // Duty xoay tại chỗ thấp nhất (35%)
#define SOUND_DRIVE_DUTY_MIN   270u    // Duty tiến thẳng thấp nhất (27%)
#define SOUND_KICK_DUTY        450u    // Xung mồi kickstart phá ma sát nghỉ (45%)
#define SOUND_KICK_TIME_MS     40u     // Thời gian xung mồi (ms)

// --- Hiệu chỉnh thông số xoay và tiến ---
#define SOUND_SPIN_MS_PER_DEG  4.6f    // 4.6 ms/độ để xe dừng xoay chuẩn xác, không bị lố góc
#define SOUND_SPIN_OFFSET_MS   25.0f   // Bù trễ khởi động/phanh xoay (ms)
#define SOUND_DRIVE_MS_PER_CM  70.0f   // Thời gian chạy 1 cm ở Duty 270 (ms/cm)
#define SOUND_STOP_DIST_CM     3.0f    // Cự ly dừng cách vật mong muốn (cm)

// Khởi tạo điều hướng âm thanh, ngắt UART1 và servo scanner
void SoundNav_Init(void);

// Di chuyển tới tọa độ (x, y) với tốc độ thấp nhất có thể
void SoundNav_GoTo(float x, float y);

// Quét vi chỉnh bằng siêu âm kiểm tra xem mũi xe đã thẳng với vật thể chưa trước khi đi tới
bool SoundNav_VerifyAndAlignWithObject(float *outDistanceCm);

// Chạy đúng cự ly đã tính rồi dừng lại, sau khi dừng mới đo kiểm tra xem đã tới chưa
void SoundNav_DriveAndVerifyArrival(float targetDistanceCm);

// Xoay servo quét tìm vật thể khi phía trước không thấy vật
bool SoundNav_ScanAndApproach(void);

// Vòng lặp kiểm tra và thực thi: nhận tọa độ âm thanh hoặc quét servo khi không phát hiện ra
void SoundNav_Process(void);

#endif // SOUND_NAV_H
