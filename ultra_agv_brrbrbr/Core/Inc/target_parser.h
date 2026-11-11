/**
  ******************************************************************************
  * @file    target_parser.h
  * @brief   Bộ nhận ngắt và giải mã tọa độ (x, y) từ cảm biến âm thanh qua UART1
  ******************************************************************************
  */

#ifndef TARGET_PARSER_H
#define TARGET_PARSER_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>

#define UART_RX_BUFFER_SIZE 64

// Khởi tạo nhận ngắt UART1 (PA10 RX) để đọc tọa độ từ cảm biến âm thanh
void TargetParser_Init(void);

// Kiểm tra xem đã có gói tọa độ mới gửi về chưa
bool TargetParser_HasNewTarget(void);

// Đọc tọa độ x, y gần nhất (cm) và xóa cờ báo mới
bool TargetParser_GetTarget(float *out_x, float *out_y);

#endif // TARGET_PARSER_H
