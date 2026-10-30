#ifndef OLED_H
#define OLED_H

#include "stm32f1xx_hal.h"

// Khởi tạo SSD1306 trên I2C2 và xóa màn hình
void oledInit(void);

// In một dòng (printf), chỉ vẽ lại khi chữ đổi
void oledPrintLine(uint8_t line, const char *format, ...);

#endif // OLED_H
