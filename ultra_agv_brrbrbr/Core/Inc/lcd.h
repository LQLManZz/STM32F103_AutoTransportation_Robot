#ifndef LCD_H
#define LCD_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>

// Dò địa chỉ module PCF8574, khởi tạo LCD 16x2 HD44780 trên I2C2 và xóa màn hình
// addr7 nhận địa chỉ 7 bit tìm được (0: không có module trả lời), trả true nếu mọi lệnh khởi tạo gửi thành công
bool lcdInit(uint8_t *addr7);

// In một dòng (printf, tự đệm khoảng trắng đủ 16 ô), chỉ gửi lại khi chữ đổi
void lcdPrintLine(uint8_t row, const char *format, ...);

#endif // LCD_H
