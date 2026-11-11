#ifndef LCD_I2C_H
#define LCD_I2C_H

#include "main.h"

#define LCD_I2C_ADDR_27 (0x27u << 1)
#define LCD_I2C_ADDR_3F (0x3Fu << 1)
#define LCD_COLS 16u
#define LCD_ROWS 2u

HAL_StatusTypeDef LCD_Init(I2C_HandleTypeDef *hi2c, uint8_t addr);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_Print(const char *str);
void LCD_PrintLine(uint8_t row, const char *str);
void LCD_Backlight(uint8_t on);

#endif // LCD_I2C_H
