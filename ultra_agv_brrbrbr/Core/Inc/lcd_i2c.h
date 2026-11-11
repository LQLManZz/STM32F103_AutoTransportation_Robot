/**
 * lcd_i2c.h - Driver LCD 16x2 qua module I2C (PCF8574) cho STM32F103 (HAL)
 * Phu trach: Nguyen Quang Chinh (FSM & LCD)
 *
 * Noi day: SCL -> PB6, SDA -> PB7 (I2C1)
 */
#ifndef LCD_I2C_H
#define LCD_I2C_H

#include "main.h"          /* keo theo stm32f1xx_hal.h */

/* Dia chi I2C (dich trai 1 bit cho HAL). Thu 0x27 truoc, neu khong duoc thi 0x3F */
#define LCD_I2C_ADDR_27     (0x27 << 1)
#define LCD_I2C_ADDR_3F     (0x3F << 1)

#define LCD_COLS            16
#define LCD_ROWS            2

/* Khoi tao LCD. Tra ve HAL_OK neu tim thay LCD tren bus I2C */
HAL_StatusTypeDef LCD_Init(I2C_HandleTypeDef *hi2c, uint8_t addr);

void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_Print(const char *str);

/* Ghi de nguyen 1 dong (tu dong chen khoang trang cho du 16 ky tu, khong can Clear -> khong nhap nhay) */
void LCD_PrintLine(uint8_t row, const char *str);

void LCD_Backlight(uint8_t on);

#endif /* LCD_I2C_H */
