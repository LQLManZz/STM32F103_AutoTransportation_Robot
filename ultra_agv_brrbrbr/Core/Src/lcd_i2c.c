/**
 * lcd_i2c.c - Driver LCD 16x2 qua PCF8574, che do 4-bit
 *
 * Anh xa bit cua PCF8574 (module pho bien):
 *   P0=RS  P1=RW  P2=EN  P3=Backlight  P4..P7 = D4..D7
 */
#include "lcd_i2c.h"

#define BIT_RS   0x01
#define BIT_RW   0x02
#define BIT_EN   0x04
#define BIT_BL   0x08

static I2C_HandleTypeDef *lcd_i2c;
static uint8_t lcd_addr;
static uint8_t lcd_bl = BIT_BL;

/* Gui 1 nibble (dung luc khoi tao) */
static void lcd_send_nibble(uint8_t nibble)
{
    uint8_t d = (uint8_t)((nibble << 4) & 0xF0) | lcd_bl;
    uint8_t buf[2] = { (uint8_t)(d | BIT_EN), d };
    HAL_I2C_Master_Transmit(lcd_i2c, lcd_addr, buf, 2, 20);
}

/* Gui 1 byte (lenh rs=0, du lieu rs=1) = 2 nibble, 4 byte I2C trong 1 lan truyen */
static void lcd_send(uint8_t value, uint8_t rs)
{
    uint8_t hi  = value & 0xF0;
    uint8_t lo  = (uint8_t)(value << 4) & 0xF0;
    uint8_t ctl = lcd_bl | (rs ? BIT_RS : 0);
    uint8_t buf[4] = {
        (uint8_t)(hi | ctl | BIT_EN), (uint8_t)(hi | ctl),
        (uint8_t)(lo | ctl | BIT_EN), (uint8_t)(lo | ctl)
    };
    HAL_I2C_Master_Transmit(lcd_i2c, lcd_addr, buf, 4, 20);
}

HAL_StatusTypeDef LCD_Init(I2C_HandleTypeDef *hi2c, uint8_t addr)
{
    lcd_i2c  = hi2c;
    lcd_addr = addr;

    if (HAL_I2C_IsDeviceReady(lcd_i2c, lcd_addr, 3, 50) != HAL_OK)
        return HAL_ERROR;           /* Khong thay LCD: kiem tra day / dia chi */

    HAL_Delay(50);                  /* cho LCD len nguon */

    /* Quy trinh khoi tao 4-bit theo datasheet HD44780 */
    lcd_send_nibble(0x03); HAL_Delay(5);
    lcd_send_nibble(0x03); HAL_Delay(1);
    lcd_send_nibble(0x03); HAL_Delay(1);
    lcd_send_nibble(0x02); HAL_Delay(1);   /* chuyen sang 4-bit */

    lcd_send(0x28, 0);   /* 4-bit, 2 dong, font 5x8 */
    lcd_send(0x0C, 0);   /* bat hien thi, tat con tro */
    lcd_send(0x06, 0);   /* con tro tu tang sang phai */
    LCD_Clear();
    return HAL_OK;
}

void LCD_Clear(void)
{
    lcd_send(0x01, 0);
    HAL_Delay(2);                   /* lenh Clear can ~1.5 ms */
}

void LCD_SetCursor(uint8_t row, uint8_t col)
{
    static const uint8_t offs[LCD_ROWS] = { 0x00, 0x40 };
    if (row >= LCD_ROWS) row = LCD_ROWS - 1;
    if (col >= LCD_COLS) col = LCD_COLS - 1;
    lcd_send((uint8_t)(0x80 | (offs[row] + col)), 0);
}

void LCD_Print(const char *str)
{
    while (*str) lcd_send((uint8_t)*str++, 1);
}

void LCD_PrintLine(uint8_t row, const char *str)
{
    LCD_SetCursor(row, 0);
    uint8_t i = 0;
    while (i < LCD_COLS && str[i]) { lcd_send((uint8_t)str[i], 1); i++; }
    while (i < LCD_COLS)           { lcd_send(' ', 1); i++; }
}

void LCD_Backlight(uint8_t on)
{
    lcd_bl = on ? BIT_BL : 0;
    uint8_t d = lcd_bl;
    HAL_I2C_Master_Transmit(lcd_i2c, lcd_addr, &d, 1, 20);
}
