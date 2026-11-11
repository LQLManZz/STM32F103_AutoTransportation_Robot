#include "lcd_i2c.h"

#define BIT_RS 0x01u
#define BIT_EN 0x04u
#define BIT_BL 0x08u

static I2C_HandleTypeDef *lcd_i2c;
static uint8_t lcd_addr;
static uint8_t lcd_bl = BIT_BL;

static void lcd_send_nibble(uint8_t nibble)
{
  if (!lcd_i2c) return;
  uint8_t d = (uint8_t)((nibble << 4) & 0xF0u) | lcd_bl;
  uint8_t buf[2] = {(uint8_t)(d | BIT_EN), d};
  HAL_I2C_Master_Transmit(lcd_i2c, lcd_addr, buf, 2, 20);
}

static void lcd_send(uint8_t value, uint8_t rs)
{
  if (!lcd_i2c) return;
  uint8_t hi = value & 0xF0u;
  uint8_t lo = (uint8_t)(value << 4) & 0xF0u;
  uint8_t ctl = lcd_bl | (rs ? BIT_RS : 0u);
  uint8_t buf[4] = {(uint8_t)(hi | ctl | BIT_EN), (uint8_t)(hi | ctl),
                    (uint8_t)(lo | ctl | BIT_EN), (uint8_t)(lo | ctl)};
  HAL_I2C_Master_Transmit(lcd_i2c, lcd_addr, buf, 4, 20);
}

HAL_StatusTypeDef LCD_Init(I2C_HandleTypeDef *hi2c, uint8_t addr)
{
  if (!hi2c || HAL_I2C_IsDeviceReady(hi2c, addr, 2, 20) != HAL_OK)
  {
    lcd_i2c = NULL;
    return HAL_ERROR;
  }
  lcd_i2c = hi2c;
  lcd_addr = addr;

  HAL_Delay(50);
  lcd_send_nibble(0x03); HAL_Delay(5);
  lcd_send_nibble(0x03); HAL_Delay(1);
  lcd_send_nibble(0x03); HAL_Delay(1);
  lcd_send_nibble(0x02); HAL_Delay(1);

  lcd_send(0x28, 0);
  lcd_send(0x0C, 0);
  lcd_send(0x06, 0);
  LCD_Clear();
  return HAL_OK;
}

void LCD_Clear(void)
{
  lcd_send(0x01, 0);
  HAL_Delay(2);
}

void LCD_SetCursor(uint8_t row, uint8_t col)
{
  if (row >= LCD_ROWS) row = LCD_ROWS - 1u;
  if (col >= LCD_COLS) col = LCD_COLS - 1u;
  lcd_send((uint8_t)(0x80u | ((row ? 0x40u : 0x00u) + col)), 0);
}

void LCD_Print(const char *str)
{
  while (*str)
    lcd_send((uint8_t)*str++, 1);
}

void LCD_PrintLine(uint8_t row, const char *str)
{
  LCD_SetCursor(row, 0);
  uint8_t i = 0;
  for (; i < LCD_COLS && str[i]; i++)
    lcd_send((uint8_t)str[i], 1);
  for (; i < LCD_COLS; i++)
    lcd_send(' ', 1);
}

void LCD_Backlight(uint8_t on)
{
  if (!lcd_i2c) return;
  lcd_bl = on ? BIT_BL : 0u;
  HAL_I2C_Master_Transmit(lcd_i2c, lcd_addr, &lcd_bl, 1, 20);
}
