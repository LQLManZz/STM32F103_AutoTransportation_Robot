#include "debug.h"
#include "oled.h"
#include "lcd_i2c.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c2;

// printf ra UART1
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void debugInit(void)
{
  oledInit();
  // Thử khởi tạo màn hình LCD 16x2 nếu có gắn vào I2C2
  if (LCD_Init(&hi2c2, LCD_I2C_ADDR_27) != HAL_OK)
  {
    LCD_Init(&hi2c2, LCD_I2C_ADDR_3F);
  }
}

void debugScan(float distanceCm, uint8_t angleDeg, float xCm, float yCm, bool found)
{
  printf("SCAN d=%.1f ang=%u x=%.1f y=%.1f %s\r\n", distanceCm, angleDeg, xCm, yCm,
         found ? "OBJ_FOUND" : "OBJ_NOTFND");

  if (!found) // không thấy vật thì tọa độ vô nghĩa
  {
    xCm = 0.0f;
    yCm = 0.0f;
  }
  oledPrintLine(0, "ang: %u.0", angleDeg);
  oledPrintLine(1, "X: %.1f", xCm);
  oledPrintLine(2, "Y: %.1f", yCm);
  oledPrintLine(3, found ? "OBJ_FOUND" : "OBJ_NOTFND");
}

void debugDrive(float distanceCm)
{
  printf("DRV d=%.1f\r\n", distanceCm);
  oledPrintLine(3, "DRV %.1fcm", distanceCm);
}

void debugDriveLost(void)
{
  printf("DRV_LOST\r\n");
  oledPrintLine(3, "DRV_LOST");
}

void debugMotorTest(const char *step)
{
  printf("TEST step=%s\r\n", step);
  oledPrintLine(0, "MOTOR TEST");
  oledPrintLine(3, "%s", step);
}

void debugMotorPwmTest(uint16_t duty, uint8_t percent, const char *status)
{
  printf("TEST_PWM Duty=%4u (PWM=%2u%%) - %s\r\n", duty, percent, status);

  // Hiển thị lên OLED SSD1306 (mỗi dòng tối đa 10 ký tự)
  oledPrintLine(0, (strncmp(status, "SPIN", 4) == 0) ? "TEST SPIN" : "TEST PWM");
  oledPrintLine(1, "DUTY:%4u", duty);
  oledPrintLine(2, "PWM : %2u%%", percent);
  oledPrintLine(3, "%-10s", status);

  // Hiển thị lên LCD 16x2 (nếu có gắn trên bus I2C)
  char lcd_buf[24];
  snprintf(lcd_buf, sizeof(lcd_buf), "PWM: %-11s", status);
  LCD_PrintLine(0, lcd_buf);
  snprintf(lcd_buf, sizeof(lcd_buf), "D:%-4u  PWM:%2u%%", duty, percent);
  LCD_PrintLine(1, lcd_buf);
}
