#include "debug.h"
#include "oled.h"
#include <stdio.h>

extern UART_HandleTypeDef huart1;

// printf ra UART1
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void debugInit(void)
{
  oledInit();
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
