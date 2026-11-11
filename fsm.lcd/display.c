#include "display.h"
#include "lcd_i2c.h"
#include "fsm.h"
#include "robot_if.h"
#include <stdio.h>

void Display_Init(void)
{
  LCD_Clear();
  LCD_PrintLine(0, "ROBOT 2WD");
  LCD_PrintLine(1, "Khoi dong...");
  HAL_Delay(1000);
  LCD_Clear();
}

void Display_Update(void)
{
  static uint32_t last_tick = 0;
  uint32_t now = HAL_GetTick();
  char line[24];

  if (now - last_tick < DISPLAY_REFRESH_MS)
    return;
  last_tick = now;

  snprintf(line, sizeof(line), "ST: %s", FSM_GetStateName(FSM_GetState()));
  LCD_PrintLine(0, line);
  snprintf(line, sizeof(line), "Dist:%3u cm %c",
           (unsigned)Sensor_GetDistanceCm(),
           Sensor_ObjectFound() ? '*' : ' ');
  LCD_PrintLine(1, line);
}
