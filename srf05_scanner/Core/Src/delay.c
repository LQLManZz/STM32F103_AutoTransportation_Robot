#include "delay.h"
#include "project_config.h"

/* Delay micro giây bằng cách chờ đủ số chu kỳ CPU */
void delay_us(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (SystemCoreClock / 1000000UL); // Số chu kỳ CPU tương ứng us
  while ((DWT->CYCCNT - start) < ticks)
    ;
}