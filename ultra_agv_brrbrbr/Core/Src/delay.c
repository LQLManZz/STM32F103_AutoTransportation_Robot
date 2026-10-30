#include "delay.h"
#include "project_config.h"

void delayInit(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;
}

/* Delay micro giây bằng cách chờ đủ số chu kỳ CPU */
void delayUs(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (SystemCoreClock / 1000000UL);

  while ((DWT->CYCCNT - start) < ticks)
    ;
}
