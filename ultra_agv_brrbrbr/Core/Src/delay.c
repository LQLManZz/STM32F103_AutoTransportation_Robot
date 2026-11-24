#include "delay.h"

void delayInit(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;
}

void delayUs(uint32_t us)
{
  // ST-Link ngắt kết nối có thể xóa TRCENA làm CYCCNT đứng yên, bật lại để vòng chờ không treo
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (SystemCoreClock / 1000000UL);

  while ((DWT->CYCCNT - start) < ticks)
    ;
}
