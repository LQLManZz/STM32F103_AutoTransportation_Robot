#include "srf05.h"
#include "project_config.h"
#include "delay.h"

/* Đo độ dài mức "state" trên chân GPIO (us), có timeout */
uint32_t pulseIn(GPIO_TypeDef *gpio, uint16_t gpioPin, GPIO_PinState state, uint32_t timeoutUs)
{
  uint32_t timeoutTicks = timeoutUs * (SystemCoreClock / 1000000UL);
  uint32_t minTicks = 50UL * (SystemCoreClock / 1000000UL); // Bỏ qua xung < 50 us (nhiễu)
  uint32_t startWait = DWT->CYCCNT;

  while ((DWT->CYCCNT - startWait) < timeoutTicks)
  {
    // Chờ chân chuyển sang trạng thái khác "state" (bắt đầu từ trạng thái hiện tại)
    while (HAL_GPIO_ReadPin(gpio, gpioPin) == state)
    {
      if ((DWT->CYCCNT - startWait) > timeoutTicks)
        return 0;
    }
    // Chờ chân lên đúng "state": bắt đầu xung cần đo
    while (HAL_GPIO_ReadPin(gpio, gpioPin) != state)
    {
      if ((DWT->CYCCNT - startWait) > timeoutTicks)
        return 0;
    }
    // Đo thời gian chân giữ "state"
    uint32_t pulseStart = DWT->CYCCNT;
    while (HAL_GPIO_ReadPin(gpio, gpioPin) == state)
    {
      if ((DWT->CYCCNT - pulseStart) > timeoutTicks)
        return 0;
    }
    uint32_t pulseTicks = DWT->CYCCNT - pulseStart;
    if (pulseTicks >= minTicks)
    {
      return pulseTicks / (SystemCoreClock / 1000000UL); // Đổi chu kỳ CPU sang us
    }
  }
  return 0;
}

/* Gửi xung Trigger 10 us rồi đo thời gian echo để suy ra khoảng cách */
float measureDistanceCM(void)
{
  // Đưa Trigger xuống mức thấp trước để tạo cạnh lên sạch
  HAL_GPIO_WritePin(TRIGGER_PORT, TRIGGER_PIN, GPIO_PIN_RESET);
  delayUs(4);

  // Xung Trigger dương 10 us: SRF05 yêu cầu tối thiểu 10 us
  HAL_GPIO_WritePin(TRIGGER_PORT, TRIGGER_PIN, GPIO_PIN_SET);
  delayUs(10);
  HAL_GPIO_WritePin(TRIGGER_PORT, TRIGGER_PIN, GPIO_PIN_RESET);

  // Echo lên mức cao trong thời gian sóng âm đi và về
  uint32_t duration = pulseIn(ECHO_PORT, ECHO_PIN, GPIO_PIN_SET, MAX_ECHO_TIMEOUT_US);

  if (duration == 0)
  {
    return -1.0f; // Không có echo (timeout hoặc không có vật cản)
  }
  if (duration < 116)
  {
    return -2.0f; // 116 us ~ 2 cm: quá gần, dưới ngưỡng đo tin cậy
  }

  // Khoảng cách = thời gian * tốc độ âm / 2 (chia 2 vì sóng đi rồi về)
  return ((float)duration * SOUND_SPEED_CM_US) / 2.0f;
}
