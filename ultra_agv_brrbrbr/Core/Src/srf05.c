#include "srf05.h"
#include "project_config.h"
#include "delay.h"

typedef enum
{
  ECHO_WAIT_LOW = 0,
  ECHO_WAIT_RISE,
  ECHO_WAIT_FALL,
  ECHO_DONE
} EchoState;

// Chuyển thời gian xung (us) sang khoảng cách thực tế (cm)
static float durationToCm(uint32_t durationUs)
{
  if (durationUs == 0UL) return ECHO_ERROR_TIMEOUT;
  if (durationUs < ECHO_MIN_VALID_US) return ECHO_ERROR_TOO_CLOSE;
  return ((float)durationUs * SOUND_SPEED_CM_US) / 2.0f;
}

// Cập nhật máy trạng thái đo độ rộng xung mức cao cho 1 chân Echo
static void updateEchoChannel(GPIO_TypeDef *port, uint16_t pin, uint32_t nowTick, uint32_t usTicks,
                              EchoState *state, uint32_t *riseTick, uint32_t *durationUs)
{
  GPIO_PinState level = HAL_GPIO_ReadPin(port, pin);

  if (*state == ECHO_WAIT_LOW)
  {
    if (level == GPIO_PIN_RESET) *state = ECHO_WAIT_RISE;
  }
  else if (*state == ECHO_WAIT_RISE)
  {
    if (level == GPIO_PIN_SET)
    {
      *riseTick = nowTick;
      *state = ECHO_WAIT_FALL;
    }
  }
  else if (*state == ECHO_WAIT_FALL)
  {
    if (level == GPIO_PIN_RESET)
    {
      uint32_t pulseUs = (nowTick - *riseTick) / usTicks;
      if (pulseUs >= ECHO_MIN_NOISE_US)
      {
        *durationUs = pulseUs;
        *state = ECHO_DONE;
      }
      else
      {
        *state = ECHO_WAIT_RISE;
      }
    }
  }
}

// Đo độ dài mức "state" trên chân GPIO (us), có timeout; bỏ qua xung nhiễu < 50 us
uint32_t pulseIn(GPIO_TypeDef *gpio, uint16_t gpioPin, GPIO_PinState state, uint32_t timeoutUs)
{
  uint32_t usTicks = SystemCoreClock / 1000000UL;
  uint32_t timeoutTicks = timeoutUs * usTicks;
  uint32_t startWait = DWT->CYCCNT;

  while ((DWT->CYCCNT - startWait) < timeoutTicks)
  {
    while (HAL_GPIO_ReadPin(gpio, gpioPin) == state)
      if ((DWT->CYCCNT - startWait) > timeoutTicks) return 0;
    while (HAL_GPIO_ReadPin(gpio, gpioPin) != state)
      if ((DWT->CYCCNT - startWait) > timeoutTicks) return 0;

    uint32_t pulseStart = DWT->CYCCNT;
    while (HAL_GPIO_ReadPin(gpio, gpioPin) == state)
      if ((DWT->CYCCNT - pulseStart) > timeoutTicks) return 0;

    uint32_t pulseUs = (DWT->CYCCNT - pulseStart) / usTicks;
    if (pulseUs >= ECHO_MIN_NOISE_US)
      return pulseUs;
  }
  return 0;
}

void measureDualDistanceCM(float *dist05Cm, float *dist04Cm)
{
  HAL_GPIO_WritePin(TRIGGER_PORT, TRIGGER_PIN, GPIO_PIN_RESET);
  delayUs(TRIGGER_RESET_US);
  HAL_GPIO_WritePin(TRIGGER_PORT, TRIGGER_PIN, GPIO_PIN_SET);
  delayUs(TRIGGER_PULSE_US);
  HAL_GPIO_WritePin(TRIGGER_PORT, TRIGGER_PIN, GPIO_PIN_RESET);

  uint32_t usTicks = SystemCoreClock / 1000000UL;
  uint32_t timeoutTicks = MAX_ECHO_TIMEOUT_US * usTicks;
  uint32_t startWait = DWT->CYCCNT;

  EchoState state05 = ECHO_WAIT_LOW;
  EchoState state04 = ECHO_WAIT_LOW;
  uint32_t rise05 = 0UL, rise04 = 0UL;
  uint32_t dur05Us = 0UL, dur04Us = 0UL;

  while ((DWT->CYCCNT - startWait) < timeoutTicks && (state05 != ECHO_DONE || state04 != ECHO_DONE))
  {
    uint32_t now = DWT->CYCCNT;
    if (state05 != ECHO_DONE)
      updateEchoChannel(ECHO_PORT, ECHO_PIN, now, usTicks, &state05, &rise05, &dur05Us);
    if (state04 != ECHO_DONE)
      updateEchoChannel(ECHO_SRF04_PORT, ECHO_SRF04_PIN, now, usTicks, &state04, &rise04, &dur04Us);
  }

  if (dist05Cm) *dist05Cm = durationToCm(dur05Us);
  if (dist04Cm) *dist04Cm = durationToCm(dur04Us);
}

float measureDistanceCM(void)
{
  float d05Cm = ECHO_ERROR_TIMEOUT;
  measureDualDistanceCM(&d05Cm, NULL);
  return d05Cm;
}
