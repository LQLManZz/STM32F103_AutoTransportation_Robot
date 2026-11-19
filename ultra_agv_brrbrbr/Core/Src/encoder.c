#include "encoder.h"
#include "project_config.h"
#include "delay.h"

static volatile GPIO_PinState lastPinState[2] = {GPIO_PIN_RESET, GPIO_PIN_RESET};
static volatile uint32_t lastCyccnt[2] = {0, 0};
static volatile uint32_t lastSameEdgeCyccnt[2][2] = {{0, 0}, {0, 0}};
static volatile uint32_t lastTickMs[2] = {0, 0};
static volatile uint16_t wheelRpm[2] = {0, 0};
static volatile uint32_t wheelPulses[2] = {0, 0};

void encoderInit(void)
{
  for (uint8_t i = 0; i < 2u; i++)
  {
    lastCyccnt[i] = 0;
    lastSameEdgeCyccnt[i][0] = 0;
    lastSameEdgeCyccnt[i][1] = 0;
    lastTickMs[i] = 0;
    wheelRpm[i] = 0;
    wheelPulses[i] = 0;
  }
  lastPinState[MOTOR_LEFT] = HAL_GPIO_ReadPin(GPIOB, ENC_LEFT_PIN);
  lastPinState[MOTOR_RIGHT] = HAL_GPIO_ReadPin(GPIOB, ENC_RIGHT_PIN);
}

void encoderResetPulses(void)
{
  encoderInit();
}

uint32_t encoderGetPulses(MotorId motor)
{
  return wheelPulses[motor];
}

uint16_t encoderGetRpm(MotorId motor)
{
  uint32_t tick = lastTickMs[motor];
  uint32_t cyc = lastCyccnt[motor];
  uint16_t rpm = wheelRpm[motor];
  if (tick == 0u || (HAL_GetTick() - tick) > ENC_STOP_TIMEOUT_MS)
    return 0;

  uint32_t nowCyc = DWT->CYCCNT;
  if (cyc != lastCyccnt[motor])
    return wheelRpm[motor];

  uint32_t elapsedUs = (nowCyc - cyc) / (SystemCoreClock / 1000000UL);
  if (elapsedUs > ENC_DEBOUNCE_US)
  {
    uint32_t maxRpm = 60000000UL / ((uint32_t)ENC_PULSES_PER_REV * elapsedUs);
    if (maxRpm < rpm)
      return (uint16_t)maxRpm;
  }
  return rpm;
}

// Lọc nhiễu cả 2 sườn (lên/xuống); tính RPM theo chu kỳ cùng sườn để tránh sai số khi lỗ đĩa không đều 50/50
void HAL_GPIO_EXTI_Callback(uint16_t gpioPin)
{
  MotorId id;
  if (gpioPin == ENC_LEFT_PIN)
    id = MOTOR_LEFT;
  else if (gpioPin == ENC_RIGHT_PIN)
    id = MOTOR_RIGHT;
  else
    return;

  // Chờ 20us rồi kiểm tra mức logic thực tế có thật sự đổi trạng thái so với sườn trước không
  delayUs(20);
  GPIO_PinState level = HAL_GPIO_ReadPin(GPIOB, gpioPin);
  if (level == lastPinState[id])
    return;

  uint32_t nowCyc = DWT->CYCCNT;
  uint32_t nowMs = HAL_GetTick();
  uint8_t edgeIdx = (level == GPIO_PIN_SET) ? 1u : 0u;

  // Sườn đầu tiên sau khi bánh dừng: tăng xung nhưng chỉ lưu mốc để tính RPM từ sườn thứ 2
  if (lastTickMs[id] == 0u || (nowMs - lastTickMs[id]) > ENC_STOP_TIMEOUT_MS)
  {
    lastPinState[id] = level;
    wheelRpm[id] = 0;
    wheelPulses[id]++;
    lastCyccnt[id] = nowCyc;
    lastSameEdgeCyccnt[id][edgeIdx] = nowCyc;
    lastSameEdgeCyccnt[id][1u - edgeIdx] = 0u;
    lastTickMs[id] = nowMs;
    return;
  }

  uint32_t usTicks = SystemCoreClock / 1000000UL;
  uint32_t diffUs = (nowCyc - lastCyccnt[id]) / usTicks;
  if (diffUs < ENC_DEBOUNCE_US)
    return;

  lastPinState[id] = level;
  wheelPulses[id]++;
  lastCyccnt[id] = nowCyc;
  lastTickMs[id] = nowMs;

  uint16_t newRpm;
  if (lastSameEdgeCyccnt[id][edgeIdx] != 0u)
  {
    uint32_t slotUs = (nowCyc - lastSameEdgeCyccnt[id][edgeIdx]) / usTicks;
    newRpm = (uint16_t)(60000000UL / ((uint32_t)ENC_SLOTS_PER_REV * slotUs));
  }
  else
  {
    newRpm = (uint16_t)(60000000UL / ((uint32_t)ENC_PULSES_PER_REV * diffUs));
  }
  lastSameEdgeCyccnt[id][edgeIdx] = nowCyc;
  wheelRpm[id] = (wheelRpm[id] == 0u) ? newRpm : (uint16_t)((wheelRpm[id] + newRpm) / 2u);
}
