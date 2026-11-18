#include "encoder.h"
#include "project_config.h"
#include "delay.h"

static volatile uint32_t lastCyccnt[2] = {0, 0};
static volatile uint32_t lastTickMs[2] = {0, 0};
static volatile uint16_t wheelRpm[2] = {0, 0};
static volatile uint32_t wheelPulses[2] = {0, 0};

void encoderInit(void)
{
  for (uint8_t i = 0; i < 2u; i++)
  {
    lastCyccnt[i] = 0;
    lastTickMs[i] = 0;
    wheelRpm[i] = 0;
    wheelPulses[i] = 0;
  }
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

// shortcut: lọc trung bình 2 xung liên tiếp, khi chạy tốc độ thấp có thể cần lọc thêm nếu đĩa lỗ không đều
void HAL_GPIO_EXTI_Callback(uint16_t gpioPin)
{
  MotorId id;
  if (gpioPin == ENC_LEFT_PIN)
    id = MOTOR_LEFT;
  else if (gpioPin == ENC_RIGHT_PIN)
    id = MOTOR_RIGHT;
  else
    return;

  // Chờ 20us rồi kiểm tra chân vẫn ở mức HIGH để loại bỏ nhiễu rung ở sườn xuống LM393
  delayUs(20);
  if (HAL_GPIO_ReadPin(GPIOB, gpioPin) != GPIO_PIN_SET)
    return;

  uint32_t nowCyc = DWT->CYCCNT;
  uint32_t nowMs = HAL_GetTick();

  // Xung đầu tiên sau khi bánh dừng: tăng xung nhưng chỉ lưu mốc để tính RPM từ xung thứ 2
  if (lastTickMs[id] == 0u || (nowMs - lastTickMs[id]) > ENC_STOP_TIMEOUT_MS)
  {
    wheelRpm[id] = 0;
    wheelPulses[id]++;
    lastCyccnt[id] = nowCyc;
    lastTickMs[id] = nowMs;
    return;
  }

  uint32_t diffUs = (nowCyc - lastCyccnt[id]) / (SystemCoreClock / 1000000UL);
  if (diffUs < ENC_DEBOUNCE_US)
    return;

  wheelPulses[id]++;
  lastCyccnt[id] = nowCyc;
  lastTickMs[id] = nowMs;
  uint16_t newRpm = (uint16_t)(60000000UL / ((uint32_t)ENC_PULSES_PER_REV * diffUs));
  wheelRpm[id] = (wheelRpm[id] == 0u) ? newRpm : (uint16_t)((wheelRpm[id] + newRpm) / 2u);
}
