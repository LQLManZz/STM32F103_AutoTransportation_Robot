#include "srf05.h"
#include "project_config.h"
#include "delay.h"
#include <math.h>
#include <stddef.h>

typedef enum
{
  ECHO_OK = 0,
  ECHO_OUT_RANGE,
  ECHO_STUCK_HI,
  ECHO_NO_RISE
} EchoStatus;

static EchoStatus lastStatus05 = ECHO_NO_RISE;
static EchoStatus lastStatus04 = ECHO_NO_RISE;
static uint32_t lastAnyPingMs = 0u;
static uint32_t lastSensorPingMs[2] = {0u, 0u};
static bool hasAnyPing = false;
static bool hasSensorPing[2] = {false, false};

static bool isEchoHigh(GPIO_TypeDef *port, uint16_t pin)
{
  return (port->IDR & pin) != 0u;
}

// Chờ đủ giãn cách giữa 2 lần phát bất kỳ và giữa 2 lần phát trên cùng một cảm biến
static void waitPingCooldown(uint8_t sensorIdx)
{
  uint32_t nowMs = HAL_GetTick();
  uint32_t waitMs = 0u;

  if (hasAnyPing)
  {
    uint32_t elapsedAny = nowMs - lastAnyPingMs;
    if (elapsedAny < SENSOR_INTER_PING_MS)
      waitMs = SENSOR_INTER_PING_MS - elapsedAny;
  }
  if (hasSensorPing[sensorIdx])
  {
    uint32_t elapsedSame = nowMs - lastSensorPingMs[sensorIdx];
    if (elapsedSame < SENSOR_SAME_COOLDOWN_MS)
    {
      uint32_t needSame = SENSOR_SAME_COOLDOWN_MS - elapsedSame;
      if (needSame > waitMs)
        waitMs = needSame;
    }
  }
  if (waitMs > 0u)
    HAL_Delay(waitMs);
}

// Phát xung Trigger sạch và cập nhật mốc thời gian phát sóng
static void triggerPulse(GPIO_TypeDef *trigPort, uint16_t trigPin, uint8_t sensorIdx)
{
  HAL_GPIO_WritePin(trigPort, trigPin, GPIO_PIN_RESET);
  delayUs(TRIGGER_RESET_US);
  HAL_GPIO_WritePin(trigPort, trigPin, GPIO_PIN_SET);
  delayUs(TRIGGER_PULSE_US);
  HAL_GPIO_WritePin(trigPort, trigPin, GPIO_PIN_RESET);

  uint32_t nowMs = HAL_GetTick();
  lastAnyPingMs = nowMs;
  lastSensorPingMs[sensorIdx] = nowMs;
  hasAnyPing = true;
  hasSensorPing[sensorIdx] = true;
}

// Chờ Echo hạ LOW và tự giải kẹt nếu module siêu âm treo mức cao
static bool waitEchoIdle(GPIO_TypeDef *trigPort, uint16_t trigPin,
                         GPIO_TypeDef *echoPort, uint16_t echoPin, uint8_t sensorIdx)
{
  if (!isEchoHigh(echoPort, echoPin))
    return true;

  uint32_t startMs = HAL_GetTick();
  while ((HAL_GetTick() - startMs) < ECHO_IDLE_WAIT_MS)
  {
    if (!isEchoHigh(echoPort, echoPin))
    {
      HAL_Delay(ECHO_POST_IDLE_MS);
      return true;
    }
    HAL_Delay(1u);
  }

  // Phát 1 xung Trigger giải kẹt khi Echo treo HIGH quá lâu rồi chờ hạ xuống
  triggerPulse(trigPort, trigPin, sensorIdx);
  uint32_t recStartMs = HAL_GetTick();
  while ((HAL_GetTick() - recStartMs) < ECHO_RECOVERY_WAIT_MS)
  {
    if (!isEchoHigh(echoPort, echoPin))
    {
      HAL_Delay(SENSOR_INTER_PING_MS);
      return true;
    }
    HAL_Delay(1u);
  }
  return false;
}

// Đo độ rộng xung mức HIGH trên chân Echo (us) có lọc sụt cạnh xuống; trả về 0 nếu lỗi hoặc vượt tầm
static uint32_t pulseInHigh(GPIO_TypeDef *port, uint16_t pin, uint32_t riseTimeoutUs,
                            uint32_t maxEchoUs, EchoStatus *outStatus)
{
  uint32_t usTicks = SystemCoreClock / 1000000UL;
  if (usTicks == 0UL)
    usTicks = 1UL;
  uint32_t riseTicks = riseTimeoutUs * usTicks;
  uint32_t maxEchoTicks = maxEchoUs * usTicks;
  uint32_t minTicks = ECHO_MIN_NOISE_US * usTicks;
  uint32_t glitchTicks = ECHO_FALL_GLITCH_US * usTicks;
  uint32_t startWait = DWT->CYCCNT;

  while ((DWT->CYCCNT - startWait) < riseTicks)
  {
    // Chờ chân về LOW nếu còn dư mức cao trước đó
    while (isEchoHigh(port, pin))
    {
      if ((DWT->CYCCNT - startWait) >= riseTicks)
      {
        if (outStatus) *outStatus = ECHO_STUCK_HI;
        return 0UL;
      }
    }
    // Chờ cạnh lên bắt đầu xung Echo
    while (!isEchoHigh(port, pin))
    {
      if ((DWT->CYCCNT - startWait) >= riseTicks)
      {
        if (outStatus) *outStatus = ECHO_NO_RISE;
        return 0UL;
      }
    }
    // Đo độ rộng mức HIGH trong bán kính quét, bỏ qua nhiễu sụt LOW ngắn < ECHO_FALL_GLITCH_US
    uint32_t pulseStart = DWT->CYCCNT;
    uint32_t pulseEnd = pulseStart;
    while (1)
    {
      if ((DWT->CYCCNT - pulseStart) > maxEchoTicks)
      {
        if (outStatus) *outStatus = ECHO_OUT_RANGE;
        return 0UL;
      }
      if (!isEchoHigh(port, pin))
      {
        uint32_t fallStart = DWT->CYCCNT;
        bool stayedLow = true;
        while ((DWT->CYCCNT - fallStart) < glitchTicks)
        {
          if (isEchoHigh(port, pin))
          {
            stayedLow = false;
            break;
          }
        }
        if (stayedLow)
        {
          pulseEnd = fallStart;
          break;
        }
      }
    }
    uint32_t pulseTicks = pulseEnd - pulseStart;
    if (pulseTicks > maxEchoTicks)
    {
      if (outStatus) *outStatus = ECHO_OUT_RANGE;
      return 0UL;
    }
    if (pulseTicks >= minTicks)
    {
      if (outStatus) *outStatus = ECHO_OK;
      return pulseTicks / usTicks;
    }
  }

  if (outStatus) *outStatus = ECHO_NO_RISE;
  return 0UL;
}

// Đo 1 lượt đơn trên 1 cảm biến trong bán kính maxRangeCm
static float measureSingleOnce(GPIO_TypeDef *trigPort, uint16_t trigPin,
                               GPIO_TypeDef *echoPort, uint16_t echoPin,
                               uint8_t sensorIdx, float maxRangeCm, EchoStatus *outStatus)
{
  waitPingCooldown(sensorIdx);
  if (!waitEchoIdle(trigPort, trigPin, echoPort, echoPin, sensorIdx))
  {
    if (outStatus) *outStatus = ECHO_STUCK_HI;
    return ECHO_ERROR_TIMEOUT;
  }

  triggerPulse(trigPort, trigPin, sensorIdx);

  uint32_t maxEchoUs = (uint32_t)ceilf((maxRangeCm * 2.0f) / SOUND_SPEED_CM_US) + ECHO_RANGE_MARGIN_US;
  EchoStatus status = ECHO_NO_RISE;
  uint32_t durationUs = pulseInHigh(echoPort, echoPin, ECHO_RISE_TIMEOUT_US, maxEchoUs, &status);
  if (outStatus) *outStatus = status;

  if (status == ECHO_OUT_RANGE) return ECHO_OUT_OF_RANGE_CM;
  if (durationUs == 0UL) return ECHO_ERROR_TIMEOUT;
  if (durationUs < ECHO_MIN_VALID_US) return ECHO_ERROR_TOO_CLOSE;

  float distCm = ((float)durationUs * SOUND_SPEED_CM_US) / 2.0f;
  if (distCm > maxRangeCm)
  {
    if (outStatus) *outStatus = ECHO_OUT_RANGE;
    return ECHO_OUT_OF_RANGE_CM;
  }
  return distCm;
}

// Đo khoảng cách (cm) cho 1 cảm biến có tự thử lại khi gặp lỗi STUCK_HI hoặc NO_RISE
static float measureSingleDistanceCM(GPIO_TypeDef *trigPort, uint16_t trigPin,
                                     GPIO_TypeDef *echoPort, uint16_t echoPin,
                                     uint8_t sensorIdx, float maxRangeCm, EchoStatus *outStatus)
{
  EchoStatus status = ECHO_NO_RISE;
  float distCm = ECHO_ERROR_TIMEOUT;

  for (uint8_t attempt = 0u; attempt <= SENSOR_MAX_RETRIES; attempt++)
  {
    distCm = measureSingleOnce(trigPort, trigPin, echoPort, echoPin,
                               sensorIdx, maxRangeCm, &status);
    if (status == ECHO_OK || status == ECHO_OUT_RANGE)
      break;
    if (attempt < SENSOR_MAX_RETRIES)
      HAL_Delay(SENSOR_RETRY_DELAY_MS);
  }
  if (outStatus) *outStatus = status;
  return distCm;
}

// Lọc đồng thuận 2/3 từ 3 mẫu đo; tách riêng lỗi phần cứng, ngoài tầm và nhiễu tán loạn
static float filterTriplet(const float raw[SCAN_MEDIAN_SAMPLES], float maxRangeCm, bool isSrf04,
                           EchoStatus lastErrStatus, EchoStatus *outStatus)
{
  uint8_t timeoutCount = 0u;
  uint8_t tooCloseCount = 0u;
  uint8_t outRangeCount = 0u;
  uint8_t validCount = 0u;
  float validVals[SCAN_MEDIAN_SAMPLES];

  for (uint8_t i = 0u; i < SCAN_MEDIAN_SAMPLES; i++)
  {
    float v = raw[i];
    if (v == ECHO_ERROR_TOO_CLOSE)
      tooCloseCount++;
    else if (v == ECHO_ERROR_TIMEOUT || v < 0.0f)
      timeoutCount++;
    else if (v > maxRangeCm)
      outRangeCount++;
    else
      validVals[validCount++] = v;
  }

  if (timeoutCount >= 2u)
  {
    if (outStatus)
      *outStatus = (lastErrStatus == ECHO_STUCK_HI) ? ECHO_STUCK_HI : ECHO_NO_RISE;
    return ECHO_ERROR_TIMEOUT;
  }
  if (outRangeCount >= 2u)
  {
    if (outStatus) *outStatus = ECHO_OUT_RANGE;
    return ECHO_OUT_OF_RANGE_CM;
  }
  if (tooCloseCount >= 2u)
  {
    if (outStatus) *outStatus = ECHO_OK;
    return ECHO_ERROR_TOO_CLOSE;
  }

  if (validCount >= 2u)
  {
    for (uint8_t i = 1u; i < validCount; i++)
    {
      float key = validVals[i];
      uint8_t j = i;
      while (j > 0u && validVals[j - 1u] > key)
      {
        validVals[j] = validVals[j - 1u];
        j--;
      }
      validVals[j] = key;
    }

    if (validCount == 3u)
    {
      float d01 = validVals[1] - validVals[0];
      float d12 = validVals[2] - validVals[1];
      if (d01 <= SCAN_SAMPLE_SPREAD_CM && d12 <= SCAN_SAMPLE_SPREAD_CM)
      {
        if (outStatus) *outStatus = ECHO_OK;
        if ((validVals[2] - validVals[0]) <= SCAN_SAMPLE_TRIPLE_SPREAD_CM)
          return (validVals[0] + validVals[1] + validVals[2]) / 3.0f;
        return (d01 <= d12) ? ((validVals[0] + validVals[1]) * 0.5f)
                            : ((validVals[1] + validVals[2]) * 0.5f);
      }
      if (d01 <= SCAN_SAMPLE_SPREAD_CM)
      {
        if (outStatus) *outStatus = ECHO_OK;
        return (validVals[0] + validVals[1]) * 0.5f;
      }
      if (d12 <= SCAN_SAMPLE_SPREAD_CM)
      {
        if (outStatus) *outStatus = ECHO_OK;
        return (validVals[1] + validVals[2]) * 0.5f;
      }
    }
    else if ((validVals[1] - validVals[0]) <= SCAN_SAMPLE_SPREAD_CM)
    {
      if (outStatus) *outStatus = ECHO_OK;
      return (validVals[0] + validVals[1]) * 0.5f;
    }

    // Các mẫu hợp lệ phân tán quá rộng: SRF04 fail-safe lỗi phần cứng để không tạo vật giả
    if (outStatus) *outStatus = isSrf04 ? ECHO_NO_RISE : ECHO_OUT_RANGE;
    return isSrf04 ? ECHO_ERROR_TIMEOUT : ECHO_OUT_OF_RANGE_CM;
  }

  // Trường hợp hòa 1-1-1: chỉ giữ khi có 1 mẫu quá gần + 1 mẫu hợp lệ sát biên dưới
  if (tooCloseCount == 1u && validCount == 1u &&
      validVals[0] <= (SCAN_MIN_RANGE_CM + SCAN_SAMPLE_SPREAD_CM))
  {
    if (outStatus) *outStatus = ECHO_OK;
    return validVals[0];
  }

  if (outStatus) *outStatus = isSrf04 ? ECHO_NO_RISE : ECHO_OUT_RANGE;
  return isSrf04 ? ECHO_ERROR_TIMEOUT : ECHO_OUT_OF_RANGE_CM;
}

void measureDualDistanceCM(float *dist05Cm, float *dist04Cm)
{
  if (dist05Cm)
    *dist05Cm = measureSingleDistanceCM(TRIGGER_SRF05_PORT, TRIGGER_SRF05_PIN,
                                        ECHO_SRF05_PORT, ECHO_SRF05_PIN,
                                        0u, SRF05_MAX_RANGE_CM, &lastStatus05);
  if (dist04Cm)
    *dist04Cm = measureSingleDistanceCM(TRIGGER_SRF04_PORT, TRIGGER_SRF04_PIN,
                                        ECHO_SRF04_PORT, ECHO_SRF04_PIN,
                                        1u, SRF04_MAX_RANGE_CM, &lastStatus04);
}

void measureDualDistanceConfirmCM(float firstD05Cm, float firstD04Cm, float *dist05Cm, float *dist04Cm)
{
  float s05[SCAN_MEDIAN_SAMPLES];
  float s04[SCAN_MEDIAN_SAMPLES];
  EchoStatus errSt05 = (firstD05Cm == ECHO_ERROR_TIMEOUT) ? lastStatus05 : ECHO_NO_RISE;
  EchoStatus errSt04 = (firstD04Cm == ECHO_ERROR_TIMEOUT) ? lastStatus04 : ECHO_NO_RISE;

  s05[0] = firstD05Cm;
  s04[0] = firstD04Cm;
  for (uint8_t i = 1u; i < SCAN_MEDIAN_SAMPLES; i++)
  {
    float d05 = ECHO_ERROR_TIMEOUT;
    float d04 = ECHO_ERROR_TIMEOUT;
    measureDualDistanceCM(dist05Cm ? &d05 : NULL, dist04Cm ? &d04 : NULL);
    s05[i] = d05;
    s04[i] = d04;
    if (dist05Cm && d05 == ECHO_ERROR_TIMEOUT)
      errSt05 = lastStatus05;
    if (dist04Cm && d04 == ECHO_ERROR_TIMEOUT)
      errSt04 = lastStatus04;
  }

  if (dist05Cm)
    *dist05Cm = filterTriplet(s05, SRF05_MAX_RANGE_CM, false, errSt05, &lastStatus05);
  if (dist04Cm)
    *dist04Cm = filterTriplet(s04, SRF04_MAX_RANGE_CM, true, errSt04, &lastStatus04);
}

void measureDualDistanceMedianCM(float *dist05Cm, float *dist04Cm)
{
  float firstD05 = ECHO_ERROR_TIMEOUT;
  float firstD04 = ECHO_ERROR_TIMEOUT;
  measureDualDistanceCM(dist05Cm ? &firstD05 : NULL, dist04Cm ? &firstD04 : NULL);
  measureDualDistanceConfirmCM(firstD05, firstD04, dist05Cm, dist04Cm);
}

const char *srf05GetStateStr(bool isSrf04)
{
  EchoStatus st = isSrf04 ? lastStatus04 : lastStatus05;
  if (st == ECHO_OK) return "OK";
  if (st == ECHO_OUT_RANGE) return "OUT_RNG";
  if (st == ECHO_STUCK_HI) return "STUCK_HI";
  return "NO_RISE";
}
