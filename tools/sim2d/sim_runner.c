#include "stm32f1xx_hal.h"
#include "main.h"
#include "agv.h"
#include "debug.h"
#include "delay.h"
#include "destination.h"
#include "encoder.h"
#include "grabber.h"
#include "imu.h"
#include "motion.h"
#include "motor.h"
#include "nav.h"
#include "project_config.h"
#include "scanner.h"
#include "servo.h"
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// --- HAL & TIM Stubs (cho phép chạy trực tiếp grabber.c và filterTriplet của srf05.c) ---
typedef struct
{
  volatile uint32_t CR1;
  volatile uint32_t CNT;
  volatile uint32_t CCR1;
  volatile uint32_t CCR2;
  volatile uint32_t CCR4;
} SimTimRegs;

#define TIM_CR1_CEN 1u
#define TIM_CHANNEL_1 1u
#define TIM_CHANNEL_2 2u
#define TIM_CHANNEL_4 4u

typedef struct
{
  SimTimRegs *Instance;
} SimTimHandle;

static SimTimRegs simTim1Regs = {1u, 3000u, 0u, 0u, 0u};
static SimTimRegs simTim2Regs = {1u, 3000u, 0u, 0u, 0u};
static SimTimHandle simHtim1 = {&simTim1Regs};
static SimTimHandle simHtim2 = {&simTim2Regs};

GPIO_TypeDef mockGpioA = {0u, 0u};
GPIO_TypeDef mockGpioB = {0u, 0u};
static DWT_Type mockDwt = {0u};
DWT_Type *DWT = &mockDwt;
uint32_t SystemCoreClock = 72000000UL;
static uint32_t simTickMs = 0u;

void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
  if (PinState == GPIO_PIN_SET) GPIOx->ODR |= GPIO_Pin;
  else GPIOx->ODR &= (uint32_t)(~GPIO_Pin);
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  return ((GPIOx->IDR & GPIO_Pin) != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}
uint32_t HAL_GetTick(void) { return simTickMs; }
void HAL_Delay(uint32_t Delay)
{
  simTickMs += Delay;
  mockDwt.CYCCNT += Delay * (SystemCoreClock / 1000UL);
}
void HAL_NVIC_DisableIRQ(int IRQn) { (void)IRQn; }
void HAL_NVIC_EnableIRQ(int IRQn) { (void)IRQn; }
void HAL_NVIC_ClearPendingIRQ(int IRQn) { (void)IRQn; }
void delayInit(void) {}
void delayUs(uint32_t us) { mockDwt.CYCCNT += us * (SystemCoreClock / 1000000UL); }

// Include thật filterTriplet từ srf05.c
#define measureDualDistanceCM srf05_hw_measureDualDistanceCM
#define measureDualDistanceMedianCM srf05_hw_measureDualDistanceMedianCM
#define measureDualDistanceConfirmCM srf05_hw_measureDualDistanceConfirmCM
#include "../../ultra_agv_brrbrbr/Core/Src/srf05.c"
#undef measureDualDistanceCM
#undef measureDualDistanceMedianCM
#undef measureDualDistanceConfirmCM

// --- Dữ liệu Thế giới 2D & Trace JSON ---
#define DEG2RAD (MATH_PI / 180.0f)
#define MAX_WALLS 24u
#define MAX_OBJS 8u
#define MAX_EVENTS 12000u

typedef struct { float x1, y1, x2, y2, specDeg; } SimWall;
typedef struct { float x, y, hx, hy; bool picked; } SimObj;

static SimWall walls[MAX_WALLS];
static uint8_t wallCount = 0u;
static SimObj objs[MAX_OBJS];
static uint8_t objCount = 0u;

static float trueX = 0.0f, trueY = 0.0f, trueThetaDeg = 0.0f;
static float estX = 0.0f, estY = 0.0f, estThetaDeg = 0.0f;
static float imuZeroDeg = 0.0f, imuRelYawDeg = 0.0f;
static uint32_t encPulses[2] = {0u, 0u};

static uint8_t logicalServoDeg = SCAN_HOME_ANGLE;
static float physicalServoDeg = (float)SCAN_HOME_ANGLE;
static int8_t lastServoDir = 0;

// Trạng thái cơ khí thực tế của tay gắp (cho phép giả lập bị khép/hạ sẵn trước khi bật nguồn)
static float physGripDeg = (float)GRABBER_GRIP_OPEN_DEG;
static float physLiftDeg = (float)GRABBER_LIFT_UP_DEG;
static bool gripPwmActive = false;
static bool liftPwmActive = false;
static bool initHomedOk = false;
static int carriedObjIdx = -1;

// Tham số sai số thực tế
static float cfgJitterCm = 0.8f;
static float cfgConeDeg = 9.0f;
static float cfgBacklashDeg = 3.0f;
static float cfgSpinNoiseDeg = 3.0f;
static float cfgDriveErrPct = 4.0f;
static float cfgDriveDriftDeg = 1.5f;
static float cfgDropoutPct = 3.0f;
static bool cfgFarFlare = false;
static uint32_t cfgSeed = 13579u;
static uint8_t cfgMaxIter = 24u;
static bool cfgAllowCloseStart = false;

static bool crashedWall = false;
static bool pickedFlag = false;
static bool deliveredFlag = false;
static uint32_t scanRayCount = 0u, spinCount = 0u, driveCount = 0u;
static uint32_t eventCount = 0u;
static char lastDecision[32] = "INIT";
static char lastStatus[32] = "INIT";

static float normDeg(float d)
{
  while (d > 180.0f) d -= 360.0f;
  while (d <= -180.0f) d += 360.0f;
  return d;
}

static float rand01(void)
{
  cfgSeed = cfgSeed * 1664525u + 1013904223u;
  return (float)((cfgSeed >> 16) & 0xFFFFu) / 65535.0f;
}

static float randSigned(void) { return rand01() * 2.0f - 1.0f; }

static void emitPrefix(void)
{
  if (eventCount >= MAX_EVENTS) return;
  if (eventCount > 0u) putchar(',');
  eventCount++;
}

static void emitEscapedStr(const char *s)
{
  putchar('"');
  for (; *s; s++)
  {
    if (*s == '"' || *s == '\\') putchar('\\');
    if (*s == '\r' || *s == '\n') continue;
    putchar(*s);
  }
  putchar('"');
}

static void emitPoseEvent(const char *act, const char *st)
{
  if (eventCount >= MAX_EVENTS) return;
  emitPrefix();
  printf("{\"t\":%lu,\"k\":\"pose\",\"x\":%.2f,\"y\":%.2f,\"th\":%.2f,"
         "\"ex\":%.2f,\"ey\":%.2f,\"eth\":%.2f,\"sv\":%.1f,\"g\":%.0f,\"l\":%.0f,"
         "\"gp\":%d,\"lp\":%d,\"co\":%d,\"act\":",
         (unsigned long)simTickMs, trueX, trueY, trueThetaDeg,
         estX, estY, estThetaDeg, physicalServoDeg, physGripDeg, physLiftDeg,
         gripPwmActive ? 1 : 0, liftPwmActive ? 1 : 0, carriedObjIdx);
  emitEscapedStr(act ? act : lastDecision);
  printf(",\"st\":");
  emitEscapedStr(st ? st : lastStatus);
  putchar('}');
}

static void emitScanEvent(uint8_t ang, float d05, float d04, ScanTargetType typ)
{
  if (eventCount >= MAX_EVENTS) return;
  scanRayCount++;
  emitPrefix();
  printf("{\"t\":%lu,\"k\":\"scan\",\"x\":%.2f,\"y\":%.2f,\"th\":%.2f,"
         "\"a\":%u,\"pa\":%.1f,\"d05\":%.2f,\"d04\":%.2f,\"typ\":%d}",
         (unsigned long)simTickMs, trueX, trueY, trueThetaDeg,
         (unsigned)ang, physicalServoDeg, d05, d04, (int)typ);
}

static void emitWallMapEvent(float x1, float y1, float x2, float y2)
{
  if (eventCount >= MAX_EVENTS) return;
  emitPrefix();
  printf("{\"t\":%lu,\"k\":\"wall\",\"x1\":%.2f,\"y1\":%.2f,\"x2\":%.2f,\"y2\":%.2f}",
         (unsigned long)simTickMs, x1, y1, x2, y2);
}

static void emitLogEvent(const char *msg)
{
  if (eventCount >= MAX_EVENTS) return;
  emitPrefix();
  printf("{\"t\":%lu,\"k\":\"log\",\"msg\":", (unsigned long)simTickMs);
  emitEscapedStr(msg);
  putchar('}');
}

// --- Wrapper bắt mọi đoạn tường mà firmware thêm vào bản đồ 2D của destination.c ---
extern void __real_destinationAddWallSeg(float x1Cm, float y1Cm, float x2Cm, float y2Cm,
                                         uint8_t startDeg, uint8_t endDeg);
void __wrap_destinationAddWallSeg(float x1Cm, float y1Cm, float x2Cm, float y2Cm,
                                  uint8_t startDeg, uint8_t endDeg)
{
  emitWallMapEvent(x1Cm, y1Cm, x2Cm, y2Cm);
  __real_destinationAddWallSeg(x1Cm, y1Cm, x2Cm, y2Cm, startDeg, endDeg);
}

// --- Mô phỏng hình học & vật lý tay gắp thật chạy từ grabber.c ---
static void updatePhysicalGrabberFromPulse(int chIdx, uint32_t pulseUs)
{
  if (pulseUs < GRABBER_MIN_PULSE_US) return;
  float deg = ((float)(pulseUs - GRABBER_MIN_PULSE_US) * (float)GRABBER_MAX_ANGLE_DEG) /
              (float)(GRABBER_MAX_PULSE_US - GRABBER_MIN_PULSE_US);
  if (deg < 0.0f) deg = 0.0f;
  if (deg > 180.0f) deg = 180.0f;

  float prevGrip = physGripDeg;
  float prevLift = physLiftDeg;
  if (chIdx == 0) physGripDeg = deg;
  else physLiftDeg = deg;

  // Kiểm tra vật lý: kẹp đóng khi cần đang hạ sát đất (lift >= 165°) -> gắp nếu ngàm đã mở trước đó và vật nằm trong tầm càng
  if (chIdx == 0 && prevGrip < 165.0f && physGripDeg >= 165.0f && physLiftDeg >= 165.0f && carriedObjIdx < 0)
  {
    float rad = trueThetaDeg * DEG2RAD;
    float noseX = trueX + (SERVO_PIVOT_OFFSET_CM + SENSOR_FACE_OFFSET_CM) * cosf(rad);
    float noseY = trueY + (SERVO_PIVOT_OFFSET_CM + SENSOR_FACE_OFFSET_CM) * sinf(rad);
    for (uint8_t i = 0u; i < objCount; i++)
    {
      if (objs[i].picked) continue;
      // Chiếu tâm vật vào hệ tọa độ mũi xe: fwd (phía trước mặt đầu dò) và lat (lệch ngang)
      float dx = objs[i].x - noseX;
      float dy = objs[i].y - noseY;
      float fwd = dx * cosf(rad) + dy * sinf(rad);
      float lat = -dx * sinf(rad) + dy * cosf(rad);
      if (fwd >= 1.5f && fwd <= 10.5f && fabsf(lat) <= 6.5f)
      {
        objs[i].picked = true;
        carriedObjIdx = (int)i;
        pickedFlag = true;
        break;
      }
    }
  }
  // Nhả vật: mở ngàm khi cần hạ sát đất
  if (chIdx == 0 && prevGrip > 130.0f && physGripDeg <= 130.0f && physLiftDeg >= 165.0f && carriedObjIdx >= 0)
  {
    float rad = trueThetaDeg * DEG2RAD;
    objs[carriedObjIdx].x = trueX + (SERVO_PIVOT_OFFSET_CM + SENSOR_FACE_OFFSET_CM + 4.5f) * cosf(rad);
    objs[carriedObjIdx].y = trueY + (SERVO_PIVOT_OFFSET_CM + SENSOR_FACE_OFFSET_CM + 4.5f) * sinf(rad);
    objs[carriedObjIdx].picked = false;
    carriedObjIdx = -1;
  }
  (void)prevLift;
}

static void simTimSetCompare(SimTimHandle *h, uint32_t ch, uint32_t val)
{
  if (h == &simHtim2 && ch == TIM_CHANNEL_1)
  {
    h->Instance->CCR1 = val;
    if (val > 0u) updatePhysicalGrabberFromPulse(0, val);
  }
  else if (h == &simHtim1 && ch == TIM_CHANNEL_4)
  {
    h->Instance->CCR4 = val;
    if (val > 0u) updatePhysicalGrabberFromPulse(1, val);
  }
}

static void simTimPwmStart(SimTimHandle *h, uint32_t ch)
{
  if (h == &simHtim2 && ch == TIM_CHANNEL_1) gripPwmActive = true;
  if (h == &simHtim1 && ch == TIM_CHANNEL_4) liftPwmActive = true;
}

static void simTimPwmStop(SimTimHandle *h, uint32_t ch)
{
  if (h == &simHtim2 && ch == TIM_CHANNEL_1) gripPwmActive = false;
  if (h == &simHtim1 && ch == TIM_CHANNEL_4) liftPwmActive = false;
}

// Include code thật của grabber.c dùng stub TIM ở trên
#define TIM_HandleTypeDef SimTimHandle
#define htim1 simHtim1
#define htim2 simHtim2
#define __HAL_TIM_SET_COMPARE(h, ch, v) simTimSetCompare((h), (ch), (v))
#define __HAL_TIM_GET_COUNTER(h) ((h)->Instance->CNT)
#define HAL_TIM_PWM_Start(h, ch) simTimPwmStart((h), (ch))
#define HAL_TIM_PWM_Stop(h, ch) simTimPwmStop((h), (ch))
#include "../../ultra_agv_brrbrbr/Core/Src/grabber.c"
#undef HAL_TIM_PWM_Stop
#undef HAL_TIM_PWM_Start
#undef __HAL_TIM_GET_COUNTER
#undef __HAL_TIM_SET_COMPARE
#undef htim2
#undef htim1
#undef TIM_HandleTypeDef

// --- Hình học va chạm & Raycast 2D siêu âm ---
static float pointSegDist(float px, float py, SimWall w)
{
  float vx = w.x2 - w.x1, vy = w.y2 - w.y1;
  float l2 = vx * vx + vy * vy;
  if (l2 < 1e-5f) return hypotf(px - w.x1, py - w.y1);
  float t = ((px - w.x1) * vx + (py - w.y1) * vy) / l2;
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  return hypotf(px - (w.x1 + t * vx), py - (w.y1 + t * vy));
}

static float minBodyWallDist(float x, float y)
{
  float best = 999.0f;
  for (uint8_t i = 0u; i < wallCount; i++)
  {
    float d = pointSegDist(x, y, walls[i]);
    if (d < best) best = d;
  }
  return best;
}

static float minNoseWallDist(float x, float y, float thDeg)
{
  float r = thDeg * DEG2RAD;
  float nx = x + (SERVO_PIVOT_OFFSET_CM + SENSOR_FACE_OFFSET_CM) * cosf(r);
  float ny = y + (SERVO_PIVOT_OFFSET_CM + SENSOR_FACE_OFFSET_CM) * sinf(r);
  return minBodyWallDist(nx, ny);
}

static bool raySegHit(float ox, float oy, float dx, float dy, SimWall w, float *outT, float *outIncDeg)
{
  float sx = w.x2 - w.x1, sy = w.y2 - w.y1;
  float denom = dx * sy - dy * sx;
  if (fabsf(denom) < 1e-6f) return false;
  float qx = w.x1 - ox, qy = w.y1 - oy;
  float t = (qx * sy - qy * sx) / denom;
  float u = (qx * dy - qy * dx) / denom;
  if (t <= 0.1f || u < 0.0f || u > 1.0f) return false;
  float len = hypotf(sx, sy);
  float nx = -sy / len, ny = sx / len;
  float cosInc = fabsf(dx * nx + dy * ny);
  if (cosInc > 1.0f) cosInc = 1.0f;
  *outT = t;
  *outIncDeg = acosf(cosInc) / DEG2RAD;
  return true;
}

static bool rayBoxHit(float ox, float oy, float dx, float dy, SimObj o, float *outT)
{
  if (o.picked) return false;
  SimWall edges[4] = {
      {o.x - o.hx, o.y - o.hy, o.x + o.hx, o.y - o.hy, 75.0f},
      {o.x + o.hx, o.y - o.hy, o.x + o.hx, o.y + o.hy, 75.0f},
      {o.x + o.hx, o.y + o.hy, o.x - o.hx, o.y + o.hy, 75.0f},
      {o.x - o.hx, o.y + o.hy, o.x - o.hx, o.y - o.hy, 75.0f}};
  float bestT = 1e9f;
  bool hit = false;
  for (uint8_t i = 0u; i < 4u; i++)
  {
    float t = 0.0f, inc = 90.0f;
    if (raySegHit(ox, oy, dx, dy, edges[i], &t, &inc) && t < bestT && inc <= 75.0f)
    {
      bestT = t;
      hit = true;
    }
  }
  if (hit) *outT = bestT;
  return hit;
}

static float castConeSensor(float faceOffCm, float halfConeDeg, float defaultSpecDeg,
                            bool canSeeObj, float maxRangeCm)
{
  float thRad = trueThetaDeg * DEG2RAD;
  float pivX = trueX + SERVO_PIVOT_OFFSET_CM * cosf(thRad);
  float pivY = trueY + SERVO_PIVOT_OFFSET_CM * sinf(thRad);
  float beamDeg = trueThetaDeg + (physicalServoDeg - (float)SCAN_HOME_ANGLE);
  float beamRad = beamDeg * DEG2RAD;
  float ox = pivX + faceOffCm * cosf(beamRad);
  float oy = pivY + faceOffCm * sinf(beamRad);

  float bestEcho = 1e9f;
  const int subRays = 25;
  for (int k = 0; k < subRays; k++)
  {
    float off = -halfConeDeg + (2.0f * halfConeDeg * (float)k) / (float)(subRays - 1);
    float rRad = (beamDeg + off) * DEG2RAD;
    float dx = cosf(rRad), dy = sinf(rRad);

    float firstWallT = 1e9f, firstWallInc = 90.0f, firstSpec = defaultSpecDeg;
    for (uint8_t w = 0u; w < wallCount; w++)
    {
      float t = 0.0f, inc = 90.0f;
      if (raySegHit(ox, oy, dx, dy, walls[w], &t, &inc) && t < firstWallT)
      {
        firstWallT = t;
        firstWallInc = inc;
        firstSpec = canSeeObj ? defaultSpecDeg : walls[w].specDeg;
      }
    }

    bool hitObj = false;
    if (canSeeObj)
    {
      for (uint8_t o = 0u; o < objCount; o++)
      {
        float tObj = 1e9f;
        if (rayBoxHit(ox, oy, dx, dy, objs[o], &tObj) && tObj < firstWallT && tObj < bestEcho)
        {
          bestEcho = tObj;
          hitObj = true;
        }
      }
    }

    if (!hitObj && firstWallT < 1e8f && firstWallInc <= firstSpec && firstWallT < bestEcho)
      bestEcho = firstWallT;
  }

  if (cfgDropoutPct > 0.0f && (rand01() * 100.0f) < cfgDropoutPct)
    return maxRangeCm + 1.0f;
  if (bestEcho > 1e8f) return maxRangeCm + 1.0f;
  if (bestEcho < 2.0f) return ECHO_ERROR_TOO_CLOSE;
  if (bestEcho > maxRangeCm) return maxRangeCm + 1.0f;
  float res = bestEcho + randSigned() * cfgJitterCm;
  if (res < 2.0f) res = 2.0f;
  if (res > maxRangeCm) return maxRangeCm + 1.0f;
  return res;
}

void measureDualDistanceCM(float *dist05Cm, float *dist04Cm)
{
  HAL_Delay(SENSOR_INTER_PING_MS * 2u);
  float d05 = castConeSensor(SENSOR_FACE_OFFSET_CM, cfgConeDeg, 65.0f, true, SCAN_MAX_RANGE_CM);
  float d04 = castConeSensor(SENSOR_FACE_OFFSET_CM - SENSOR_DEPTH_OFFSET_CM, cfgConeDeg, 60.0f, false, SRF04_MAX_RANGE_CM);
  if (cfgFarFlare && d05 > 24.0f && d05 <= SCAN_MAX_RANGE_CM && d04 > SRF04_MAX_RANGE_CM)
    d04 = d05 + SENSOR_DEPTH_OFFSET_CM + 0.5f;

  lastStatus05 = (d05 == ECHO_ERROR_TIMEOUT) ? ECHO_NO_RISE : ((d05 > SRF05_MAX_RANGE_CM) ? ECHO_OUT_RANGE : ECHO_OK);
  lastStatus04 = (d04 == ECHO_ERROR_TIMEOUT) ? ECHO_NO_RISE : ((d04 > SRF04_MAX_RANGE_CM) ? ECHO_OUT_RANGE : ECHO_OK);
  if (dist05Cm) *dist05Cm = d05;
  if (dist04Cm) *dist04Cm = d04;
}

void measureDualDistanceConfirmCM(float firstD05Cm, float firstD04Cm, float *dist05Cm, float *dist04Cm)
{
  float s05[SCAN_MEDIAN_SAMPLES], s04[SCAN_MEDIAN_SAMPLES];
  EchoStatus err05 = (firstD05Cm == ECHO_ERROR_TIMEOUT) ? lastStatus05 : ECHO_NO_RISE;
  EchoStatus err04 = (firstD04Cm == ECHO_ERROR_TIMEOUT) ? lastStatus04 : ECHO_NO_RISE;
  s05[0] = firstD05Cm;
  s04[0] = firstD04Cm;
  for (uint8_t i = 1u; i < SCAN_MEDIAN_SAMPLES; i++)
  {
    float d05 = ECHO_ERROR_TIMEOUT, d04 = ECHO_ERROR_TIMEOUT;
    measureDualDistanceCM(dist05Cm ? &d05 : NULL, dist04Cm ? &d04 : NULL);
    s05[i] = d05;
    s04[i] = d04;
  }
  if (dist05Cm) *dist05Cm = filterTriplet(s05, SRF05_MAX_RANGE_CM, false, err05, &lastStatus05);
  if (dist04Cm) *dist04Cm = filterTriplet(s04, SRF04_MAX_RANGE_CM, true, err04, &lastStatus04);
}

void measureDualDistanceMedianCM(float *dist05Cm, float *dist04Cm)
{
  float firstD05 = ECHO_ERROR_TIMEOUT, firstD04 = ECHO_ERROR_TIMEOUT;
  measureDualDistanceCM(dist05Cm ? &firstD05 : NULL, dist04Cm ? &firstD04 : NULL);
  measureDualDistanceConfirmCM(firstD05, firstD04, dist05Cm, dist04Cm);
}

// --- Servo, IMU, Encoder, Motor & Motion Hardware Simulation ---
void servoInit(void)
{
  logicalServoDeg = SCAN_HOME_ANGLE;
  physicalServoDeg = (float)SCAN_HOME_ANGLE;
  lastServoDir = 0;
}

void setServoAngle(uint8_t angleDeg)
{
  if (angleDeg < SERVO_SAFE_MIN_DEG) angleDeg = SERVO_SAFE_MIN_DEG;
  if (angleDeg > SERVO_SAFE_MAX_DEG) angleDeg = SERVO_SAFE_MAX_DEG;
  int16_t diff = (int16_t)angleDeg - (int16_t)logicalServoDeg;
  if (diff != 0)
  {
    int8_t dir = (diff > 0) ? 1 : -1;
    if (lastServoDir != 0 && dir != lastServoDir && cfgBacklashDeg > 0.0f)
      physicalServoDeg = (float)angleDeg - (float)dir * cfgBacklashDeg;
    else
      physicalServoDeg = (float)angleDeg;
    lastServoDir = dir;
  }
  logicalServoDeg = angleDeg;
}

void moveServoSmooth(uint8_t targetDeg)
{
  if (targetDeg < SERVO_SAFE_MIN_DEG) targetDeg = SERVO_SAFE_MIN_DEG;
  if (targetDeg > SERVO_SAFE_MAX_DEG) targetDeg = SERVO_SAFE_MAX_DEG;
  int16_t diff = abs((int16_t)targetDeg - (int16_t)logicalServoDeg);
  setServoAngle(targetDeg);
  HAL_Delay((uint32_t)diff * SERVO_SMOOTH_DELAY_MS);
}

uint8_t getServoAngle(void) { return logicalServoDeg; }
void servoStop(void) {}

bool imuInit(void) { return true; }
bool imuIsReady(void) { return true; }
void imuCalibrate(void) { imuZeroDeg = trueThetaDeg; imuRelYawDeg = 0.0f; HAL_Delay(200u); }
void imuResetYaw(void) { imuRelYawDeg = 0.0f; }
float imuUpdate(void) { return imuRelYawDeg; }
float imuGetYawDeg(void) { return imuRelYawDeg; }
float imuGetHeadingDeg(void) { return normDeg(trueThetaDeg - imuZeroDeg); }
float imuGetRateDps(void) { return 0.0f; }

void encoderInit(void) { encPulses[0] = encPulses[1] = 0u; }
void encoderResetPulses(void) { encoderInit(); }
uint32_t encoderGetPulses(MotorId m) { return encPulses[m]; }
uint16_t encoderGetRpm(MotorId m) { (void)m; return 0u; }

void motorInit(void) {}
void motorSetSpeed(MotorId m, MotorDir d, uint8_t s) { (void)m; (void)d; (void)s; }
void motorSetDutyRaw(MotorId m, MotorDir d, uint16_t duty) { (void)m; (void)d; (void)duty; }
void motorStopAll(void) {}
void motorBrakeAll(void) {}
uint8_t motorGetSpeedPercent(MotorId m) { (void)m; return 0u; }
uint16_t motorGetDutyRaw(MotorId m) { (void)m; return 0u; }
MotorDir motorGetDir(MotorId m) { (void)m; return MOTOR_STOP; }

void motionInit(void) {}
void motionForward(uint8_t s) { (void)s; }
void motionBackward(uint8_t s) { (void)s; }
void motionBrake(void) {}
void motionStop(void) {}
void motionSpinAngle(int16_t a, uint8_t s) { (void)s; motionSpinAnglePid(a, PID_TARGET_SPIN_RPM); }
void motionDriveDistance(int16_t d, uint8_t s) { (void)s; motionDriveDistancePid(d, PID_TARGET_DRIVE_RPM); }
void motionDriveTimePid(MotorDir d, uint32_t ms, uint16_t rpm) { (void)d; (void)ms; (void)rpm; }
void motionForwardHoldStart(void) {}
void motionForwardHoldStep(uint8_t s) { (void)s; }

void motionSpinAnglePid(int16_t angleDeg, uint16_t targetRpm)
{
  (void)targetRpm;
  if (angleDeg == 0) return;
  spinCount++;
  float actualDelta = -(float)angleDeg + randSigned() * cfgSpinNoiseDeg;
  int steps = (int)ceilf(fabsf(actualDelta) / 12.0f);
  if (steps < 1) steps = 1;
  float stepDeg = actualDelta / (float)steps;
  for (int i = 0; i < steps; i++)
  {
    trueThetaDeg = normDeg(trueThetaDeg + stepDeg);
    estThetaDeg = normDeg(trueThetaDeg - imuZeroDeg);
    HAL_Delay(45u);
    emitPoseEvent((angleDeg > 0) ? "SPIN_R" : "SPIN_L", "SPINNING");
  }
  imuRelYawDeg = actualDelta;
  uint32_t pulses = (uint32_t)lroundf(fabsf(actualDelta) * (float)ENC_SPIN_360_PULSES / 360.0f);
  encPulses[0] = encPulses[1] = pulses;
}

void motionDriveDistancePid(int16_t distanceCm, uint16_t targetRpm)
{
  (void)targetRpm;
  if (distanceCm == 0) return;
  driveCount++;
  uint32_t targetPulses = (uint32_t)((float)abs(distanceCm) * (float)ENC_PULSES_PER_REV / WHEEL_CIRCUMFERENCE_CM + 0.5f);
  if (targetPulses == 0u) targetPulses = 1u;
  float encDistCm = ((float)targetPulses * WHEEL_CIRCUMFERENCE_CM) / (float)ENC_PULSES_PER_REV;
  float slipScale = 1.0f + randSigned() * (cfgDriveErrPct / 100.0f);
  float realDistCm = encDistCm * slipScale;
  float driftDeg = randSigned() * cfgDriveDriftDeg;
  float dir = (distanceCm > 0) ? 1.0f : -1.0f;

  float prevNose = minNoseWallDist(trueX, trueY, trueThetaDeg);
  int steps = (int)ceilf(realDistCm / 2.5f);
  if (steps < 1) steps = 1;
  float stepCm = realDistCm / (float)steps;
  float stepEncCm = encDistCm / (float)steps;
  float stepDrift = driftDeg / (float)steps;

  for (int s = 0; s < steps; s++)
  {
    trueThetaDeg = normDeg(trueThetaDeg + stepDrift);
    float rad = trueThetaDeg * DEG2RAD;
    trueX += dir * stepCm * cosf(rad);
    trueY += dir * stepCm * sinf(rad);
    float estRad = normDeg(trueThetaDeg - imuZeroDeg) * DEG2RAD;
    estX += dir * stepEncCm * cosf(estRad);
    estY += dir * stepEncCm * sinf(estRad);
    estThetaDeg = normDeg(trueThetaDeg - imuZeroDeg);
    HAL_Delay(90u);

    float bDist = minBodyWallDist(trueX, trueY);
    float nDist = minNoseWallDist(trueX, trueY, trueThetaDeg);
    bool noseWorse = (nDist < 2.0f) && (!cfgAllowCloseStart || nDist < (prevNose - 0.1f) || nDist < 0.3f);
    if (bDist < DEST_BODY_RADIUS_CM || noseWorse)
      crashedWall = true;
    prevNose = nDist;
    emitPoseEvent((distanceCm > 0) ? "FORWARD" : "BACKWARD", crashedWall ? "CRASH" : "DRIVING");
  }

  encPulses[0] = encPulses[1] = targetPulses;
}

// --- Debug hooks ghi trace JSON ---
const char *debugF1(float v)
{
  static char bufs[8][16];
  static uint8_t idx = 0u;
  char *b = bufs[idx++ & 7u];
  int32_t t = (int32_t)lroundf(v * 10.0f);
  if (t < 0) { t = -t; snprintf(b, 16u, "-%ld.%ld", (long)(t / 10), (long)(t % 10)); }
  else { snprintf(b, 16u, "%ld.%ld", (long)(t / 10), (long)(t % 10)); }
  return b;
}

void debugLog(const char *format, ...)
{
  char line[256];
  va_list args;
  va_start(args, format);
  vsnprintf(line, sizeof(line), format, args);
  va_end(args);
  if (strncmp(line, "DELIVERED ", 10) == 0)
    deliveredFlag = true;
  emitLogEvent(line);
}

void debugInit(void) {}
void debugDecision(const char *d)
{
  if (d) { strncpy(lastDecision, d, sizeof(lastDecision) - 1u); lastDecision[sizeof(lastDecision) - 1u] = '\0'; }
}
void debugScan(float d05, float d04, uint8_t ang, uint8_t hit, ScanTargetType typ)
{
  (void)hit;
  emitScanEvent(ang, d05, d04, typ);
}
void debugScanFine(uint8_t ang, float d05, float d04, ScanTargetType typ, float bd, uint8_t ba)
{
  (void)bd; (void)ba;
  emitScanEvent(ang, d05, d04, typ);
}
void debugAlign(int16_t c, uint8_t fa, int16_t fs, float bd, bool ff)
{ (void)c; (void)fa; (void)fs; (void)bd; (void)ff; }
void debugApproach(uint32_t t, float d05, float d04, float tg, bool w)
{ (void)t; (void)d05; (void)d04; (void)tg; (void)w; }
void debugDriveEnd(uint32_t t, float d05, float d04, uint8_t l, const char *st)
{ (void)t; (void)d05; (void)d04; (void)l; if (st) { strncpy(lastStatus, st, sizeof(lastStatus) - 1u); emitPoseEvent(lastDecision, st); } }
void debugWallDetected(float d05, float d04) { (void)d05; (void)d04; emitPoseEvent("WALL", "OBSTACLE_FND"); }
void debugWallTurn(bool l, float ld, bool r, float rd, int16_t td) { (void)l; (void)ld; (void)r; (void)rd; (void)td; }
void debugMotorTest(const char *s) { (void)s; }
void debugMotorPwmTest(uint16_t d, uint8_t p, const char *s) { (void)d; (void)p; (void)s; }
void debugState(const char *dec, const char *st)
{
  if (dec) strncpy(lastDecision, dec, sizeof(lastDecision) - 1u);
  if (st) strncpy(lastStatus, st, sizeof(lastStatus) - 1u);
  emitPoseEvent(lastDecision, lastStatus);
}
void debugNavWaypoint(uint8_t i, uint8_t n, float cx, float cy, float ct, float tx, float ty, float tt, const char *p)
{ (void)i; (void)n; (void)cx; (void)cy; (void)ct; (void)tx; (void)ty; (void)tt; (void)p; }
void debugNavPreview(uint8_t i, uint8_t n, float cx, float cy, float ct, float tx, float ty, float tt, const char *a, uint8_t s, bool r)
{ (void)i; (void)n; (void)cx; (void)cy; (void)ct; (void)tx; (void)ty; (void)tt; (void)a; (void)s; (void)r; }
void debugStepPreview(uint8_t i, uint8_t n, const char *a, uint8_t s, bool r)
{ (void)i; (void)n; (void)a; (void)s; (void)r; }
void debugGrabber(uint8_t g, uint8_t l, const char *act, const char *st)
{
  (void)g; (void)l;
  if (act) strncpy(lastDecision, act, sizeof(lastDecision) - 1u);
  if (st) strncpy(lastStatus, st, sizeof(lastStatus) - 1u);
  emitPoseEvent(act, st);
}

// --- Đọc cấu hình kịch bản đơn giản từ stdin ---
static void parseConfigStdin(void)
{
  char line[256];
  while (fgets(line, sizeof(line), stdin))
  {
    char cmd[32];
    if (sscanf(line, "%31s", cmd) != 1) continue;
    if (strcmp(cmd, "wall") == 0 && wallCount < MAX_WALLS)
    {
      SimWall w = {0, 0, 0, 0, 60.0f};
      if (sscanf(line, "%*s %f %f %f %f %f", &w.x1, &w.y1, &w.x2, &w.y2, &w.specDeg) >= 4)
        walls[wallCount++] = w;
    }
    else if (strcmp(cmd, "obj") == 0 && objCount < MAX_OBJS)
    {
      SimObj o = {0, 0, 2.5f, 2.5f, false};
      if (sscanf(line, "%*s %f %f %f %f", &o.x, &o.y, &o.hx, &o.hy) >= 2)
        objs[objCount++] = o;
    }
    else if (strcmp(cmd, "robot") == 0)
    {
      sscanf(line, "%*s %f %f %f", &trueX, &trueY, &trueThetaDeg);
    }
    else if (strcmp(cmd, "grabber") == 0)
    {
      sscanf(line, "%*s %f %f", &physGripDeg, &physLiftDeg);
    }
    else if (strcmp(cmd, "noise") == 0)
    {
      int flare = 0, closeStart = 0;
      unsigned long seed = 13579ul;
      unsigned int maxIt = 24u;
      sscanf(line, "%*s %f %f %f %f %f %f %f %d %lu %u %d",
             &cfgJitterCm, &cfgConeDeg, &cfgBacklashDeg, &cfgSpinNoiseDeg,
             &cfgDriveErrPct, &cfgDriveDriftDeg, &cfgDropoutPct,
             &flare, &seed, &maxIt, &closeStart);
      cfgFarFlare = (flare != 0);
      cfgSeed = (uint32_t)seed;
      cfgMaxIter = (uint8_t)maxIt;
      cfgAllowCloseStart = (closeStart != 0);
    }
  }
}

int main(void)
{
  parseConfigStdin();
  float initX = trueX, initY = trueY, initTh = trueThetaDeg;

  printf("{\"events\":[");
  emitPoseEvent("BOOT", "POWER_ON");

  agvInit();
  initHomedOk = (fabsf(physGripDeg - (float)GRABBER_GRIP_OPEN_DEG) < 2.0f) &&
                (fabsf(physLiftDeg - (float)GRABBER_LIFT_UP_DEG) < 2.0f) &&
                !gripPwmActive && !liftPwmActive;

  agvRun(); // AGV_STATE_INIT_CALIB -> AGV_STATE_SCAN
  uint8_t iters = 0u;
  for (; iters < cfgMaxIter && !deliveredFlag && !crashedWall; iters++)
    agvRun();

  float homeErrCm = hypotf(trueX - initX, trueY - initY);
  float headErrDeg = fabsf(normDeg(trueThetaDeg - initTh));

  printf("],\"summary\":{\"picked\":%s,\"delivered\":%s,\"crashed\":%s,\"initHomedOk\":%s,"
         "\"iters\":%u,\"simTimeMs\":%lu,\"scanRays\":%lu,\"spins\":%lu,\"drives\":%lu,"
         "\"homeErrCm\":%.2f,\"headErrDeg\":%.2f,\"finalX\":%.2f,\"finalY\":%.2f,\"finalTh\":%.2f}}\n",
         pickedFlag ? "true" : "false",
         deliveredFlag ? "true" : "false",
         crashedWall ? "true" : "false",
         initHomedOk ? "true" : "false",
         (unsigned)iters, (unsigned long)simTickMs,
         (unsigned long)scanRayCount, (unsigned long)spinCount, (unsigned long)driveCount,
         homeErrCm, headErrDeg, trueX, trueY, trueThetaDeg);
  return 0;
}
