#include "agv.h"
#include "project_config.h"
#include "debug.h"
#include "delay.h"
#include "motion.h"
#include "scanner.h"
#include "servo.h"
#include "srf05.h"
#include <math.h>

extern TIM_HandleTypeDef htim2;

/* =========================================================================
 * CAU TRUC DU LIEU VA THUAT TOAN PID CU LY SIEU AM (DISTANCE PID)
 * ========================================================================= */
typedef struct {
  float setpoint;     // Cu ly dich mong muon truoc mui xe (cm)
  float kp;           // He so ty le P
  float ki;           // He so tich phan I
  float kd;           // He so vi phan D
  float integral;     // Bien tich luy sai so tich phan
  float prevError;    // Sai so cua chu ky truoc (dung cho khau D)
  uint32_t lastTime;  // Moc thoi gian lan tinh toan truoc (ms)
} DistancePID_t;

static DistancePID_t distPid;

static void PID_Init(DistancePID_t *pid, float setpoint)
{
  pid->setpoint = setpoint;
  pid->kp = PID_KP;
  pid->ki = PID_KI;
  pid->kd = PID_KD;
  pid->integral = 0.0f;
  pid->prevError = 0.0f;
  pid->lastTime = HAL_GetTick();
}

static uint8_t PID_Compute(DistancePID_t *pid, float currentDist, uint8_t *isArrived)
{
  uint32_t now = HAL_GetTick();
  float dt = (float)(now - pid->lastTime) / 1000.0f; // Doi sang giay
  if (dt <= 0.0f || dt > 0.5f)
  {
    dt = (float)PID_SAMPLE_TIME_MS / 1000.0f;
  }
  pid->lastTime = now;

  // 1. Tinh sai so: e(t) = Khoang cach thuc te - Khoang cach mong muon
  float error = currentDist - pid->setpoint;

  // 2. Kiem tra vung dung sai: Neu da den dich trong pham vi [setpoint - TOLERANCE, setpoint + TOLERANCE]
  if (fabsf(error) <= PID_DIST_TOLERANCE_CM || error < 0.0f)
  {
    *isArrived = 1;
    return 0;
  }
  *isArrived = 0;

  // 3. Khau Ty le (P): e cang lon -> ga cang cao de rut ngan thoi gian
  float p_term = pid->kp * error;

  // 4. Khau Tich phan (I) kem Anti-Windup: Triet tieu sai so tinh o centimet cuoi cung
  if (error < 20.0f)
  {
    pid->integral += error * dt;
    if (pid->integral > PID_I_MAX) pid->integral = PID_I_MAX;
    else if (pid->integral < -PID_I_MAX) pid->integral = -PID_I_MAX;
  }
  else
  {
    pid->integral = 0.0f;
  }
  float i_term = pid->ki * pid->integral;

  // 5. Khau Vi phan (D): Ham quan tinh chong dam sam khi xe lao toi qua nhanh
  float d_term = 0.0f;
  if (pid->prevError != 0.0f)
  {
    float derivative = (error - pid->prevError) / dt;
    d_term = pid->kd * derivative;
  }
  pid->prevError = error;

  // 6. Tong hop tin hieu dau ra PID
  float output = p_term + i_term + d_term;
  if (output <= 0.0f)
  {
    return 0;
  }

  // 7. Khong che vung chet (Deadband) va tran toc do an toan
  if (output < (float)PID_SPEED_MIN)
  {
    output = (float)PID_SPEED_MIN;
  }
  if (output > (float)PID_SPEED_MAX)
  {
    output = (float)PID_SPEED_MAX;
  }

  return (uint8_t)output;
}

/**
 * @brief  Tiep can vat the su dung vong lap PID cu ly thoi gian thuc trong dai dung sai
 * @param  targetDistanceCm: Khoang cach dich can dung (cm)
 */
void agvDrivePID(float targetDistanceCm)
{
  PID_Init(&distPid, targetDistanceCm);
  uint32_t startTime = HAL_GetTick();
  uint8_t consecutiveLost = 0;

  for (;;)
  {
    // 1. Kiem tra timeout an toan
    if ((HAL_GetTick() - startTime) > PID_TIMEOUT_MS)
    {
      motionBrake();
      debugDriveLost();
      return;
    }

    // 2. Do khoang cach tu cam bien sieu am SRF05
    float d = measureDistanceCM();
    debugDrive(d);

    // 3. Xu ly truong hop mat dau vat the
    if (d == -1.0f || d > SCAN_MAX_RANGE_CM)
    {
      consecutiveLost++;
      if (consecutiveLost >= 3)
      {
        motionBrake();
        debugDriveLost();
        return;
      }
      HAL_Delay(PID_SAMPLE_TIME_MS);
      continue;
    }
    consecutiveLost = 0;

    // 4. Vat the qua gan nguong tin cay cua cam bien
    if (d < SCAN_MIN_RANGE_CM)
    {
      motionBrake();
      return;
    }

    // 5. Tinh toan toc do qua PID
    uint8_t isArrived = 0;
    uint8_t speed = PID_Compute(&distPid, d, &isArrived);

    // 6. Neu da dat cu ly dich trong dai dung sai -> Phanh cung dung xe!
    if (isArrived)
    {
      motionBrake();
      HAL_Delay(100);
      return;
    }

    // 7. Cap toc do PID cho dong co chay tien
    motionForward(speed);
    HAL_Delay(PID_SAMPLE_TIME_MS);
  }
}

/**
 * @brief  Can huong xe chu dong: Xe vua xoay vua phat xung sieu am thang mui xe,
 *         lap tuc dung lai ngay khi bat trung vat the (khoa muc tieu)
 * @param  headingDeg: Goc lech ban dau tu servo (> 0: phai, < 0: trai)
 * @param  expectedDistCm: Khoang cach quet ban dau de so sanh
 * @retval 1: Khoa muc tieu thanh cong, 0: Timeout hoac mat dau
 */
uint8_t agvAlignHeadingActive(float headingDeg, float expectedDistCm)
{
  // 1. Neu goc lech ban dau da nam trong dai dung sai thang mat
  if (fabsf(headingDeg) <= ALIGN_TOLERANCE_DEG)
  {
    return 1;
  }

  // 2. Tra servo ve 90 do (nhin thang truc mui xe)
  moveServoSmooth(&htim2, SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);

  // 3. Ra lenh xe bat dau xoay cham theo chieu ve phia vat the
  uint8_t isRight = (headingDeg > 0.0f) ? 1 : 0;
  if (isRight)
  {
    motionSpinRight(ALIGN_SPEED_PERCENT);
  }
  else
  {
    motionSpinLeft(ALIGN_SPEED_PERCENT);
  }

  uint32_t startTime = HAL_GetTick();
  uint8_t detectCount = 0;

  // 4. Vong lap vua xoay vua do: lien tuc kiem tra khoang cach thang truoc mui
  for (;;)
  {
    // A. Kiem tra timeout an toan chong quay tron vo tan
    if ((HAL_GetTick() - startTime) > ALIGN_MAX_TIMEOUT_MS)
    {
      motionBrake();
      return 0;
    }

    // B. Do khoang cach truc tiep
    float d = measureDistanceCM();

    // C. Kiem tra tin hieu phan hoi: khi vat the lot vao tam mat sieu am
    if (d >= SCAN_MIN_RANGE_CM && d <= SCAN_MAX_RANGE_CM)
    {
      // Khoang cach do duoc phu hop voi khoang cach quet ban dau
      if (fabsf(d - expectedDistCm) <= ALIGN_DIST_MARGIN_CM || expectedDistCm <= 0.0f)
      {
        detectCount++;
        if (detectCount >= 2) // Xac nhan 2 nhip lien tiep de chong nhieu
        {
          // Nhich them 50ms de dua vat vao dung tim bup song 15 do
          HAL_Delay(ALIGN_CENTER_DELAY_MS);
          motionBrake();
          HAL_Delay(120);
          return 1; // Khoa muc tieu thanh cong!
        }
      }
      else
      {
        detectCount = 0;
      }
    }
    else
    {
      detectCount = 0;
    }

    HAL_Delay(SCAN_PERIOD_MS);
  }
}

/**
 * @brief  Dieu phoi toan trinh: Khoa huong chu dong -> Tiep can PID cu ly
 */
static void driveToObject(float xCm, float yCm)
{
  float distance = sqrtf(xCm * xCm + yCm * yCm);
  float headingDeg = atan2f(xCm, yCm) * 180.0f / (float)M_PI; // > 0: phai, < 0: trai

  // 1. Can huong chu dong co phan hoi sieu am
  uint8_t aligned = agvAlignHeadingActive(headingDeg, distance);

  // Du phong: Neu khong khoa duoc bang phan hoi (do song doi lech), xoay theo thoi gian dinh luong
  if (!aligned)
  {
    motionSpinAngle((int16_t)lroundf(headingDeg), SPEED_SPIN_CALM);
    moveServoSmooth(&htim2, SCAN_HOME_ANGLE);
    HAL_Delay(SCAN_SETTLE_MS);
  }

  // 2. Chay tiep can bang PID cu ly trong dai dung sai [3.0cm - 4.0cm]
  agvDrivePID(PID_DIST_TARGET_CM);

  // 3. Day them nhe mot doan nho de tiep xuc chac chan voi cuc tay
#if (AGV_RAM_EXTRA_CM > 0)
  motionDriveDistance(AGV_RAM_EXTRA_CM, AGV_DRIVE_SPEED);
#endif
}

void agvInit(void)
{
  delayInit();
  debugInit();
  motionInit();
  scanInit();
}

void agvRun(void)
{
  float xCm, yCm;

  if (scanForObject(&xCm, &yCm) == SCAN_OBJECT_FOUND)
  {
    driveToObject(xCm, yCm);
  }
}
