#include "imu.h"
#include "project_config.h"
#include <math.h>

#define MPU6050_REG_SMPLRT_DIV   0x19u
#define MPU6050_REG_CONFIG       0x1Au
#define MPU6050_REG_GYRO_CONFIG  0x1Bu
#define MPU6050_REG_GYRO_ZOUT_H  0x47u
#define MPU6050_REG_PWR_MGMT_1   0x6Bu
#define MPU6050_REG_WHO_AM_I     0x75u

extern I2C_HandleTypeDef IMU_I2C_HANDLE;

static bool imuReady = false;
static float gyroZBias = 0.0f;
static float yawDeg = 0.0f;
static float headingDeg = 0.0f;
static float prevRateDps = 0.0f;
static uint32_t lastCyccnt = 0u;

static float normalizeHeadingDeg(float deg)
{
  while (deg > 180.0f) deg -= 360.0f;
  while (deg <= -180.0f) deg += 360.0f;
  return deg;
}

static bool mpuWriteReg(uint8_t reg, uint8_t val)
{
  return HAL_I2C_Mem_Write(&IMU_I2C_HANDLE, IMU_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                           &val, 1u, IMU_I2C_TIMEOUT_MS) == HAL_OK;
}

static bool mpuReadGyroZRaw(int16_t *rawZ)
{
  uint8_t buf[2];
  if (HAL_I2C_Mem_Read(&IMU_I2C_HANDLE, IMU_I2C_ADDR, MPU6050_REG_GYRO_ZOUT_H, I2C_MEMADD_SIZE_8BIT,
                       buf, 2u, IMU_I2C_TIMEOUT_MS) != HAL_OK)
    return false;

  *rawZ = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
  return true;
}

bool imuInit(void)
{
  imuReady = false;
  gyroZBias = 0.0f;
  yawDeg = 0.0f;
  headingDeg = 0.0f;
  prevRateDps = 0.0f;

  if (HAL_I2C_IsDeviceReady(&IMU_I2C_HANDLE, IMU_I2C_ADDR, 3u, IMU_I2C_TIMEOUT_MS) != HAL_OK)
    return false;

  uint8_t whoAmI = 0u;
  if (HAL_I2C_Mem_Read(&IMU_I2C_HANDLE, IMU_I2C_ADDR, MPU6050_REG_WHO_AM_I, I2C_MEMADD_SIZE_8BIT,
                       &whoAmI, 1u, IMU_I2C_TIMEOUT_MS) != HAL_OK)
    return false;
  if (whoAmI == 0x00u || whoAmI == 0xFFu)
    return false;

  // Chọn nguồn clock PLL trục X (ổn định hơn RC nội 8 MHz) và đánh thức cảm biến
  if (!mpuWriteReg(MPU6050_REG_PWR_MGMT_1, 0x01u)) return false;
  HAL_Delay(50);

  // DLPF ~42 Hz lọc rung động cơ TT; tần số mẫu gyro nội 1 kHz / (1 + 4) = 200 Hz
  if (!mpuWriteReg(MPU6050_REG_CONFIG, 0x03u)) return false;
  if (!mpuWriteReg(MPU6050_REG_SMPLRT_DIV, 0x04u)) return false;
  // FS_SEL = 1 -> dải đo +-500 °/s (độ nhạy 65.5 LSB/(°/s))
  if (!mpuWriteReg(MPU6050_REG_GYRO_CONFIG, 0x08u)) return false;

  imuReady = true;
  imuResetYaw();
  return true;
}

bool imuIsReady(void)
{
  return imuReady;
}

void imuCalibrate(void)
{
  if (!imuReady)
    return;

  int32_t sum = 0;
  uint16_t validCount = 0u;

  for (uint16_t i = 0u; i < IMU_CALIB_SAMPLES; i++)
  {
    int16_t rawZ = 0;
    if (mpuReadGyroZRaw(&rawZ))
    {
      sum += (int32_t)rawZ;
      validCount++;
    }
    HAL_Delay(IMU_CALIB_DELAY_MS);
  }

  if (validCount > 0u)
    gyroZBias = (float)sum / (float)validCount;
  headingDeg = 0.0f;
  imuResetYaw();
}

void imuResetYaw(void)
{
  yawDeg = 0.0f;
  prevRateDps = 0.0f;
  lastCyccnt = DWT->CYCCNT;
}

float imuUpdate(void)
{
  if (!imuReady)
    return yawDeg;

  uint32_t nowCyc = DWT->CYCCNT;
  float dtSec = (float)(nowCyc - lastCyccnt) / (float)SystemCoreClock;
  lastCyccnt = nowCyc;

  if (dtSec <= 0.0f || dtSec > 0.25f)
    return yawDeg;

  int16_t rawZ = 0;
  if (!mpuReadGyroZRaw(&rawZ))
    return yawDeg;

  float rateDps = IMU_YAW_SIGN * (((float)rawZ - gyroZBias) / IMU_GYRO_SENSITIVITY);
  if (fabsf(rateDps) < IMU_GYRO_DEADBAND_DPS)
    rateDps = 0.0f;

  // Tích phân hình thang giúp giảm sai số trễ pha khi vận tốc góc thay đổi
  float deltaDeg = 0.5f * (prevRateDps + rateDps) * dtSec;
  yawDeg += deltaDeg;
  headingDeg = normalizeHeadingDeg(headingDeg + deltaDeg);
  prevRateDps = rateDps;
  return yawDeg;
}

float imuGetYawDeg(void)
{
  return yawDeg;
}

float imuGetHeadingDeg(void)
{
  return headingDeg;
}

float imuGetRateDps(void)
{
  return prevRateDps;
}
