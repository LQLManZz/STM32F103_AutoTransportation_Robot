#include "grabber.h"
#include "debug.h"
#include "project_config.h"
#include <stdbool.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

static uint8_t gripAngleDeg = GRABBER_GRIP_OPEN_DEG;
static uint8_t liftAngleDeg = GRABBER_LIFT_UP_DEG;
static bool gripPwmOn = false;
static bool liftPwmOn = false;

// Kẹp góc servo trong giới hạn 0°..180°
static uint8_t clampAngle(uint8_t angleDeg)
{
  if (angleDeg > GRABBER_MAX_ANGLE_DEG)
    return GRABBER_MAX_ANGLE_DEG;
  return angleDeg;
}

// Quy đổi góc (độ) sang độ rộng xung PWM (us)
static uint32_t angleToPulseUs(uint8_t angleDeg)
{
  angleDeg = clampAngle(angleDeg);
  return GRABBER_MIN_PULSE_US +
         ((uint32_t)angleDeg * (GRABBER_MAX_PULSE_US - GRABBER_MIN_PULSE_US)) / GRABBER_MAX_ANGLE_DEG;
}

// Ngắt PWM một kênh servo sau khi kết thúc cửa sổ xung HIGH để tránh giật biên
static void stopServoChannelClean(TIM_HandleTypeDef *htim, uint32_t channel)
{
  if ((htim->Instance->CR1 & TIM_CR1_CEN) != 0u)
  {
    uint32_t startMs = HAL_GetTick();
    while (__HAL_TIM_GET_COUNTER(htim) <= (uint32_t)(GRABBER_MAX_PULSE_US + 100u))
    {
      if ((HAL_GetTick() - startMs) >= 25u)
        break;
    }
  }

  __HAL_TIM_SET_COMPARE(htim, channel, 0u);
  HAL_TIM_PWM_Stop(htim, channel);
}

void grabberInit(void)
{
  // Đưa tuần tự từng servo về vị trí chờ vật lý (mở ngàm -> nâng cần) rồi ngắt PWM tránh sụt áp
  gripAngleDeg = GRABBER_GRIP_OPEN_DEG;
  liftAngleDeg = GRABBER_LIFT_UP_DEG;
  gripPwmOn = false;
  liftPwmOn = false;
  debugGrabber(gripAngleDeg, liftAngleDeg, "GRAB_INIT", "HOMING");

  grabberSetGripAngle(GRABBER_GRIP_OPEN_DEG);
  HAL_Delay(GRABBER_INIT_STEP_MS);
  grabberGripStop();

  grabberSetLiftAngle(GRABBER_LIFT_UP_DEG);
  HAL_Delay(GRABBER_INIT_STEP_MS);
  grabberLiftStop();
  debugGrabber(gripAngleDeg, liftAngleDeg, "GRAB_INIT", "OFF");
}

void grabberSetGripAngle(uint8_t angleDeg)
{
  angleDeg = clampAngle(angleDeg);
  gripAngleDeg = angleDeg;
  __HAL_TIM_SET_COMPARE(&GRABBER_GRIP_TIM, GRABBER_GRIP_CHANNEL, angleToPulseUs(angleDeg));
  if (!gripPwmOn)
  {
    HAL_TIM_PWM_Start(&GRABBER_GRIP_TIM, GRABBER_GRIP_CHANNEL);
    gripPwmOn = true;
  }
}

void grabberSetLiftAngle(uint8_t angleDeg)
{
  angleDeg = clampAngle(angleDeg);
  liftAngleDeg = angleDeg;
  __HAL_TIM_SET_COMPARE(&GRABBER_LIFT_TIM, GRABBER_LIFT_CHANNEL, angleToPulseUs(angleDeg));
  if (!liftPwmOn)
  {
    HAL_TIM_PWM_Start(&GRABBER_LIFT_TIM, GRABBER_LIFT_CHANNEL);
    liftPwmOn = true;
  }
}

void grabberGripStop(void)
{
  if (!gripPwmOn)
    return;
  stopServoChannelClean(&GRABBER_GRIP_TIM, GRABBER_GRIP_CHANNEL);
  gripPwmOn = false;
}

void grabberLiftStop(void)
{
  if (!liftPwmOn)
    return;
  stopServoChannelClean(&GRABBER_LIFT_TIM, GRABBER_LIFT_CHANNEL);
  liftPwmOn = false;
}

void grabberStopAll(void)
{
  grabberGripStop();
  grabberLiftStop();
}

void grabberMoveGripSmooth(uint8_t targetDeg)
{
  targetDeg = clampAngle(targetDeg);
  int16_t angle = (int16_t)gripAngleDeg;

  while (angle != (int16_t)targetDeg)
  {
    int16_t diff = (int16_t)targetDeg - angle;
    if (diff > (int16_t)GRABBER_SMOOTH_STEP_DEG)
      angle += (int16_t)GRABBER_SMOOTH_STEP_DEG;
    else if (diff < -(int16_t)GRABBER_SMOOTH_STEP_DEG)
      angle -= (int16_t)GRABBER_SMOOTH_STEP_DEG;
    else
      angle = (int16_t)targetDeg;

    grabberSetGripAngle((uint8_t)angle);
    HAL_Delay(GRABBER_SMOOTH_DELAY_MS);
  }
}

void grabberMoveLiftSmooth(uint8_t targetDeg)
{
  targetDeg = clampAngle(targetDeg);
  int16_t angle = (int16_t)liftAngleDeg;

  while (angle != (int16_t)targetDeg)
  {
    int16_t diff = (int16_t)targetDeg - angle;
    if (diff > (int16_t)GRABBER_SMOOTH_STEP_DEG)
      angle += (int16_t)GRABBER_SMOOTH_STEP_DEG;
    else if (diff < -(int16_t)GRABBER_SMOOTH_STEP_DEG)
      angle -= (int16_t)GRABBER_SMOOTH_STEP_DEG;
    else
      angle = (int16_t)targetDeg;

    grabberSetLiftAngle((uint8_t)angle);
    HAL_Delay(GRABBER_SMOOTH_DELAY_MS);
  }
}

uint8_t grabberGetGripAngle(void)
{
  return gripAngleDeg;
}

uint8_t grabberGetLiftAngle(void)
{
  return liftAngleDeg;
}

void grabberPick(void)
{
  // Bảo đảm ngàm mở sẵn rồi tắt ngay kẹp trước khi hạ cần
  grabberSetGripAngle(GRABBER_GRIP_OPEN_DEG);
  HAL_Delay(GRABBER_ACTION_SETTLE_MS);
  grabberGripStop();

  debugGrabber(gripAngleDeg, liftAngleDeg, "GRAB_PICK", "LIFT_DOWN");
  grabberMoveLiftSmooth(GRABBER_LIFT_DOWN_DEG);
  HAL_Delay(GRABBER_ACTION_SETTLE_MS);

  debugGrabber(gripAngleDeg, liftAngleDeg, "GRAB_PICK", "GRIP_CLOSE");
  grabberMoveGripSmooth(GRABBER_GRIP_CLOSE_DEG);
  HAL_Delay(GRABBER_ACTION_SETTLE_MS);

  debugGrabber(gripAngleDeg, liftAngleDeg, "GRAB_PICK", "LIFT_UP");
  grabberMoveLiftSmooth(GRABBER_LIFT_UP_DEG);
  HAL_Delay(GRABBER_ACTION_SETTLE_MS);

  // Gắp xong cắt toàn bộ PWM tay gắp về trạng thái tắt
  grabberStopAll();
  debugGrabber(gripAngleDeg, liftAngleDeg, "GRAB_PICK", "DONE_OFF");
}

void grabberRelease(void)
{
  debugGrabber(gripAngleDeg, liftAngleDeg, "GRAB_DROP", "LIFT_DOWN");
  grabberMoveLiftSmooth(GRABBER_LIFT_DOWN_DEG);
  HAL_Delay(GRABBER_ACTION_SETTLE_MS);

  debugGrabber(gripAngleDeg, liftAngleDeg, "GRAB_DROP", "GRIP_OPEN");
  grabberMoveGripSmooth(GRABBER_GRIP_OPEN_DEG);
  HAL_Delay(GRABBER_ACTION_SETTLE_MS);
  grabberGripStop();

  debugGrabber(gripAngleDeg, liftAngleDeg, "GRAB_DROP", "LIFT_UP");
  grabberMoveLiftSmooth(GRABBER_LIFT_UP_DEG);
  HAL_Delay(GRABBER_ACTION_SETTLE_MS);
  grabberStopAll();
  debugGrabber(gripAngleDeg, liftAngleDeg, "GRAB_DROP", "RELEASED");
}

void grabberTestCycle(void)
{
  HAL_Delay(GRABBER_TEST_WAIT_MS);
  grabberPick();
  HAL_Delay(GRABBER_TEST_HOLD_MS);
  grabberRelease();
}
