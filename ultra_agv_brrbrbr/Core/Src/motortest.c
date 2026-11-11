#include "motortest.h"
#include "project_config.h"
#include "debug.h"
#include "motion.h"
#include "motor.h"
#include "sound_nav.h"
#include <stdio.h>

// Kịch bản test tìm ngưỡng PWM thấp nhất bánh vẫn xoay
void motorTestMinPwm(void)
{
  // 1. Chờ 3 giây để người dùng chuẩn bị và nhìn màn hình
  debugMotorPwmTest(0, 0, "WAIT 3S");
  HAL_Delay(MOTOR_TEST_START_DELAY_MS);

  // 2. Kích hoạt mức Max (650) để bánh xe chắc chắn quay và có trớn/đà
  uint16_t duty = PWM_TEST_MAX_DUTY;
  uint8_t percent = (uint8_t)((duty * 100u + 499u) / 999u);

  motorSetDutyRaw(MOTOR_LEFT, MOTOR_FORWARD, duty);
  motorSetDutyRaw(MOTOR_RIGHT, MOTOR_FORWARD, duty);
  debugMotorPwmTest(duty, percent, "MAX START");
  HAL_Delay(PWM_TEST_HOLD_MAX_MS);

  // 3. Giảm từ từ theo từng bước (mỗi bước giảm 10 duty ~ 1% PWM, giữ 600ms)
  while (duty > 0)
  {
    if (duty > PWM_TEST_STEP_DUTY)
    {
      duty -= PWM_TEST_STEP_DUTY;
    }
    else
    {
      duty = 0;
    }

    percent = (uint8_t)((duty * 100u + 499u) / 999u);

    if (duty > 0)
    {
      motorSetDutyRaw(MOTOR_LEFT, MOTOR_FORWARD, duty);
      motorSetDutyRaw(MOTOR_RIGHT, MOTOR_FORWARD, duty);
      debugMotorPwmTest(duty, percent, "RAMP DOWN");
    }
    else
    {
      motorStopAll();
      debugMotorPwmTest(0, 0, "ZERO PWM");
    }

    HAL_Delay(PWM_TEST_STEP_MS);
  }

  // 4. Kết thúc 1 chu kỳ, dừng động cơ và nghỉ 5s trước khi lặp lại chu kỳ mới
  motorStopAll();
  debugMotorPwmTest(0, 0, "FINISHED");
  HAL_Delay(PWM_TEST_CYCLE_PAUSE_MS);
}

// Kịch bản test tìm ngưỡng PWM thấp nhất khi XOAY TẠI CHỖ liên tục
void motorTestMinSpinPwm(void)
{
  static bool spinClockwise = true; // Đảo chiều giữa các chu kỳ để chống xoắn dây

  // 1. Chờ 3 giây để người dùng đặt xe xuống sàn và quan sát màn hình
  debugMotorPwmTest(0, 0, "WAIT 3S");
  HAL_Delay(MOTOR_TEST_START_DELAY_MS);

  // Xác định chiều quay của từng bánh để xe xoay tròn tại chỗ:
  // - Quay phải (CW): Bánh trái TIẾN, Bánh phải LÙI
  // - Quay trái (CCW): Bánh trái LÙI, Bánh phải TIẾN
  MotorDir dirLeft  = spinClockwise ? MOTOR_FORWARD : MOTOR_BACKWARD;
  MotorDir dirRight = spinClockwise ? MOTOR_BACKWARD : MOTOR_FORWARD;
  const char *spinLabel = spinClockwise ? "SPIN CW" : "SPIN CCW";

  // 2. Kích hoạt mức Max (650) để xe có trớn xoay tròn tại chỗ mạnh mẽ
  uint16_t duty = PWM_TEST_MAX_DUTY;
  uint8_t percent = (uint8_t)((duty * 100u + 499u) / 999u);

  motorSetDutyRaw(MOTOR_LEFT, dirLeft, duty);
  motorSetDutyRaw(MOTOR_RIGHT, dirRight, duty);
  debugMotorPwmTest(duty, percent, spinLabel);
  HAL_Delay(PWM_TEST_HOLD_MAX_MS);

  // 3. Giảm từ từ từng bước (mỗi bước giảm 10 duty ~ 1% PWM, giữ 600ms)
  while (duty > 0)
  {
    if (duty > PWM_TEST_STEP_DUTY)
    {
      duty -= PWM_TEST_STEP_DUTY;
    }
    else
    {
      duty = 0;
    }

    percent = (uint8_t)((duty * 100u + 499u) / 999u);

    if (duty > 0)
    {
      motorSetDutyRaw(MOTOR_LEFT, dirLeft, duty);
      motorSetDutyRaw(MOTOR_RIGHT, dirRight, duty);
      debugMotorPwmTest(duty, percent, spinLabel);
    }
    else
    {
      motorStopAll();
      debugMotorPwmTest(0, 0, "ZERO PWM");
    }

    HAL_Delay(PWM_TEST_STEP_MS);
  }

  // 4. Kết thúc 1 chu kỳ xoay, dừng và nghỉ 5s
  motorStopAll();
  debugMotorPwmTest(0, 0, "FINISHED");
  HAL_Delay(PWM_TEST_CYCLE_PAUSE_MS);

  // Đổi chiều quay cho chu kỳ sau (vừa test cả 2 chiều, vừa chống rối dây)
  spinClockwise = !spinClockwise;
}

// Kịch bản test kích hoạt trực tiếp Duty = 350 từ trạng thái đứng yên
void motorTestDirectSpin350(void)
{
  static bool spinClockwise = true;

  // 1. Dừng hẳn và chờ 3 giây
  motorStopAll();
  debugMotorPwmTest(0, 0, "WAIT 3S");
  HAL_Delay(MOTOR_TEST_START_DELAY_MS);

  // 2. Kích hoạt trực tiếp ở mức 350 không có trớn/đà
  uint16_t duty = 350;
  uint8_t percent = 35;

  MotorDir dirLeft  = spinClockwise ? MOTOR_FORWARD : MOTOR_BACKWARD;
  MotorDir dirRight = spinClockwise ? MOTOR_BACKWARD : MOTOR_FORWARD;
  const char *spinLabel = spinClockwise ? "SPIN CW" : "SPIN CCW";

  motorSetDutyRaw(MOTOR_LEFT, dirLeft, duty);
  motorSetDutyRaw(MOTOR_RIGHT, dirRight, duty);
  debugMotorPwmTest(duty, percent, spinLabel);

  // Giữ mức 350 trong 5 giây để quan sát xem xe có xoay được không
  HAL_Delay(5000);

  // 3. Dừng nghỉ 3 giây trước khi đổi chiều ở chu kỳ sau
  motorStopAll();
  debugMotorPwmTest(0, 0, "STOPPED");
  HAL_Delay(3000);

  spinClockwise = !spinClockwise;
}

// Xoay liên tục không ngừng ở tốc độ thấp nhất có thể (Duty = 350 ~ 35%)
void motorSpinContinuousLowestSpeed(void)
{
  // 1. Chờ 3 giây để người dùng đặt xe xuống sàn và chuẩn bị quan sát
  debugMotorPwmTest(0, 0, "WAIT 3S");
  HAL_Delay(3000);

  // 2. Xung mồi kickstart 40ms ở mức 450 phá ma sát nghỉ ban đầu
  MotorDir dirLeft  = MOTOR_FORWARD;
  MotorDir dirRight = MOTOR_BACKWARD;
  motorSetDutyRaw(MOTOR_LEFT, dirLeft, SOUND_KICK_DUTY);
  motorSetDutyRaw(MOTOR_RIGHT, dirRight, SOUND_KICK_DUTY);
  HAL_Delay(SOUND_KICK_TIME_MS);

  // 3. Ghim cố định ở tốc độ thấp nhất (Duty 350 ~ 35%) và duy trì xoay liên tục
  uint16_t duty = SOUND_SPIN_DUTY_MIN;
  uint8_t percent = (uint8_t)((duty * 100u + 499u) / 999u);
  motorSetDutyRaw(MOTOR_LEFT, dirLeft, duty);
  motorSetDutyRaw(MOTOR_RIGHT, dirRight, duty);
  debugMotorPwmTest(duty, percent, "SPIN MIN 350");
  printf("MOTOR: Xoay lien tuc o toc do thap nhat: Duty = %u (%u%%)\r\n", duty, percent);

  // Giữ trạng thái xoay liên tục mãi mãi, không ngắt quãng
  while (1)
  {
    HAL_Delay(1000);
  }
}

void motorTestRun(void)
{
  // Chạy test kích hoạt trực tiếp Duty = 350
  motorTestDirectSpin350();
}
