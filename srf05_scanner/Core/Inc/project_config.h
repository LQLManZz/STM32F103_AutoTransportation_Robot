#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

#include "main.h"

#define TRIGGER_PIN Trigger_Pin        // PB15 (Output) - phát xung Trigger
#define TRIGGER_PORT Trigger_GPIO_Port // GPIOB
#define ECHO_PIN Echo_Pin              // PA8 (Input) - nhận xung Echo
#define ECHO_PORT Echo_GPIO_Port       // GPIOA
#define SERVO_PIN GPIO_PIN_1           // PA1 (TIM2_CH2 PWM) - xung điều khiển servo
#define SERVO_PORT GPIOA               // GPIOA

#define SERVO_MIN_PULSE_US 500u  // Độ rộng xung ứng với 0° (us), tick timer = 1 us
#define SERVO_MAX_PULSE_US 2500u // Độ rộng xung ứng với 180° (us)

#define SCAN_MIN_RANGE_CM 3.0f  // Khoảng cách tối thiểu (cm), gần hơn coi như không có vật
#define SCAN_MAX_RANGE_CM 50.0f // Bán kính quét tối đa (cm)
#define SCAN_DWELL_MS 2000u     // Thời gian quét tại mỗi vị trí (ms)
#define SCAN_STEP_DEG 45u       // Góc đổi vị trí mỗi lần
#define SCAN_HOME_ANGLE 90u     // Vị trí ban đầu của servo
#define SCAN_SETTLE_MS 400u     // Chờ servo quay xong (ms)
#define SCAN_PERIOD_MS 60u      // Chu kỳ giữa 2 lần đo (ms), SRF05 cần >= 50

#define SOUND_SPEED_CM_US 0.0343f   // Tốc độ âm thanh (cm/us)
#define MAX_ECHO_TIMEOUT_US 30000UL // Thời gian chờ echo tối đa của chân Echo 30us

#endif
