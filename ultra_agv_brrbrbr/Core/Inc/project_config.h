#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

#include "main.h"

// --- Scanner (SRF05 + servo SG90) ---
#define TRIGGER_PIN Trig_SRF05_Pin        // PB15 (Output) - phát xung Trigger
#define TRIGGER_PORT Trig_SRF05_GPIO_Port // GPIOB
#define ECHO_PIN Echo_SRF05_Pin           // PA8 (Input) - nhận xung Echo
#define ECHO_PORT Echo_SRF05_GPIO_Port    // GPIOA

#define SERVO_MIN_PULSE_US 600u   // Độ rộng xung ứng với 0° (us), tick timer = 1 us
#define SERVO_MAX_PULSE_US 2400u  // Độ rộng xung ứng với 180° (us)
#define SERVO_SMOOTH_STEP_DEG 1u  // Góc mỗi bước khi quay dần (độ)
#define SERVO_SMOOTH_DELAY_MS 15u // Chờ giữa 2 bước (ms)

#define SCAN_MIN_RANGE_CM 3.0f  // Khoảng cách tối thiểu (cm), gần hơn coi như không có vật
#define SCAN_MAX_RANGE_CM 50.0f // Bán kính quét tối đa (cm)
#define SCAN_DWELL_MS 3000u     // Thời gian quét tại mỗi vị trí (ms)
#define SCAN_STEP_DEG 45u       // Góc đổi vị trí mỗi lần
#define SCAN_HOME_ANGLE 90u     // Vị trí ban đầu của servo
#define SCAN_SETTLE_MS 400u     // Chờ servo quay xong (ms)
#define SCAN_PERIOD_MS 60u      // Chu kỳ giữa 2 lần đo và in (ms), SRF05 cần >= 50
#define SCAN_FOUND_HOLD_MS 1500u // Dừng sau khi thấy vật (ms)

#define SOUND_SPEED_CM_US 0.0343f   // Tốc độ âm thanh (cm/us)
#define MAX_ECHO_TIMEOUT_US 30000UL // Thời gian chờ echo tối đa (us)

// --- Màn hình OLED SSD1306 128x64 (I2C2: SCL = PB10, SDA = PB11) ---
#define OLED_I2C_ADDR (0x3Cu << 1) // Địa chỉ 7 bit 0x3C, HAL cần dạng dịch trái 1 bit
#define OLED_I2C_TIMEOUT_MS 50u    // Chờ tối đa mỗi lần gửi (ms), OLED không gắn thì không treo
#define OLED_LINES 4u              // Số dòng chữ (mỗi dòng cao 2 page vì chữ phóng đôi)
#define OLED_LINE_CHARS 10u        // Số ký tự mỗi dòng: 128 px / 12 px (chữ phóng đôi)

// --- Động cơ (L298N, PWM TIM3 CH1 = ENA/PA6, CH2 = ENB/PA7) ---
#define MOTOR_PORT_DIR GPIOA
#define MOTOR_PIN_IN1 motor_IN1_Pin // PA2 - IN1 L298N (bánh phải)
#define MOTOR_PIN_IN2 motor_IN2_Pin // PA3 - IN2 L298N (bánh phải)
#define MOTOR_PIN_IN3 motor_IN3_Pin // PA4 - IN3 L298N (bánh trái)
#define MOTOR_PIN_IN4 motor_IN4_Pin // PA5 - IN4 L298N (bánh trái)

#define MOTOR_PWM_MAX_DUTY 450 // Trần an toàn cho TT Motor trên pin 12V (ARR TIM3 = 999)
#define MOTOR_PWM_MIN_DUTY 200 // Ngưỡng thắng ma sát tĩnh

#define SPIN_MS_PER_DEGREE 4.8f // Thời gian xoay 1° ở SPEED_SPIN_CALM (ms/độ), đo thực tế
#define SPEED_SPIN_CALM 26      // Tốc độ xoay tại chỗ (% PWM), chậm để không trôi quá góc
#define DRIVE_MS_PER_CM 45.0f   // Thời gian chạy 1 cm ở 40% PWM (ms/cm)

// Bù tốc độ để xe chạy thẳng: xe lệch phải thì tăng bánh phải, lệch trái thì giảm
#define MOTOR_TRIM_LEFT_OFFSET 0  // % PWM cộng thêm cho bánh trái
#define MOTOR_TRIM_RIGHT_OFFSET 4 // % PWM cộng thêm cho bánh phải

#define RAMP_START_SPEED 20   // Tốc độ bắt đầu tăng tốc (% PWM), đủ thắng ma sát tĩnh
#define RAMP_STEP_DELAY_MS 30 // Thời gian mỗi bước tăng/giảm tốc (ms)

// --- Tiếp cận vật ---
#define AGV_DRIVE_SPEED 40 // Tốc độ chạy tới vật (% PWM)
#define AGV_RAM_EXTRA_CM 3 // Chạy thêm sau khi tới vị trí đo được (cm), bù sai số thời gian để chắc chắn chạm vật
#define AGV_STEP_MAX_CM 10 // Mỗi đoạn chạy tối đa (cm), hết đoạn đo lại khoảng cách thật để chống sai số chạy theo thời gian

#endif
