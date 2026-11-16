#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

#include "main.h"

// --- Cụm 2 cảm biến siêu âm (SRF05 dưới + SRF04 trên) & Servo SG90 ---
#define TRIGGER_PIN Trig_SRF05_Pin           // PB15 (Output) - phát xung Trigger chung cho cả 2 cảm biến
#define TRIGGER_PORT Trig_SRF05_GPIO_Port    // GPIOB
#define ECHO_PIN Echo_SRF05_Pin              // PA8 (Input) - nhận xung Echo SRF05 (cảm biến dưới)
#define ECHO_PORT Echo_SRF05_GPIO_Port       // GPIOA
#define ECHO_SRF04_PIN Echo_SRF04_Pin        // PA12 (Input) - nhận xung Echo SRF04 (cảm biến trên)
#define ECHO_SRF04_PORT Echo_SRF04_GPIO_Port // GPIOA

#define SENSOR_DEPTH_OFFSET_CM 7.0f      // Chênh lệch khoảng cách: SRF04 nằm lùi về phía sau 7 cm so với SRF05
#define SCAN_OBSTACLE_TOLERANCE_CM 8.0f  // Dung sai độ lệch (sau khi bù 7 cm) để kết luận vật cản

#define SERVO_MIN_PULSE_US 600u   // Độ rộng xung ứng với 0° (us), tick timer = 1 us
#define SERVO_MAX_PULSE_US 2400u  // Độ rộng xung ứng với 180° (us)
#define SERVO_MAX_ANGLE_DEG 180u  // Góc quay tối đa của servo (độ)
#define SERVO_SMOOTH_STEP_DEG 1u  // Góc mỗi bước khi quay dần (độ)
#define SERVO_SMOOTH_DELAY_MS 15u // Chờ giữa 2 bước (ms)

#define SCAN_MIN_RANGE_CM 3.0f   // Khoảng cách tối thiểu (cm), gần hơn coi như không có vật
#define SCAN_MAX_RANGE_CM 50.0f  // Bán kính quét tối đa (cm)
#define SCAN_DWELL_MS 3000u      // Thời gian quét tại mỗi vị trí (ms)
#define SCAN_STEP_DEG 45u        // Góc đổi vị trí mỗi lần
#define SCAN_HOME_ANGLE 90u      // Vị trí ban đầu của servo (thẳng mũi xe)
#define SCAN_SETTLE_MS 400u      // Chờ servo quay xong (ms)
#define SCAN_PERIOD_MS 60u       // Chu kỳ giữa 2 lần đo và in (ms), SRF05/SRF04 cần >= 50
#define SCAN_FOUND_HOLD_MS 1500u // Dừng sau khi thấy vật (ms)
#define SCAN_CONFIRM_COUNT 2u    // Số lần đo liên tiếp thấy vật cần gắp mới xác nhận

#define SCAN_FINE_MIN_DEG 70u   // Góc bắt đầu quét hẹp vi chỉnh quanh HOME
#define SCAN_FINE_MAX_DEG 110u  // Góc kết thúc quét hẹp vi chỉnh
#define SCAN_FINE_STEP_DEG 5u   // Bước góc quét hẹp (độ)
#define SCAN_FINE_SETTLE_MS 70u // Thời gian chờ servo ở mỗi bước quét hẹp (ms)
#define AGV_FINE_DEADBAND_DEG 2 // Sai lệch góc <= giá trị này coi như đã thẳng

#define TRIGGER_RESET_US 4u         // Thời gian giữ Trigger mức LOW trước khi phát xung (us)
#define TRIGGER_PULSE_US 10u        // Độ rộng xung Trigger yêu cầu >= 10 us
#define ECHO_MIN_NOISE_US 50UL      // Bỏ qua xung nhiễu ngắn hơn 50 us
#define ECHO_MIN_VALID_US 116UL     // 116 us ~ 2 cm: dưới ngưỡng đo tin cậy
#define ECHO_ERROR_TIMEOUT -1.0f    // Mã lỗi không nhận được xung phản hồi (timeout)
#define ECHO_ERROR_TOO_CLOSE -2.0f  // Mã lỗi vật quá gần (< 2 cm)
#define SOUND_SPEED_CM_US 0.0343f   // Tốc độ âm thanh (cm/us)
#define MAX_ECHO_TIMEOUT_US 30000UL // Thời gian chờ echo tối đa (us)

// --- Màn hình OLED SSD1306 128x64 (I2C2: SCL = PB10, SDA = PB11) ---
#define OLED_I2C_ADDR (0x3Cu << 1) // Địa chỉ 7 bit 0x3C, HAL cần dạng dịch trái 1 bit
#define OLED_I2C_TIMEOUT_MS 50u    // Chờ tối đa mỗi lần gửi (ms), OLED không gắn thì không treo
#define OLED_WIDTH_PX 128u         // Chiều ngang màn hình OLED (px)
#define OLED_LINES 4u              // Số dòng chữ (mỗi dòng cao 2 page)
#define OLED_LINE_CHARS 21u        // Số ký tự mỗi dòng: 128 px / 6 px (rộng 1x, cao 2x)

// --- Động cơ (L298N, PWM TIM3 CH1 = ENA/PA6, CH2 = ENB/PA7) ---
#define MOTOR_PORT_DIR GPIOA
#define MOTOR_PIN_IN2 motor_IN1_Pin // PA2 - IN1 L298N (bánh phải)
#define MOTOR_PIN_IN1 motor_IN2_Pin // PA3 - IN2 L298N (bánh phải)
#define MOTOR_PIN_IN3 motor_IN3_Pin // PA4 - IN3 L298N (bánh trái)
#define MOTOR_PIN_IN4 motor_IN4_Pin // PA5 - IN4 L298N (bánh trái)

#define MOTOR_PWM_ARR_MAX 999u // Giá trị ARR của TIM3
#define MOTOR_PWM_MAX_DUTY 650 // Trần an toàn cho TT Motor trên pin 12V (ARR TIM3 = 999)
#define MOTOR_PWM_MIN_DUTY 300 // Ngưỡng thắng ma sát tĩnh
#define MOTOR_LEFT_SCALE 147u  // Tỉ lệ cân bằng PWM bánh trái (chuẩn tham chiếu)
#define MOTOR_RIGHT_SCALE 169u // Tỉ lệ cân bằng PWM bánh phải (bánh phải cần duty cao hơn theo tỉ lệ 169/147)

#define SPIN_MS_PER_DEGREE 4.4f   // Thời gian xoay 1° ở SPEED_SPIN_CALM (ms/độ), đo từ video: xe quay ~227°/s
#define SPIN_OFFSET_MS 70.0f      // Thời gian xoay cộng cố định (ms), bù độ trễ tăng tốc/phanh
#define SPIN_BRAKE_DELAY_MS 100u  // Thời gian giữ phanh ngắn mạch 100% sau khi xoay góc (ms)
#define SPEED_SPIN_CALM 50        // Tốc độ xoay tại chỗ (% PWM), chậm để không trôi quá góc
#define DRIVE_MS_PER_CM 45.0f     // Thời gian chạy 1 cm ở 40% PWM (ms/cm)
#define DRIVE_BRAKE_DELAY_MS 100u // Thời gian giữ phanh ngắn mạch 100% sau khi chạy thẳng (ms)
#define BRAKE_REV_DUTY 450u       // Duty xung phanh đảo chiều ngắn triệt tiêu quán tính (0 - 999)
#define BRAKE_REV_PULSE_MS 25u    // Độ rộng xung phanh đảo chiều (ms), đủ triệt đà mà không lùi xe

#define RAMP_START_SPEED 20    // Tốc độ bắt đầu tăng tốc (% PWM), đủ thắng ma sát tĩnh
#define RAMP_STEP_DELAY_MS 30  // Thời gian mỗi bước tăng/giảm tốc (ms)
#define RAMP_UP_MS 240u        // Thời gian tăng tốc đầu hành trình (ms)
#define RAMP_DOWN_MS 210u      // Thời gian giảm tốc cuối hành trình (ms)
#define RAMP_MIN_TOTAL_MS 500u // Hành trình ngắn hơn ngưỡng này thì chạy trực tiếp không ramp (ms)

// --- Tiếp cận vật (tạm ngắt PID, dùng tốc độ cố định) ---
#define AGV_APPROACH_SPEED 30u     // Tốc độ tiến cố định (% PWM) khi tạm ngắt PID
#define PID_DIST_TARGET_CM 3.5f    // Cự ly dừng mong muốn trước mũi xe (cm)
#define PID_DIST_TOLERANCE_CM 0.5f // Dung sai dừng (cm): tới đích khi cự ly <= đích + dung sai
#define PID_SAMPLE_TIME_MS 60u     // Chu kỳ đo SRF05/SRF04 (ms, cần >= 50)
#define PID_TIMEOUT_MS 6000u       // Timeout tiếp cận tối đa (ms)
#define PID_LOST_MAX 3u            // Số lần mất vật liên tiếp thì bỏ cuộc

// --- Xử lý khi phát hiện tường (vật cản cao >= 11 cm) ---
#define WALL_STOP_DIST_CM 10.0f    // Cự ly dừng sát tường (cm)
#define WALL_CHECK_LEFT_ANGLE 180u // Góc servo quay sang trái kiểm tra tường (độ)
#define WALL_CHECK_RIGHT_ANGLE 0u  // Góc servo quay sang phải kiểm tra tường (độ)
#define WALL_CHECK_SAMPLES 3u      // Số lần đo kiểm tra tường mỗi bên trái/phải
#define WALL_TURN_LEFT_DEG -90     // Góc quay trái tránh tường (độ)
#define WALL_TURN_RIGHT_DEG 90     // Góc quay phải tránh tường (độ)
#define WALL_TURN_BACK_DEG 180     // Góc quay đầu khi cả trái và phải đều có tường (độ)
#define WALL_HOLD_MS 800u          // Thời gian giữ hiển thị trạng thái tường trên OLED (ms)

// --- Kịch bản test động cơ & dò ngưỡng PWM ---
#define MOTOR_TEST_START_DELAY_MS 3000u // Chờ trước khi chạy để đặt xe xuống sàn (ms)
#define MOTOR_TEST_PAUSE_MS 1500u       // Nghỉ giữa 2 động tác (ms)
#define MOTOR_TEST_DRIVE_CM 30          // Cự ly tiến/lùi (cm)
#define MOTOR_TEST_DRIVE_SPEED 40       // Tốc độ tiến/lùi (% PWM)

#define PWM_TEST_MAX_DUTY 650u        // Duty bắt đầu (trần an toàn pin 12V cho motor TT, ARR = 999)
#define PWM_TEST_HOLD_MAX_MS 2500u    // Thời gian giữ mức Max để bánh lấy trớn/đà (ms)
#define PWM_TEST_STEP_DUTY 10u        // Bước giảm Duty mỗi lần (~1% PWM)
#define PWM_TEST_STEP_MS 600u         // Thời gian giữ mỗi bước giảm để kịp quan sát (ms)
#define PWM_TEST_CYCLE_PAUSE_MS 5000u  // Nghỉ giữa 2 chu kỳ test lặp lại (ms)
#define PWM_TEST_DRIVE_LEFT_DUTY 147u  // Duty tiến thẳng bánh trái
#define PWM_TEST_DRIVE_RIGHT_DUTY 169u // Duty tiến thẳng bánh phải
#define PWM_TEST_SPIN_MIN_DUTY 350u    // Duty xoay tại chỗ thấp nhất (35%)
#define PWM_TEST_KICK_DUTY 450u       // Xung mồi kickstart phá ma sát nghỉ (45%)
#define PWM_TEST_KICK_MS 40u          // Thời gian xung mồi (ms)

#endif // PROJECT_CONFIG_H
