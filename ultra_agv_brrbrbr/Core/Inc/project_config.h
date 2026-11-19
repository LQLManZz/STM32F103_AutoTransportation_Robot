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
#define SCAN_OBSTACLE_TOLERANCE_CM 12.0f // Dung sai độ lệch (sau khi bù 7 cm) để kết luận vật cản
#define SCAN_CLUSTER_DEPTH_JUMP_CM 12.0f // Ngưỡng nhảy khoảng cách d05 để tách riêng 2 vật gần/xa (cm)

#define SERVO_MIN_PULSE_US 600u   // Độ rộng xung ứng với 0° (us), tick timer = 1 us
#define SERVO_MAX_PULSE_US 2400u  // Độ rộng xung ứng với 180° (us)
#define SERVO_MAX_ANGLE_DEG 180u  // Góc quay tối đa của servo (độ)
#define SERVO_SMOOTH_STEP_DEG 1u  // Góc mỗi bước khi quay dần (độ)
#define SERVO_SMOOTH_DELAY_MS 15u // Chờ giữa 2 bước (ms)

#define SCAN_MIN_RANGE_CM 3.0f   // Khoảng cách tối thiểu (cm), gần hơn coi như không có vật
#define SCAN_MAX_RANGE_CM 80.0f  // Bán kính quét tối đa cho sân 2m x 2m (cm)
#define SCAN_STEP_DEG 4u         // Bước quét mịn 4° để bắt trọn từ mép đầu tới mép cuối vật
#define SCAN_HOME_ANGLE 90u      // Vị trí ban đầu của servo (thẳng mũi xe)
#define SCAN_SETTLE_MS 180u      // Chờ servo ổn định sau khi quay mượt (ms)
#define SCAN_PERIOD_MS 60u       // Chu kỳ giữa 2 lần đo và in (ms), SRF05/SRF04 cần >= 50
#define SCAN_FOUND_HOLD_MS 500u  // Dừng xác nhận sau khi tìm thấy trung tâm vật (ms)
#define SCAN_CONFIRM_COUNT 2u    // Số bước liên tiếp thấy vật để xác nhận không phải nhiễu

#define SCAN_FINE_MIN_DEG 60u   // Góc bắt đầu quét hẹp vi chỉnh quanh HOME
#define SCAN_FINE_MAX_DEG 120u  // Góc kết thúc quét hẹp vi chỉnh
#define SCAN_FINE_STEP_DEG 4u   // Bước góc quét hẹp (độ)
#define SCAN_FINE_SETTLE_MS 70u // Thời gian chờ servo ở mỗi bước quét (ms)
#define AGV_FINE_DEADBAND_DEG 2 // Sai lệch góc <= giá trị này coi như đã thẳng

#define TRIGGER_RESET_US 4u         // Thời gian giữ Trigger mức LOW trước khi phát xung (us)
#define TRIGGER_PULSE_US 10u        // Độ rộng xung Trigger yêu cầu >= 10 us
#define ECHO_MIN_NOISE_US 50UL      // Bỏ qua xung nhiễu ngắn hơn 50 us
#define ECHO_MIN_VALID_US 116UL     // 116 us ~ 2 cm: dưới ngưỡng đo tin cậy
#define ECHO_ERROR_TIMEOUT -1.0f    // Mã lỗi không nhận được xung phản hồi (timeout)
#define ECHO_ERROR_TOO_CLOSE -2.0f  // Mã lỗi vật quá gần (< 2 cm)
#define SOUND_SPEED_CM_US 0.0343f   // Tốc độ âm thanh (cm/us)
#define MAX_ECHO_TIMEOUT_US 12000UL // Chờ echo tối đa 12 ms (~206 cm), quét nhanh & đủ cho tầm 80 cm

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

// --- Tiếp cận vật bằng PID + 2 encoder & quy đổi tọa độ tâm vật ---
#define SERVO_PIVOT_OFFSET_CM 6.5f // Khoảng cách từ tâm trục 2 bánh tới trục servo SG90 (cm)
#define SENSOR_FACE_OFFSET_CM 3.5f // Khoảng cách từ trục servo tới mặt đầu dò SRF05 trên ngàm (cm)
#define OBJ_CENTER_OFFSET_CM 1.0f  // Nửa bề dày vật cần gắp để tính tọa độ tâm vật (cm)
#define AGV_APPROACH_SPEED 30u     // Tốc độ tiến cố định (% PWM) khi tiến tới tường
#define PID_DIST_TARGET_CM 3.5f    // Cự ly dừng mong muốn trước mặt vật tính từ đầu dò (cm)
#define PID_DIST_TOLERANCE_CM 0.5f // Dung sai dừng tường (cm)
#define PID_SAMPLE_TIME_MS 60u     // Chu kỳ đo SRF05/SRF04 (ms, cần >= 50)
#define PID_TIMEOUT_MS 8000u       // Timeout tiếp cận tối đa (ms)
#define PID_LOST_MAX 3u            // Số lần mất tường liên tiếp thì bỏ cuộc

// --- Xử lý khi phát hiện tường (vật cản cao >= 11 cm) ---
#define WALL_STOP_DIST_CM 10.0f    // Cự ly dừng sát tường (cm)
#define WALL_CHECK_LEFT_ANGLE 180u // Góc servo quay sang trái kiểm tra tường (độ)
#define WALL_CHECK_RIGHT_ANGLE 0u  // Góc servo quay sang phải kiểm tra tường (độ)
#define WALL_CHECK_SAMPLES 3u      // Số lần đo kiểm tra tường mỗi bên trái/phải
#define WALL_TURN_LEFT_DEG -90     // Góc quay trái tránh tường (độ)
#define WALL_TURN_RIGHT_DEG 90     // Góc quay phải tránh tường (độ)
#define WALL_TURN_BACK_DEG 180     // Góc quay đầu khi cả trái và phải đều có tường (độ)
#define WALL_HOLD_MS 400u          // Thời gian giữ hiển thị trạng thái tường trên OLED (ms)

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

// --- Encoder quang chữ U 20 lỗ (PB7 = EXTI7 bánh trái, PB6 = EXTI6 bánh phải) ---
#define ENC_LEFT_PIN Encoder_L_Pin   // PB7 - ngắt ngoài xung encoder bánh trái
#define ENC_RIGHT_PIN Encoder_R_Pin  // PB6 - ngắt ngoài xung encoder bánh phải
#define ENC_SLOTS_PER_REV 20u        // Số lỗ vật lý trên 1 vòng đĩa encoder
#define ENC_EDGES_PER_SLOT 2u        // Đếm cả 2 sườn xung (RISING + FALLING) để nhân đôi độ phân giải
#define ENC_PULSES_PER_REV (ENC_SLOTS_PER_REV * ENC_EDGES_PER_SLOT) // 40 sườn xung / vòng bánh
#define ENC_DEBOUNCE_US 2000u        // Lọc nhiễu rung sườn xung quang học (2 ms, phù hợp 40 sườn/vòng)
#define ENC_STOP_TIMEOUT_MS 250u     // Quá thời gian này không có xung thì coi như bánh dừng (ms)
#define ENC_SAMPLE_MS 100u           // Chu kỳ cập nhật tốc độ lên OLED và UART khi xe chạy (ms)
#define WHEEL_DIAMETER_CM 7.0f       // Đường kính ngoài bánh xe đo thực tế (cm)
#define WHEEL_CIRCUMFERENCE_CM (3.14159265f * WHEEL_DIAMETER_CM) // Chu vi bánh xe (~0.55 cm / sườn xung)
#define ENC_SPIN_360_PULSES 83u      // Số sườn xung mỗi bánh để xoay 360° (track 14.5 cm / 7.0 cm * 40)

#define PID_TARGET_DRIVE_RPM 65u     // Tốc độ chạy thẳng chậm, ổn định, hạn chế trượt bánh (vòng/phút)
#define PID_TARGET_SPIN_RPM 42u      // Tốc độ xoay chậm, đều để bánh không trượt (vòng/phút)
#define PID_DECEL_DRIVE_RPM 40u      // Tốc độ giảm tốc êm ở những xung cuối khi chạy thẳng (vòng/phút)
#define PID_DECEL_SPIN_RPM 28u       // Tốc độ giảm tốc chậm ở vài độ cuối trước khi phanh (vòng/phút)
#define PID_DECEL_DRIVE_PULSES 12u   // Số sườn xung cuối bắt đầu giảm tốc khi chạy thẳng (~6.6 cm)
#define PID_DECEL_SPIN_PULSES 8u     // Số sườn xung cuối bắt đầu giảm tốc khi xoay (~35° cuối)
#define DRIVE_KICK_DUTY 390u         // Xung mồi nhẹ khi bắt đầu chạy thẳng để không giật lệch mũi (0 - 999)
#define DRIVE_KICK_MS 25u            // Thời gian xung mồi chạy thẳng (ms)
#define PID_REF_RPM 105u             // Tốc độ tham chiếu ứng với PID_BASE_LEFT_DUTY / PID_BASE_RIGHT_DUTY
#define PID_LOOP_PERIOD_MS 20u       // Chu kỳ tính toán vòng lặp PID (ms)
#define PID_MOVE_TIMEOUT_MS 5000u    // Timeout tối thiểu cho mỗi động tác PID theo xung (ms)
#define PID_MIN_DUTY 140             // Ngưỡng duty thấp nhất khi chạy thẳng PID (khớp ngưỡng 147/169 đã đo)
#define PID_SPIN_MIN_DUTY 240        // Ngưỡng duty thấp nhất khi xoay PID để không tụt quá ngưỡng quay bánh
#define PID_BASE_LEFT_DUTY 420       // Duty nền bánh trái (~40% tốc độ ở PID_TARGET_DRIVE_RPM)
#define PID_BASE_RIGHT_DUTY 485      // Duty nền bánh phải (bù tỉ lệ 169/147 do bánh phải nặng hơn)
#define PID_KP 2.0f                  // Hệ số P giữ tốc độ RPM
#define PID_KI 0.4f                  // Hệ số I bù tải giữ tốc độ RPM (tỉ lệ theo chu kỳ 30 ms)
#define PID_KD 0.2f                  // Hệ số D giảm vọt lố tốc độ
#define PID_SYNC_KP 10.0f            // Hệ số P đồng bộ số sườn xung 2 bánh (đã chia 2 vì 40 sườn/vòng)
#define PID_SYNC_KD 1.5f             // Hệ số D đồng bộ vận tốc 2 bánh (duty / rpm lệch)
#define PID_INTEGRAL_MAX 900.0f      // Giới hạn chống bão hòa tích phân (anti-windup), cho phép bù đủ tới trần duty
#define PID_SPIN_SYNC_MAX 40.0f      // Giới hạn bù đồng bộ xung khi xoay để bánh kẹt không kéo chết bánh kia

// --- Điều hướng Turn - Drive - Turn theo tọa độ (x, y, z°) ---
#define NAV_DIST_TOLERANCE_CM 1.0f   // Khoảng cách nhỏ hơn ngưỡng này coi như đã tại điểm (cm)
#define NAV_ANGLE_TOLERANCE_DEG 3.0f // Sai lệch góc nhỏ hơn ngưỡng này thì bỏ qua bước xoay (độ)
#define NAV_STEP_PAUSE_MS 400u       // Nghỉ ngắn sau khi phanh để xe đứng yên hẳn (ms)
#define NAV_STEP_PREVIEW_MS 3000u    // Thời gian hiển thị trước hoạt động tiếp theo lên OLED (ms)

// --- IMU MPU6050 (I2C1 remap: SCL = PB8, SDA = PB9) ---
#define IMU_I2C_HANDLE hi2c1         // Handle I2C1 dùng cho IMU trên PB8/PB9
#define IMU_I2C_ADDR (0x68u << 1)    // Địa chỉ 7 bit 0x68 (AD0 nối GND hoặc thả nổi; 0x69 nếu AD0 nối 3.3V)
#define IMU_I2C_TIMEOUT_MS 10u       // Timeout I2C đọc/ghi thanh ghi MPU6050 (ms)
#define IMU_YAW_SIGN 1.0f            // 1.0f: chip ngửa mặt lên, quay trái CCW dương; đổi -1.0f nếu ngược chiều
#define IMU_GYRO_SENSITIVITY 65.5f   // Độ nhạy dải +-500 °/s (FS_SEL = 1)
#define IMU_CALIB_SAMPLES 200u       // Số mẫu lấy trung bình bù điểm 0 khi đứng yên
#define IMU_CALIB_DELAY_MS 5u        // Chu kỳ lấy mẫu lúc calib (200 * 5 ms = ~1 giây)
#define IMU_GYRO_DEADBAND_DPS 0.35f  // Ngưỡng lọc nhiễu trôi khi xe đứng yên (°/s)
#define IMU_LOOP_PERIOD_MS 5u        // Chu kỳ đọc & tích phân góc IMU khi xe chạy/xoay (ms)
#define IMU_SPIN_KICK_DUTY 460u      // Xung mồi đủ khỏe khi xoay để thắng ma sát tĩnh 2 bánh trên mọi bề mặt (0 - 999)
#define IMU_SPIN_KICK_MS 40u         // Thời gian xung mồi khi bắt đầu xoay (ms)
#define IMU_SPIN_DECEL_DEG 20.0f     // Góc còn lại <= ngưỡng này thì giảm tốc dần về PID_DECEL_SPIN_RPM (°)
#define IMU_SPIN_LEAD_DEG 1.5f       // Ngắt sớm trước đích để bù quán tính phanh ở tốc độ chậm (°)
#define IMU_SPIN_TOL_DEG 0.8f        // Ngưỡng sai số nhỏ nhất ổn định với độ rơ bánh răng TT 1:48 (°)
#define IMU_SPIN_MAX_TRIMS 4u        // Số lượt nhích vi chỉnh tối đa để kéo góc về trong +-0.8°
#define IMU_SPIN_TRIM_DUTY 400u      // Duty khởi điểm nhích vi chỉnh nhẹ đủ thắng ma sát tĩnh (0 - 999)
#define IMU_SPIN_TRIM_STEP_DUTY 45u  // Tăng dần duty thêm mỗi lượt vi chỉnh nếu bánh chưa thắng ma sát tĩnh
#define IMU_SPIN_TRIM_LEAD_DEG 0.25f // Ngắt sớm ở pha nhích vi chỉnh chậm (°)
#define IMU_SPIN_TRIM_PERIOD_MS 2u   // Chu kỳ đọc IMU khi đang vi chỉnh góc (ms)
#define IMU_SPIN_TRIM_TIMEOUT_MS 350u // Timeout mỗi lượt nhích vi chỉnh (ms)
#define IMU_SPIN_SETTLE_MS 120u      // Chờ xe dừng hẳn sau phanh để chốt góc IMU thực tế (ms)
#define IMU_SPIN_OPPOSITE_LIMIT_DEG -15.0f // Ngưỡng phát hiện ngược dấu IMU_YAW_SIGN để dừng an toàn (°)
#define IMU_HEADING_KP 14.0f         // Hệ số P giữ thẳng hướng bằng IMU khi chạy tiến/lùi (duty / độ)
#define IMU_HEADING_KI 0.35f         // Hệ số I bù lệch cơ khí 2 bánh khi chạy thẳng bằng IMU
#define IMU_HEADING_KD 0.4f          // Hệ số D giảm dao động lắc ngang khi giữ thẳng (duty / (°/s))
#define IMU_HEADING_MAX_ADJ 140.0f   // Giới hạn bù lái tối đa giữa 2 bánh khi giữ thẳng bằng IMU

// --- Bộ ghi log nội bộ (Blackbox) lưu vào 8 KB cuối Flash STM32F103C8T6 ---
#define BLACKBOX_FLASH_ADDR 0x0800E000UL                                  // 8 page cuối (8 KB) của 64 KB Flash
#define BLACKBOX_FLASH_PAGES 8u                                           // 8 page x 1 KB (FLASH_PAGE_SIZE = 0x400)
#define BLACKBOX_FLASH_SIZE (BLACKBOX_FLASH_PAGES * 1024u)                // 8192 byte
#define BLACKBOX_HEADER_SIZE 8u                                           // Header 8 byte: magic (4B) + length (4B)
#define BLACKBOX_MAX_DATA_LEN (BLACKBOX_FLASH_SIZE - BLACKBOX_HEADER_SIZE) // 8184 byte dữ liệu ASCII
#define BLACKBOX_MAGIC 0x4C4F4731UL                                       // Chữ ký nhận diện log hợp lệ ("LOG1")
#define BLACKBOX_BOOT_WAIT_MS 3000u                                       // Chờ lệnh UART (DD/CC/HH) sau khi dump log (ms)
#define BLACKBOX_MAX_FLUSHES 8u                                           // Giới hạn tối đa số lần ghi Flash mỗi lần bật nguồn

#endif // PROJECT_CONFIG_H
