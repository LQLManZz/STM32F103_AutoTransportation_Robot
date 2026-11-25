# ultra_agv_brrbrbr

Firmware STM32F103C8T6 (HAL, CMake): xe AGV 2 bánh. Servo SG90 quay cụm 2 cảm biến siêu âm (SRF05 dưới + SRF05 trên, lùi 7 cm) để quét gom cụm tìm vật gần nhất và phân biệt tường/vật cản cao; kết hợp MPU6050 IMU (`I2C1`), encoder quang 2 sườn (`40` sườn/vòng), điều hướng tọa độ (`nav.c`), vòng lặp PID giữ tốc độ + giữ hướng thẳng + xoay vi chỉnh bằng IMU. `main.c` gọi `agvRun()` (chạy toàn hệ thống); test riêng scanner bằng `scanBenchRun()`, test riêng 2 bánh bằng `motorBenchRun()` qua lệnh UART. Tay gắp 2 servo (`grabber.c`) và quay về điểm xuất phát theo bản đồ tường 2D (`destination.c`). Thông tin debug hiện trên serial (UART1) và LCD 16x2 HD44780 + module I2C PCF8574 (I2C2).

Project nằm trong `ultra_agv_brrbrbr/`.

## Cấu trúc (`Core/Src`, `Core/Inc`)

- `main.c`: gộp các hàm khởi tạo ngoại vi CubeMX (`MX_GPIO_Init`, `MX_TIM1_Init`, `MX_TIM2_Init`, `MX_TIM3_Init`, `MX_USART1_UART_Init`, `MX_I2C2_Init`, `MX_I2C1_Init`) và các handle toàn cục; gọi `agvInit()` rồi trong `while (1)` gọi `agvRun()` (muốn test riêng scanner thì đổi sang `scanBenchRun()`, test riêng motor thì đổi sang `motorBenchRun()`, test ghép scanner + motor thì đổi sang `agvBenchRun()`; không viết logic ứng dụng riêng vào `main.c` và không tách `gpio.c`/`i2c.c`/`tim.c`/`usart.c` ra file riêng).
- `agv.c`: điều phối FSM tự hành (`agvInit`, `agvRun`), theo dõi tọa độ xe `robotPose` và tọa độ tâm vật `objectCenter` (bù offset trục servo/đầu dò/nửa vật), xoay thô 1 lần về tâm vật (`agvTurnCmdToObject`: không xoay khi lệch < `AGV_MIN_SPIN_DEG`, trên ngưỡng mới xét lệch ngang > `AGV_OBJ_LATERAL_TOL_CM`; chưa dùng `scanFine`), tiếp cận vật bằng PID tối đa `2` bước có 1 điểm dừng đo lại ở 90° (`goToDetectedObject`) và tránh tường (`handleWall`); nghi ngờ trong ±`15°` chính diện thì chạy thẳng tới soi, lệch hơn mới xoay về nó. Hàm cho bench: `agvBenchReset`, `agvApproachObject`, `agvGetPose`.
- `scanner.c`: phân loại mục tiêu 2 cảm biến (`TARGET_NONE`, `TARGET_OBJECT`, `TARGET_OBSTACLE`), quét cung `45°..135°` bước `5°` tầm `100 cm` (`scanArcCluster`): gom cụm theo SRF05 dưới (tách khi nhảy `> 7 cm`), phân loại bằng tứ phân vị dưới của chênh lệch 2 cảm biến ở lõi cụm (điểm trong `1 cm` quanh cự ly nhỏ nhất, ±`10°`); lõi không có SRF05 trên thì xét bằng chứng cùng độ sâu (`2 cm`) và luật `16 cm`; vùng gần nhất khác cách lõi `> 30°` mang dấu hiệu tường thì cả cụm là tường (góc chữ L); cụm `≤ 30 cm` kết luận, cụm xa hơn, vật có điểm gần nhất ở mép cung hoặc xác minh tâm hỏng chỉ báo nghi ngờ (`hasSuspect`, chọn bằng `noteSuspect`: ứng viên chính diện ±`SCAN_SUSPECT_FRONT_HALF_DEG` thắng ứng viên lệch, cùng loại lấy cụm gần nhất) để `agv.c` xoay/tiến lại soi; xác minh tâm chỉ kiểm cự ly SRF05 dưới; lượt soát cuối tìm dải `≥ 3` điểm chênh kiểu vật ổn định (vật dựa sát tường, bật `isWallBump`). `scanSideArc`, quét tinh quanh HOME (`60°..120°`). Kết quả đo: `docs/scanner_bench/ket_qua_t1_t2.md`.
- `scanbench.c`: bài test bench riêng cho scanner (xe đứng yên), nhận lệnh 1 dòng qua UART1: `P` ping, `R ang n` đo thô tại 1 góc, `S start end step n` quét thô, `C` chạy `scanForObject()`; mỗi lệnh kết thúc bằng `BENCH_DONE`. Script PC `tools/scanbench/scan_bench.py` (COM5) gửi lệnh và lưu CSV.
- `motorbench.c`: bài test bench riêng cho 2 bánh (không dùng scanner/servo), nhận lệnh 1 dòng qua UART1: `P` ping; kê cao: `D duty ms` kiểm chiều quay + ánh xạ encoder từng bánh, `E wheel pulses duty` chạy 1 bánh đủ số xung rồi phanh (đo vọt lố, vạch dán), `K dir start end step holdMs` quét duty cả 2 bánh (RPM đếm xung vs RPM ISR), `V dir rpm ms` giữ tốc độ PID (`motionDriveTimePid`, calib IMU trước); dưới sàn: `M cm rpm` chạy thẳng PID, `A deg rpm` xoay PID (chờ đặt xe + calib IMU trước). Mỗi lệnh kết thúc bằng `BENCH_DONE`. Script PC `tools/motorbench/motor_bench.py` (COM5) gửi lệnh, lưu log/CSV, tự chấm ĐẠT/KHÔNG ĐẠT và gợi ý hằng số; quy trình: `docs/motor_bench/huong_dan_test.md`.
- `agvbench.c`: bài test bench ghép scanner + motor (dùng đúng hàm của `agv.c`/`destination.c`), nhận lệnh 1 dòng qua UART1: `P` ping; kê cao: `H ms` đo 90° trước/sau khi motor chạy lúc servo tắt xung/sau khi cấp xung lại; dưới sàn (chờ đặt xe + calib IMU + `agvBenchReset`): `D cm` chạy PID rồi đo 90° liên tục 1 s (Δsiêu âm vs encoder, thời gian ổn định), `T` quét → xoay về vật → `scanFine` đo góc còn lệch, `G` quét → `agvApproachObject` (không gắp), `N` chạy 1 lượt `agvRun` (xem quyết định nghi ngờ). Mỗi lệnh kết thúc bằng `BENCH_DONE`. Script PC `tools/agvbench/agv_bench.py` (COM5), quy trình I0–I6: `docs/agv_bench/huong_dan_test.md`.
- `grabber.c`: tay gắp 2 servo 180° (kẹp `TIM2 CH1` `PA0`, nâng hạ `TIM1 CH4` `PA11`), `grabberPick`/`grabberRelease`, quay mượt và ngắt xung giữ.
- `destination.c`: ghi lịch sử xoay/chạy + mốc tọa độ + đoạn tường 2D, kiểm tra đường thông, lập điểm vòng góc tường và quay về gốc `(0, 0)` theo đường rút gọn (`destinationReturnHome`).
- `servo.c`, `srf05.c`, `delay.c`: driver servo (`TIM2 CH2`, có `moveServoSmooth`), đo tuần tự 2 cảm biến siêu âm bằng `pulseInHigh` (`measureDualDistanceCM`: SRF05 dưới Trigger `PB15` / Echo `PA8`, nghỉ `38 ms` rồi SRF05 trên Trigger `PB14` / Echo `PA12` - tên chân CubeMX `Trig_SRF04`/`Echo_SRF04`), delay us (có `delayInit` bật DWT).
- `imu.c`: driver MPU6050 trên `I2C1` (`PB8`/`PB9`), hiệu chuẩn bias gyro Z, tích phân hình thang tính góc Yaw tương đối và heading chuẩn hóa `(-180°, 180°]`.
- `encoder.c`: đọc 2 encoder quang 20 lỗ qua ngắt ngoài `EXTI9_5` cả 2 sườn `RISING_FALLING` (`40` sườn/vòng, `PB7` trái, `PB6` phải), lọc nhiễu đổi mức `lastPinState` + debounce `2 ms`, tính RPM theo chu kỳ cùng sườn.
- `motor.c`: driver L298N + PWM `TIM3 CH1/CH2`, bù tỉ lệ 2 bánh (`147`/`169`), phanh ngắn mạch 100% duty.
- `motion.c`: điều khiển chuyển động vòng hở (`motionSpinAngle`, `motionDriveDistance`, `motionBrake` có xung đảo chiều) và vòng đóng PID (`motionSpinAnglePid` dùng IMU `5 ms` (vi chỉnh `TRIM-L/R` đang tắt: `IMU_SPIN_MAX_TRIMS = 0`) hoặc fallback encoder; `motionDriveDistancePid`, `motionDriveTimePid` có giảm tốc cuối hành trình + PID RPM + giữ hướng IMU/đồng bộ encoder; `motionForwardHoldStart`/`Step`).
- `nav.c`: điều hướng Turn-Drive-Turn theo tọa độ `NavPose` (`navResetPose`, `navGetPose`, `navGoToPose`, `navFollowWaypoints`).
- `debug.c`: xuất thông tin debug ra serial (UART1) và LCD (dựng 2 dòng từ 5 ô: góc, vị trí vật, quyết định, RPM, trạng thái; bảng `lcdShortCodes` rút gọn nhãn). Các module khác chỉ gọi `debug*` hoặc `debugLog`, không tự gọi `printf` hay LCD trực tiếp. Chứa `debugF1` (format float không cần `-u _printf_float`) và `__io_putchar`.
- `lcd.c`: driver LCD 16x2 HD44780 qua PCF8574 (`I2C2`), tự dò địa chỉ 0x27..0x20 / 0x3F..0x38 và báo `LCD addr=... status=OK|WR_ERR|NO_ACK` ra serial lúc khởi động, chế độ 4 bit (mỗi nibble: đặt dữ liệu → EN cao → EN thấp), gửi cả dòng 1 lần truyền, chỉ gửi lại dòng đổi. Không giải kẹt bus kiểu bit-bang + reset I2C2 (sau đó PCF8574 NACK giao dịch đầu).
- `project_config.h`: mọi hằng số (chân, xung servo, `SCAN_*`, `WALL_*`, `LCD_*`, motor, `ENC_*`, `PID_*`, `NAV_*`, `IMU_*`, `MOTOR_BENCH_*`, `GRABBER_*`, `DEST_*`). Không để số cứng trong `.c`.
- Mỗi module riêng một cặp `.c`/`.h`. File `.c` mới phải thêm vào `target_sources` trong `CMakeLists.txt`.

## Cấu hình STM32CubeMX (không động vào)

- Không sửa `ultra_agv_brrbrbr.ioc`, `cmake/stm32cubemx/`, `Drivers/`, `startup_*.s`, `STM32F103xx_FLASH.ld`.
- Trong `main.c`, `stm32f1xx_it.c`, `stm32f1xx_hal_msp.c` chỉ viết đúng khung CubeMX / cặp `USER CODE BEGIN/END`. Không sửa logic `SystemClock_Config`.
- Chỉ dùng các giao tiếp đã cấu hình trong CubeMX: `I2C1` (`PB8` SCL, `PB9` SDA — MPU6050 IMU), `I2C2` (`PB10` SCL, `PB11` SDA — LCD 16x2), `USART1` (115200), `TIM2 CH2` (`PA1` — servo), `TIM2 CH1` (`PA0` — servo kẹp), `TIM1 CH4` (`PA11` — servo nâng hạ), `TIM3 CH1/CH2` (`PA6`/`PA7` — motor ENA/ENB), `EXTI9_5` `RISING_FALLING` (`PB6` Encoder_R, `PB7` Encoder_L), GPIO (`PA2..PA5` motor IN1..IN4, `PB15` Trigger SRF05, `PB14` Trigger SRF05 trên `Trig_SRF04`, `PA8` Echo SRF05, `PA12` Echo SRF05 trên `Echo_SRF04`). Không tự thêm hay đổi giao tiếp/chân/timer; cần thêm thì báo người dùng để chỉnh trong CubeMX rồi generate lại.
- Lưu ý cấu hình: các hàm `MX_*_Init` và `HAL_*_MspInit` được gộp trong `main.c` và `stm32f1xx_hal_msp.c` (không tách `gpio.c`/`i2c.c`/`tim.c`/`usart.c`). Nếu sau này chạy CubeMX, tránh bật `CoupleFile` để không tách lại file riêng, và lưu ý vùng `FLASH` trong `STM32F103xx_FLASH.ld` nên là `56K` (đã giữ firmware `< 56 KB`; thư viện HAL `STM32_Drivers` được ép `-Os` trong `CMakeLists.txt` để tiết kiệm Flash, code dự án vẫn `-Og`).
- Module lấy handle bằng `extern` (`htim2`, `htim3`, `huart1`, `hi2c1`, `hi2c2`).

## Quy tắc code

- C11, HAL. Hàm và biến camelCase (`measureDualDistanceCM`, `distanceCm`). Hằng số, macro, alias trong `project_config.h` và hằng enum IN_HOA_GẠCH_DƯỚI. Kiểu (struct/enum) PascalCase.
- Hàm module có tiền tố tên module (`motionSpinAngle`, `lcdPrintLine`).
- Khoảng cách cm, góc độ (`uint8_t`), thời gian ms/us; nêu đơn vị trong tên (`d05RawCm`, `angleDeg`, `delayMs`).
- Hàm chỉ dùng trong file thì `static`. Hàm công khai khai báo trong `.h` có include guard.
- Thụt lề 2 space, `{` xuống dòng riêng.
- Viết đơn giản, kiểu nghiệp dư: không thêm tính năng, tham số, tầng trừu tượng, macro bật/tắt hay biến chỉ để debug.

## Quy tắc comment

- Tiếng Việt, ngắn gọn, kiểu `//`, chỉ nêu chức năng (hoặc ý nghĩa giá trị như `-2: quá gần`).

## Debug: serial (115200) và LCD 16x2

- **Nguyên tắc bắt buộc cho LCD:** LCD chỉ dùng để debug và phải giữ nguyên bố cục cố định 2 dòng x 16 ô dưới đây. Chỉ hiển thị **thông số thực nghiệm từ cảm biến** (góc, cự ly, heading, tọa độ, RPM, duty...) và **mã trạng thái ngắn**; **không xuất text thuần hay câu mô tả**. Nhãn dài hơn ô thì cắt bớt, nhãn cắt mất nghĩa thì thêm mã riêng vào `lcdShortCodes` (`FORWARD`→`FWD`, `TURN-90L`→`L90`, `OBJ_FOUND`→`OBJ_FND`, `OBJ_NOTFND`→`NO_OBJ`...).

```
0123456789012345
 90° 12/ 19 SCAN    dòng 0: [góc 4][vị trí vật 7] [quyết định 4]
 65| 64 OBJ_FND     dòng 1: [RPM trái|phải 7] [trạng thái 8]
```

  - Ô góc: góc servo kèm `°` (` 90°`) khi quét/tiếp cận; heading IMU có dấu không kèm `°` (` -45`, `-180`) ở `debugState`/test.
  - Ô vị trí vật: cự ly SRF05 dưới/trên (cm nguyên): số, ` <2` quá gần (-2), `ERR` mất sóng (-1), `---` ngoài tầm. Ở màn hình heading thì để trống, ở điều hướng là tọa độ `x,y`.
  - Ô quyết định (4): `SCAN`, `FWD`, `BACK`, `L90`/`R135` (xoay), `TR-L`/`TR-R`, `BRAK`, `STOP`, `PICK`, `DROP`, `RET`...
  - Ô trạng thái (8): `OBJ_FND`, `OBSTACLE`, `NO_OBJ`, `FINE_OK`, `FINE_SKP`, `DRV_DONE`, `DRV_WFAR`, `IMU_OK`, `DELIVER`, `L:W R:F` (tường trái/phải: W tường, F trống, S bỏ qua), `WP2/4 3s`, `S3/8 2s`...
- **Serial (UART1, 115200):** Các module điều khiển/cảm biến xuất log qua `debug*` hoặc `debugLog` theo dạng `TAG key=value` kết thúc `
` (nhãn đầy đủ, không rút gọn như LCD).

| Sự kiện | Serial | LCD 16x2 |
| --- | --- | --- |
| Quét cụm & quét tinh (`debugScan`, `debugScanFine`) | `SCAN ang=... SRF05_bot=...(...) SRF05_top=...(...) top_adj=... diff=... hit=... status=...` / `FINE ...` | ` 90° 12/ 19 SCAN` / `  0\|  0 OBJ_FND` |
| Căn hướng (`debugAlign`) | `ALIGN coarse_spin=... fine_ang=... fine_spin=... best_d=... status=FINE_OK` (hoặc `FINE_SKIP`) | trạng thái `FINE_OK` / `FINE_SKP` |
| Tiếp cận vật / tường (`debugApproach`, `debugDriveEnd`) | `DRV` / `WALL_DRV` `t=...ms SRF05_bot=... SRF05_top=... target=... L=...rpm R=...rpm`; `DRV_END ...` | ` 90°  8/ 15 FWD` / ` 65\| 64 OBJ_FND` |
| Phát hiện & tránh tường (`debugWallDetected`, `debugWallTurn`) | `WALL_DETECT SRF05_bot=... SRF05_top=... status=OBSTACLE_FND`; `WALL_TURN left=... right=... turn=...` | quyết định `L90`/`R90`/`R180`, trạng thái `L:W R:F` |
| Quyết định / Test động cơ (`debugDecision`, `debugMotorTest`) | `TEST step=FORWARD L=110rpm R=109rpm pL=... pR=... dL=... dR=... H=0.0` | quyết định/bước, trạng thái `MTR_TEST` (khi test) |
| Trạng thái chung (`debugState`) | không xuất UART riêng | ` -45        STOP` / ` 65\| 64 IMU_OK` |
| Điều hướng waypoint & preview test (`debugNavWaypoint`, `debugNavPreview`, `debugStepPreview`) | `NAV ...` / `NAV_NEXT ...` / `TEST_NEXT ...` | heading + `x,y` + phase/action; trạng thái `WP2/4 3s` / `S3/8 2s` |
| Test ngưỡng PWM (`debugMotorPwmTest`) | `TEST_PWM Duty= 450 (PWM=45%) - RAMP DOWN` | ` 45% D450   RAMP` / `.. PWM_TEST` |
| Tay gắp (`debugGrabber`) | `GRAB action=... grip=... lift=... status=...` | `110°  90°   PICK` / `.. GRIP_CLS` |
| Log sự kiện nội bộ (`debugLog`) | `CLUSTER ...`, `OBJ_COORD ...`, `PID_START ...`, `PID_END ...`, `ARRIVED ...` | giữ nguyên |

Thêm sự kiện hoặc cảm biến mới thì thêm/mở rộng hàm `debug*` trong `debug.c` theo đúng bố cục LCD 2 dòng x 16 ô (không text thuần).

## Hình học cụm cảm biến & vật thật (đo thực tế)

- SRF05 dưới: tâm màng cao `15 mm` so với sàn; SRF05 trên: `95 mm`. Cả 2 nằm ngang (khi servo quay có rung lắc nhẹ).
- Mặt SRF05 trên lùi ngang `70 mm` so với mặt SRF05 dưới. Trục servo → mặt SRF05 dưới `60 mm` (`SENSOR_FACE_OFFSET_CM`), mặt SRF05 trên nằm sau trục `10 mm`.
- Vật cần gắp: khối rubik nhựa dán decal màu `30x30x30 mm`. Vật cản: tường cao (carton, bìa fomex, tường gạch). Sàn: nhiều loại.
- Đo bench: chùm siêu âm rất rộng (rubik hiện trên cung `50..74°`); SRF05 trên thấy rubik từ `15 cm` trở ra. Chênh lệch `top - 7 - bot` tại tâm cụm: tường `-1.7..+1.1`, rubik `+2.6..+5.4` khi `≤ 30 cm`; xa hơn 2 loại chỉ cách `~1 cm` nên không phân loại. Ở mép cụm chênh lệch không tin được. 2 cảm biến phải song song (lệch `12°` làm phân loại sai ở mép).
- Giai đoạn test scanner: dời xe bằng tay, không dùng motor/PID.

## Quy ước tọa độ & phân loại vật

- Tọa độ cảm biến/servo: gốc là cảm biến, servo 0° là bên phải (+x), 90° (`SCAN_HOME_ANGLE`, thẳng mũi xe) là +y, 180° là bên trái (-x).
- Tọa độ điều hướng xe (`NavPose`): gốc `(0, 0, 0°)` tại lúc khởi động/calib; `+X` hướng thẳng mũi ban đầu, `+Y` sang trái, `+theta` quay trái (CCW) ngược chiều kim đồng hồ.
- Xe chạy theo tọa độ tâm vật (`computeObjectCenter`, quy đổi từ góc trung tâm cụm và cự ly SRF05 + offset trục servo/đầu dò), căn hướng sơ bộ + quét tinh rồi chạy tiếp cận bằng PID (tối đa 2 bước), không bám theo mục tiêu di động.
- Phân loại mục tiêu trong khoảng `SCAN_MIN_RANGE_CM` (`2 cm`) đến `SCAN_MAX_RANGE_CM` (`100 cm`):
  - Chỉ SRF05 dưới thấy, hoặc SRF05 trên (đã bù lùi `SENSOR_DEPTH_OFFSET_CM = 7 cm`) xa hơn SRF05 dưới quá `SCAN_OBSTACLE_TOLERANCE_CM = 1.8 cm` → `TARGET_OBJECT` (vật thấp cần gắp: SRF05 trên chỉ thấy đỉnh vật).
  - Chênh lệch `≤ 1.8 cm` (cùng 1 mặt phẳng đứng) → `TARGET_OBSTACLE` (tường/vật cản cao). Chỉ tin cậy khi `≤ SCAN_CLASSIFY_MAX_CM = 30 cm`.
  - Chỉ SRF05 dưới thấy mà xa hơn `SCAN_TOP_BLIND_MAX_CM = 16 cm` → mép vật cản (SRF05 trên gắn cố định thấy rubik từ `16.6 cm`, không thấy ở `15 cm`); vật thấp hơn ~`30 mm` vì vậy không được nhận là vật.
  - Giới hạn: tường song song thân xe không dội sóng (tia tới nghiêng > 45°); vật sát tường lệch pháp tuyến hoặc dựa tường xiên thường bị coi là tường, phải nhìn dọc tường mới tách được.
  - Ngoài khoảng hoặc mã lỗi (`-1` timeout, `-2` quá gần) → `TARGET_NONE`.

## Build

```
cmake --preset Debug
cmake --build build/Debug
```

Toolchain `arm-none-eabi-gcc` nằm trong bundle STM32Cube (đường dẫn trong `CMakePresets.json`). Sau khi sửa code phải build lại, không có warning mới được coi là xong.
