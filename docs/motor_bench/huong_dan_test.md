# Bench test 2 bánh xe (motorbench)

Kiểm tra riêng motor + encoder + PID, không chạy scanner/FSM AGV.

## Chuẩn bị

1. `ultra_agv_brrbrbr/Core/Src/main.c`, vùng `USER CODE BEGIN 3`: comment `agvRun();`, bỏ comment `motorBenchRun();`. Build và nạp.
2. Pin đầy (PWM là duty thô, pin yếu thì RPM thấp hơn). Nối UART1 (COM5, 115200).
3. `pip install pyserial`, kiểm tra kết nối: `python tools/motorbench/motor_bench.py ping` → `BENCH_READY`.

Log lưu ở `tools/motorbench/data/<label>.log/.csv`; chấm lại: `python tools/motorbench/motor_bench.py summary`.

## Nhóm 1 – kê cao (bánh không chạm sàn)

| # | Lệnh | Kiểm gì | ĐẠT khi |
|---|---|---|---|
| 1 | `dir --duty 450 --ms 1000` | Mỗi bánh chạy tiến rồi lùi: đúng bánh, đúng chiều, encoder đúng bánh | Bánh chạy ≥ 10 xung, bánh kia ≤ 1 xung; **tự nhìn** chiều quay khớp nhãn (encoder 1 kênh không biết chiều) |
| 2 | `enc --wheel L --pulses 400 --duty 400` (rồi `--wheel R`) | Dán vạch lên bánh. 400 xung = 10 vòng | Vạch lệch khỏi chỗ cũ đúng bằng góc vọt lố script in ra (lệch khác → encoder mất/thừa sườn) |
| 3 | `curve --dir F --start 650 --end 200 --step 25 --hold 2000` (thêm `--dir B`, và quét lên `--start 200 --end 650`) | Đường duty → RPM 2 bánh, ngưỡng chạy, tỉ lệ bù 2 bánh, độ chính xác RPM của ISR | Sai số RPM ISR vs đếm xung TB ≤ 5%. So duty đạt 105 rpm với `PID_BASE_*_DUTY`, tỉ lệ R/L với `MOTOR_RIGHT_SCALE/LEFT_SCALE` |
| 4 | `pid --dir F --rpm 55 --ms 6000` (thêm `--rpm 24`, `--dir B`) | PID giữ tốc độ từng bánh | Sai TB ≤ 5%, độ lệch chuẩn ≤ 10% mục tiêu, tổng xung 2 bánh lệch ≤ 5% |

## Nhóm 2 – dưới sàn

Có 3 s để đặt xe sau khi gửi lệnh, sau đó xe đứng yên ~1 s để calib IMU. Đánh dấu vị trí bánh/mũi trước khi chạy; chạy xong script hỏi số đo thước.

| # | Lệnh | Đo | ĐẠT khi |
|---|---|---|---|
| 5 | `move --cm 50`, `move --cm -50`, `move --cm 100` | Quãng đường thực bằng thước | Sai ≤ 4% lệnh. Script gợi ý `WHEEL_DIAMETER_CM` |
| 6 | `spin --deg 90`, `spin --deg=-90`, `spin --deg 180` | Góc thực (thước đo góc / băng dính trên sàn) | Sai ≤ 3°. Script so IMU vs encoder, gợi ý `ENC_SPIN_360_PULSES` |

Lặp mỗi bài 3 lần trên cùng loại sàn; chỉ chỉnh hằng số trong `project_config.h` khi cả 3 lần cùng lệch một phía.

## Kết quả

| Ngày | Bài | Lệnh | Kết quả | Ghi chú |
|---|---|---|---|---|
| 2026-11-25 | 1 kê cao | `dir --duty 450 --ms 1000` | ĐẠT | L 159/155 xung, R 124/129 xung, không nhiễu chéo |
| 2026-11-25 | 2 kê cao | `enc --pulses 400 --duty 400` | L +10, R +8 xung vọt lố | Chưa kiểm vạch dán |
| 2026-11-25 | 3 kê cao | `curve` F/B quét xuống, F quét lên | ISR sai TB 1.6–1.9% (ĐẠT) | Không tải: ~250 rpm @650; 105 rpm @ L≈212, R≈234–261; R khởi động từ đứng yên cần 275 (L 150); tỉ lệ R/L 1.0–1.22 dao động giữa các lần |
| 2026-11-25 | 4 kê cao | `pid` 55 F/B, 105 F | ĐẠT | sai ≤ 1.4%, sd ≤ 4.4 rpm, lệch xung ≤ 1.3% |
| 2026-11-25 | 4 kê cao | `pid` 32 F, 24 F | KHÔNG ĐẠT | Sàn `PID_MIN_DUTY 140`: bánh L không tải không chậm hơn ~37 rpm, bánh R lúc đứng lúc giật; cần đo dưới sàn (có tải) mới chỉnh |
| 2026-11-25 | 5 sàn | `move --cm 50` (lần 2), `--cm=-50` | ĐẠT | Thước ≈ 50 cm cả 2 chiều, encoder 50.6/50.0 cm, IMU giữ hướng ±2.6°. Lần 1 lệch phải ~7° (log mất, không lặp lại). Giảm tốc 32 rpm có tải chạy tốt → giữ `PID_MIN_DUTY` |
| 2026-11-25 | 6 sàn | `spin` 90R, 180R, 90L | ĐẠT | IMU sai ≤ 0.9°, tổng 3 lần lệch < 3°. Encoder lúc đang xoay 102–106 sườn/360° (cộng xung lúc phanh 104–130) → `ENC_SPIN_360_PULSES` 83 → 105 (chỉ dùng khi không có IMU) |
