# Bench tích hợp scanner ↔ motor (agvbench)

Scanner (`docs/scanner_bench/ket_qua_t1_t2.md`) và motor (`docs/motor_bench/huong_dan_test.md`) đã đạt khi test riêng. Bộ bài này đo **chỗ ghép** trong `agv.c`: góc scanner → tọa độ → lệnh xoay/chạy → đo lại ở 90°. Mỗi bài cô lập 1 mắt xích, làm theo thứ tự I0 → I6; bài sau chỉ tin được khi bài trước đạt.

## Hai module giao tiếp thế nào

Scanner và motor không gọi nhau; `agv.c` đứng giữa, `destination.c` bọc motion và cập nhật tọa độ xe.

```
scanForObject ─► (status, angleDeg, distanceCm) + ScanExtraInfo + corridor hint + đoạn tường
      │
      ▼ agv.c
computeObjectCenter: tâm vật = trục servo (cách trục bánh 6.5 cm) + tia (6.0 + d + 1.0) cm theo hướng theta + (servo − 90)
agvTurnCmdToObject:  lệnh xoay = −(hướng tới vật − theta)   (> 0 quay phải)
      │
      ▼ destinationExecSpin / destinationExecDrive  (servoStop trước khi chạy)
motionSpinAnglePid (IMU) / motionDriveDistancePid (PID RPM + giữ hướng IMU)
      │  pose.theta = heading IMU, pose.xy += TB xung 2 bánh × chu vi
      ▼
đo lại 1 tia ở 90° (measureDualDistanceMedianCM + scanClassifyTarget), tắt ngắt encoder khi đo
```

Điểm yếu cần đo: (1) góc scanner lệch +5..+12° ở vài cảnh; (2) sau xoay không có bước nào kiểm lại góc — chùm SRF05 rộng (~24° trên rubik) nên đo ở 90° chỉ xác nhận cự ly; (3) servo tắt xung trong lúc chạy, không cấp lại trước khi đo 90°; (4) đo ngay sau phanh 100 ms; (5) Δcự ly siêu âm và quãng encoder chưa từng so trên cùng 1 lần chạy.

Đã sửa trước khi test: `isWallBump` trước đây không bao giờ được bật → vật sát tường tìm bằng lượt soát cuối bị `agv.c` coi là tường và bỏ. Luật nghi ngờ: ưu tiên mục tiêu chính diện (±15°, chạy thẳng không xoay), không có thì mục tiêu gần nhất; tiếp cận vật không bao giờ xoay góc < `AGV_MIN_SPIN_DEG`.

## Chuẩn bị

1. `ultra_agv_brrbrbr/Core/Src/main.c`, vùng `USER CODE BEGIN 3`: comment `agvRun();`, bỏ comment `agvBenchRun();`. Build và nạp.
2. Pin đầy, nối UART1 (COM5, 115200), `pip install pyserial`, `python tools/agvbench/agv_bench.py ping` → `BENCH_READY`.
3. Sàn: dán băng dính đường giữa thân xe kéo dài ra trước; thước đo góc giấy tâm đặt dưới trục servo (I1); đánh dấu vị trí bánh trước mỗi lần chạy.
4. Mỗi lệnh dưới sàn: sau khi gửi có 3 s đặt xe, ~1 s đứng yên calib IMU. Log lưu `tools/agvbench/data/<label>.log`; chấm lại: `python tools/agvbench/agv_bench.py summary`.

## Các bài

| # | Bài | Lệnh | Dựng cảnh | ĐẠT khi | Trượt thì |
|---|---|---|---|---|---|
| I0 | Xoay góc nhỏ + độ mở ngàm | `motor_bench.py spin --deg 3` / 5 / 8 / 10 / 15 và `--deg=-…` (firmware `motorBenchRun`) | Dưới sàn, đo góc thật | Chọn `AGV_MIN_SPIN_DEG` = góc nhỏ nhất có sai ≤ 1.5° cả 3 lần. Đo khe ngàm mở → `AGV_OBJ_LATERAL_TOL_CM` = (khe − 3)/2 − 0.5 | Ghi bảng góc → sai số |
| I1 | Hiệu chuẩn góc scanner (xe đứng yên) | `scan_bench.py scan` (lệnh `C`, firmware `scanBenchRun`) | Rubik tại 60/75/90/105/120° × 20 và 30 cm, mỗi chỗ 3 lần | \|góc báo − góc thật\| ≤ 3°, \|d − thước\| ≤ 1 cm | Lệch đều 1 phía → `SERVO_CENTER_TRIM_DEG`; lệch khác nhau trái/phải → độ rơ/chùm, cần quét tinh sau xoay |
| I2 | Servo giữ 90° khi tắt xung | `hold --ms 2000` | Kê cao. Rubik 20 cm đặt lệch để mép chùm rơi đúng 90° (d nhạy theo góc); dán kim giấy lên horn | `d_off` và `d_reassert` cách `d_before` ≤ 0.5 cm, kim không lệch | Cấp lại xung + settle trước mọi lần đo 90° sau chạy |
| I3 | Δsiêu âm vs encoder, thời gian ổn định | `drive --cm 30`, `drive --cm=-15` | Carton phẳng vuông góc mũi cách 60 cm (lùi: 25 cm) | \|Δsiêu âm − encoder\| ≤ 1 cm; ổn định ±0.5 cm trong ≤ 200 ms sau phanh | Thêm thời gian chờ sau phanh; lệch cùng chiều 3/3 → `WHEEL_DIAMETER_CM` (script gợi ý) |
| I4 | Xoay theo góc scanner | `turn --bias <lệch I1>` | Rubik 25 cm ở 60°, 75°, 120° | IMU xoay sai ≤ 1° so với lệnh; quét tinh còn lệch ≤ 3° | Sai lệnh → bias góc; sai xoay → motor; ngẫu nhiên → bật quét tinh sau xoay |
| I5 | Tiếp cận trọn (không gắp) | `approach` (script hỏi 2 số đo thước) | 20/30/45 cm × 60/90/120°, mỗi chỗ 2 lần; thêm 2 cảnh rubik có carton sát phía sau | `ok=1`, mặt đầu dò → rubik 2.0–4.5 cm, tâm rubik lệch đường giữa ngàm ≤ 1.5 cm; ≥ 16/18 lần, cảnh sát tường `wall=1` | Đọc "lý do cuối" (ABORT_DRIVE, MID_MISSED, DRV_LOST, DRV_WALL…) để quy về I1–I4 |
| I6 | Quyết định khi nghi ngờ | (a) `scan_bench.py scan` xem `s_ang`; (b) `step --expect-ang A --expect-spin 0/1` | Bảng cảnh I6 dưới | Đúng góc chọn ±10° và đúng có/không xoay | Sai chọn → `noteSuspect`; xoay vụn → `SCAN_SUSPECT_FRONT_HALF_DEG` |

Lệnh `step` (`N`) chạy đúng 1 lượt trạng thái quét của `agvRun`: nếu thấy vật trong 30 cm xe sẽ tiếp cận và gắp như thật — dựng cảnh I6 không có vật gần.

### Cảnh I6

| Cảnh | Dựng | Kỳ vọng |
|---|---|---|
| 1 | Rubik 50 cm ở 90° + carton 40 cm ở 60° | Chọn 90°, chạy thẳng: `--expect-ang 90 --expect-spin 0` |
| 2 | Chỉ rubik 50 cm ở 100° | Không xoay: `--expect-ang 100 --expect-spin 0` |
| 3 | Rubik 45 cm ở 60° + rubik 60 cm ở 120° | Chọn gần nhất, xoay: `--expect-ang 60 --expect-spin 1` |
| 4 | Rubik lấn ra ngoài mép cung (~40°, 25 cm) | Chọn mép cung, xoay: `--expect-ang 45 --expect-spin 1` |

Lặp mỗi bài 3 lần; chỉ chỉnh hằng số khi cả 3 lần cùng lệch một phía. Xong thì trả `main.c` về `agvRun();`.

## Kết quả

| Ngày | Bài | Lệnh / cảnh | Kết quả | Ghi chú |
|---|---|---|---|---|
| 2026-11-25 | I1 | 30 cm, rubik đặt trên trục servo: 60/75/90/105/120° | Cự ly ±0.3 cm khi đặt đúng. Góc: 60→60–62, 75→72–75, 90→100, 105→107–110, 120→nghi ngờ mép 135 (tâm 127) | Chùm cả 2 SRF05 lệch +8..+10° so với góc lệnh; vật sát mép 45° bị cắt cụm nên che lệch. 75°: 1/3 lần WALL (chênh +1.7 sát ngưỡng 1.8). 90° ở 30.2 cm → nghi ngờ (đúng thiết kế) |
| 2026-11-25 | I1 | 90°/20 cm | 97–100° (+7..+10), 19.3 cm, OBJ 3/3 | Lệch không phụ thuộc cự ly |
| 2026-11-25 | I1 | 60°/17 cm (lệch phải 30°) | WALL 3/3 | SRF05 trên không thấy rubik ở 17 cm lệch 30° → luật "chỉ SRF05 dưới thấy mà > 16 cm là mép vật cản" xếp nhầm tường |
| 2026-11-25 | I1 | Sau `SERVO_CENTER_TRIM_DEG` 0 → 9, 90°/20 cm | 87–90° (−3..0), SRF05 trên ±2.5°, 18.9–19.3 cm — ĐẠT | Góc lệnh 135° (vật lý 144°) dây vẫn chùng. Quét lên/xuống cho cùng profile: servo không rơ |
| 2026-11-25 | I4 | `turn`, rubik lệch phải ~30° / 22 cm (pin yếu) | Quét 67° → lệnh +19°, IMU +19.3° (xoay ĐẠT); quét tinh sau xoay ra WALL (KHÔNG ĐẠT) | Pin yếu: 2 SRF05 co tầm (dưới chỉ thấy 15° cung, trên mất đỉnh rubik) → luật > 16 cm xếp tường; lặp lại không xoay vẫn yếu. Chạy lại khi pin đầy |
| 2026-11-25 | I4 | `turn` pin đầy, rubik lệch phải ~30° / 24 cm | Quét 70° → lệnh +17°, IMU +19.3° (lố 2.3°); quét tinh sau xoay ra WALL | Không do pin: mặt rubik nghiêng so với tia nhìn → SRF05 trên mất đỉnh, SRF05 dưới co tầm. Xoay mặt rubik về phía xe → OBJ 90°/22.6, lệch 0° (ĐẠT) |
| 2026-11-25 | Sửa | `agv.c` `isPlaneWallSample`/`isFrontObjSample` | Sau xoay/chạy chỉ coi là tường khi 2 SRF05 cùng thấy 1 mặt phẳng | Trước đây mẫu "chỉ SRF05 dưới thấy, > 16 cm" làm `goToDetectedObject` bỏ vật nghiêng mặt |
| 2026-11-25 | I5 | `approach`, rubik nghiêng mặt lệch phải ~30° / 26 cm | Quét 62° → xoay +24°, FRONT_ALIGN 28.3 cm (top ngoài tầm) qua được nhờ bản sửa; chạy 14 cm → FRONT_MID mất sóng → MID_MISSED (KHÔNG ĐẠT) | Rubik lệch sang bên sau khi chạy: góc quét ở phía phải (sát mép cung sau trim 9: vật lý 54°) chưa đủ chính xác cho xoay thô. Việc tiếp: quét tinh + xoay bù ở điểm dừng giữa, đo I1 lại 60/75° sau trim |

Cách đặt vật cho I1: quét `scan_bench.py raw --ang <góc> --n 1` để servo giữ đúng góc, đặt rubik trên trục SRF05 dưới bằng thước từ đầu 2 mắt tròn, mặt vuông góc, rồi bỏ thước khỏi cung quét. Thước đo góc giấy và tọa độ trước/ngang đều làm sai vị trí vài cm / chục độ.
