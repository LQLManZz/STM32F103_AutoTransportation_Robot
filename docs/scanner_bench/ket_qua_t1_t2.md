# Kết quả bench scanner: T1 (nền) và T2 (cự ly)

Ngày đo: 2026-11-25. Xe đứng yên, servo 90°, đo thô bằng `measureDualDistanceCM()` (không lọc), 20 mẫu/mốc
(T1: 30 mẫu tại 90° + quét 45..135° bước 2°, 3 mẫu/góc). Dữ liệu thô: `tools/scanbench/data/*.csv`
(script `tools/scanbench/scan_bench.py`, cổng COM5).

Hình học: SRF05 dưới cao 15 mm, SRF05 trên cao 95 mm, lùi 70 mm. Vật gắp: rubik 30x30x30 mm.
Vật cản: tấm carton đứng vuông góc trục mũi xe.

## T1 — nền trống ≥ 1.2 m

- 168 mẫu: không có mã lỗi `-1`/`-2`, SRF05 dưới không dội sàn (0/168 có sóng).
- SRF05 trên chỉ bắt 1 vật thật ở ~110 cm phía trái (129°..135°), ngoài tầm quét 100 cm.

## T2 — rubik (T2a) và carton (T2b) theo cự ly

"Chênh" = `top − 7 − bot` (cm). Code hiện tại báo vật cản khi chênh ≤ `SCAN_OBSTACLE_TOLERANCE_CM` (4 cm).

| Mốc (cm) | Rubik dưới | Rubik trên | Rubik chênh | Tường dưới | Tường trên | Tường chênh | Khoảng cách |
|---|---|---|---|---|---|---|---|
| 5 | 3.4 | không thấy | — | 5.0 | 11.8 (±0.2) | -0.2 | — |
| 10 | 9.7 | không thấy | — | 11.3 | 16.6 (±0.2) | -1.7 | — |
| 15 | 13.8 | 26.2 (±0.2) | +5.4 | 14.8 | 21.7 (±0.2) | -0.1 | 5.5 |
| 20 | 18.4 | 28.9 (±0.0) | +3.5 | 20.1 | 26.3 (±0.2) | -0.8 | 4.2 |
| 25 | 23.4 | 33.2 (±0.0) | +2.8 | 24.9 | 31.5 (±0.2) | -0.3 | 3.1 |
| 30 | 28.5 | 38.2 (±0.3) | +2.7 | 29.4 | 36.1 (±0.2) | -0.2 | 3.0 |
| 40 | 37.7 | 47.0 (±0.1) | +2.3 | 39.0 | 46.1 (±0.3) | +0.1 | 2.2 |
| 50 | 47.3 | 55.8 (±0.1) | +1.4 | 48.8 | 56.5 (±0.5) | +0.7 | 0.7 |
| 60 | 57.0 | 65.7 (±0.2) | +1.7 | 59.5 | 67.1 (±0.2) | +0.7 | 1.0 |
| 70 | 66.8 | 75.8 (±0.3) | +1.9 | 69.0 | 76.5 (±0.3) | +0.5 | 1.4 |
| 80 | 76.7 | 85.3 (±0.3) | +1.7 | 78.7 | 86.3 (±0.4) | +0.6 | 1.1 |

## Kết luận

1. **Nguyên nhân lỗi "vật bị báo vật cản"**: SRF05 trên thấy rubik 30 mm từ 15 cm trở ra (chùm loe xuống
   ~17–20°). Với ngưỡng 4 cm, rubik bị báo vật cản ở mọi mốc ≥ 20 cm.
2. **Độ lùi 7 cm đúng** (tường cho chênh ≈ 0), cả 2 cảm biến rất ổn định (độ lệch ≤ 0.5 cm, 0 lần mất sóng).
3. **≤ 30 cm**: rubik ≥ +2.7, tường ≤ −0.1 → tách được bằng ngưỡng chênh ≈ **+1.2 cm**.
4. **≥ 40–50 cm**: khoảng cách chỉ 0.7–1.4 cm, ngang mức nhảy của SRF05 trên → **không tách được bằng chênh
   cự ly**; cần đặc trưng khác (bề rộng góc của cụm — tường rộng, rubik hẹp) hoặc tiến lại gần phân loại lại.
5. SRF05 dưới đọc thấp hơn thước ~3% + ~1 cm (cả 2 cảm biến cùng lệch, không ảnh hưởng phân loại).

Bước tiếp: T3 (quét góc rubik 20/40/60 cm) và T4 (tường nghiêng) để đo bề rộng góc của từng loại.

## T3 — quét góc 45°..135° (bước 2°, 3 mẫu/góc)

Lần quét rubik 20 cm đầu tiên (`t3_rubik_d20`) phát hiện SRF05 trên xoay lệch ~12° sang phải so với SRF05 dưới;
người dùng đã chỉnh lại, các lượt sau đo trên ngàm đã chỉnh (lệch còn ~5°).

| Bố trí | Cung SRF05 dưới | Cung SRF05 trên | Cự ly dưới (giữa → mép) | Chênh tại tâm | Chênh trong cụm |
|---|---|---|---|---|---|
| Rubik 20 cm | 71..121 (50°) | 81..121 | 19.5 → 23.1 | +3.5 | +2.7 .. +5.6 |
| Rubik 40 cm | 61..129 (68°) | 61..123 | 39.0 → 45.5 | +1.8 | −0.6 (mép 61°) .. +3.1 |
| Rubik 60 cm | 55..129 (74°) | 63..123 | 58.9 → 64.9 | +1.9 | +1.1 .. +3.3 |
| Carton 40 cm | 49..135+ (≥ 86°) | 61..127 | 38.9 → 47.0 | +0.7 | −0.2 .. +6.8 (mép 61°..69°) |

Nhận xét:
- Chùm siêu âm rất rộng: rubik 3 cm hiện ra trên 50–74°, bề rộng góc chủ yếu do chùm sóng, không do vật.
- Cả rubik lẫn tường đều cho cự ly gần như không đổi theo góc (vùng cự ly hằng – tiếng vọng theo pháp tuyến),
  nên phép thử "đoạn thẳng 2D" trong `scanner.c` không phản ánh đúng hình dạng tường thật.
- Ở mép cụm, chênh lệch 2 cảm biến không tin được: rubik có thể ra âm (giống tường), tường có thể ra +3..+7
  (giống vật). Chỉ nên phân loại bằng các điểm quanh góc có cự ly nhỏ nhất.
- Tại tâm: tường +0.1..+0.7, rubik +1.8..+3.5 ở 20–60 cm → tách tin cậy được khi ≤ 30 cm, xa hơn chỉ còn ~1 cm.

## Bộ phân loại mới (2026-11-25)

`scanner.c` viết lại (1146 → ~520 dòng, Flash 40 → 31 KB): gom cụm theo SRF05 dưới, phân loại bằng trung vị
`top − 7 − bot` ở lõi cụm (±10° quanh giữa vùng cự ly nhỏ nhất), ngưỡng `SCAN_OBSTACLE_TOLERANCE_CM = 1.8 cm`.
Cụm ≤ `SCAN_CLASSIFY_MAX_CM = 30 cm` được kết luận; cụm xa hơn chỉ báo nghi ngờ (`hasSuspect`) để `agv.c`
xoay về, tiến lại ~20 cm rồi quét lại (`SUSP_INSPECT`). Bỏ các heuristic giả định "tường là đoạn thẳng 2D".

Kiểm chứng:
- Phát lại CSV qua đúng `scanner.c` trên PC (`tools/scanbench/replay.c`): 22/22 mốc tĩnh T2 đúng; quét T1 → NONE;
  rubik 20 cm → OBJ 97°; rubik 40/60 cm, carton 40 cm → nghi ngờ.
- Lệnh `C` trên xe:

| Bố trí | Kết quả | Chênh tại lõi |
|---|---|---|
| Carton 40 cm | NONE + nghi ngờ 97° / 39.1 cm | +0.8 |
| Carton 20 cm | WALL 20.0 cm (2 lần) | +0.9 / +1.1 |
| Rubik 20 cm | OBJ 97° / 19.8 cm | +3.9 |
| Rubik sát trước carton, 20 cm | OBJ 97° / 19.9 cm | +4.5 |

## Vật sát tường lệch góc

- Xe nhìn thẳng vào tường gạch (~23 cm), rubik sát tường lệch trái ~125–135°: kết quả WALL 22.9 cm. SRF05 dưới
  nhận tiếng vọng pháp tuyến của tường tới ±45° nên che mất rubik (đo 23.7 cm, mô hình mặt phẳng 24.0 cm);
  dấu hiệu "giống vật" ở mép cụm cũng xuất hiện ở tường không có vật → không tách được bằng 1 lượt quét.
- T6A — xe song song tường (tường bên phải, cách ~15 cm), rubik sát tường phía trước-phải: tường **không dội về**
  ở cả 2 cảm biến (tia tới nghiêng > 45°), rubik thành cụm riêng 45..101°, cự ly nhỏ nhất 29.4 cm, chênh +2.4..+2.6
  → lệnh `C` cho **OBJ 67° / 29.7 cm**.
- Kết luận: vật sát tường chỉ phát hiện được khi xe nhìn dọc theo tường (sau khi xoay tránh tường), không phải
  khi nhìn thẳng vào tường.
- T6B — xe nghiêng 45° so với tường (tường phía trước-phải), rubik sát tường: quét tách được 2 cụm — tường
  45..83° ở ~16 cm (chênh +0.8..+1.2 → WALL) và rubik 85..133° ở ~29.6 cm (chênh +2.6..+3.7 → OBJ).
  Lệnh `C`: **OBJ 107° / 29.6 cm**. Bước nhảy cự ly 16 → 30 cm (> 7 cm) giữa 2 cụm giúp tách đúng.

## T4 — tường nghiêng (carton, tâm thẳng mũi xe)

| Cự ly | Nghiêng | Kết quả `C` (bản cuối) | Ghi chú |
|---|---|---|---|
| 25 cm | 20° | WALL 23.8 cm | bản đầu báo OBJ: lõi 3 cm lan sang vùng SRF05 trên mất sóng → sửa lõi 1 cm + tứ phân vị dưới |
| 25 cm | 40° | WALL 25.7 cm | lõi lệch ra 55° → sửa: cụm tường phủ lên 60..120° là tường chắn mũi; chênh +1.7 (sát ngưỡng 1.8) |
| 25 cm | 60° | WALL 27.4 cm | bản đầu báo OBJ: chỉ SRF05 dưới bắt mép carton → luật mới: chỉ SRF05 dưới thấy mà > 12 cm là mép vật cản |
| 45 cm | 20° | NONE + nghi ngờ 65° / 44.2 | chênh +0.3 |
| 45 cm | 40° | NONE + nghi ngờ 57° / 43.5 | chênh ~0 |
| 45 cm | 60° | NONE + nghi ngờ 50° / 43.7 | chênh +0.2 |

Thay đổi thuật toán sau T4: `SCAN_CORE_DEPTH_MARGIN_CM` 3 → 1 cm, phân loại bằng tứ phân vị dưới của chênh lệch
(SRF05 trên chỉ đo dài ra khi tiếng vọng yếu), `SCAN_TOP_BLIND_MAX_CM = 12 cm`, tường chắn mũi xét theo bề rộng cụm.

## T5 — kịch bản (lệnh `C` × 3 mỗi cảnh, bản firmware cuối)

| # | Cảnh | Kỳ vọng | Kết quả | Đạt |
|---|---|---|---|---|
| 1 | Trống ≥ 1.2 m | NONE | NONE ×3 | 3/3 |
| 2 | Carton vuông góc 25 cm | WALL | WALL 23.9–24.0 ×3 | 3/3 |
| 3 | Carton áp sát ~9 cm | WALL | WALL 9.0–9.2 ×3 | 3/3 |
| 4 | Góc chữ L, đỉnh ~25 cm | WALL | WALL 15.8–15.9 ×3 (bản đầu: OBJ/WALL/NONE → thêm bằng chứng tường cùng độ sâu) | 3/3 |
| 5 | Rubik chính diện 20 cm | OBJ | OBJ 100–102° / 18.8–19.3 ×3 | 3/3 |
| 6 | Rubik lệch phải (servo 60°) 20 cm | OBJ | OBJ 65–67° / 19.5 ×3 (bản đầu: WALL do lõi rơi vào vùng SRF05 trên mất sóng → thêm bằng chứng vật cùng độ sâu) | 3/3 |
| 7 | Rubik chính diện 50 cm | NONE + nghi ngờ | NONE + nghi ngờ 95° / 48.7 ×3 | 3/3 |
| 8 | Rubik 15 cm trái + vật thấp < 30 mm 25 cm phải | OBJ vật gần | OBJ 112° / 15.3 ×3 (vật < 30 mm bị xếp WALL: SRF05 trên không thấy) | 3/3 |
| 9 | Rubik 20 cm chính diện, carton sát sau | OBJ | OBJ 87–92° / 19.8–20.2 ×3 | 3/3 |

Giới hạn đã biết:
- Vật thấp hơn ~30 mm ở xa hơn 12 cm bị coi là mép vật cản (SRF05 trên không thấy đỉnh vật).
- Vật sát tường lệch khỏi pháp tuyến tường khi xe nhìn thẳng vào tường bị tường che (chùm SRF05 dưới ±45°).
- Góc báo ra có lúc lệch +5..+12° (cảnh 5, 6), lúc không (cảnh 9) — cần bài hiệu chuẩn góc riêng.

## Stress test (cảnh ngẫu nhiên do người dùng dựng, Claude đoán mù từ UART + vẽ lại cảnh)

Công cụ: `tools/scanbench/stress.sh <n>` (chạy lệnh `C`, lưu log) và `draw_scene.py` (vẽ cảnh 2D: điểm SRF05 dưới/trên,
tường từ các điểm gần nhất cục bộ và từ bề mặt cao chỉ SRF05 trên thấy, rubik, vùng nghi ngờ).

| Cảnh | Bối cảnh thật | Lần đầu (mù) | Sửa thuật toán | Kết quả sau sửa |
|---|---|---|---|---|
| S1 | Rubik sát tường phải, lấn ngoài mép cung (~40°) | WALL — sai | bằng chứng tường chỉ từ vùng gần nhất khác; vật có điểm gần nhất ở mép cung → nghi ngờ | Nghi ngờ 45° (xe xoay lại soi) |
| S2 | Rubik trước góc tường lõm | NONE — sai | xác minh tâm chỉ kiểm cự ly, không phân loại lại; xác minh hỏng → nghi ngờ | OBJ 72° / 21.1 |
| S3 | Rubik trong tường chữ U | OBJ 105° / 22.5 — đúng | — | — |
| S4 | Rubik dựa sát tấm fomex xiên | WALL — sai (phía an toàn) | lượt soát dải ≥ 3 điểm chênh kiểu vật ổn định | Lần chạy lại vẫn WALL: mặt rubik chỉ gần hơn fomex ~1 cm, dấu hiệu chập chờn |

Đúng ngay lần đầu: 1/4. Các lỗi còn lại đều là "không nhận ra vật" (an toàn), chưa có lần nào báo tường/mép thành vật.
Giới hạn bổ sung: vật dựa sát tường xiên (mặt vật gần hơn tường < ~2 cm theo hướng nhìn) thường bị coi là tường; xe
phải đổi góc nhìn (nhìn dọc tường) mới tách được.
