# AGV dò vật bằng SG90 + HY-SRF05: thiết kế

Ngày: 2026-10-17 · Trạng thái: thiết kế đã duyệt trong chat, chờ duyệt spec

## 1. Mục tiêu

AGV 2 bánh chủ động + 1 caster. Module servo–SRF05 quét 0–180° để tìm một vật thấp (cục tẩy trắng),
tính tọa độ vật, quay tại chỗ về phía vật rồi chạy tới dừng gần vật. Thiết kế bằng FreeCAD 1.1.

| # | Yêu cầu (người dùng) | Cách đáp ứng / kiểm chứng |
|---|---|---|
| R1 | Dùng lại module ngàm servo + SRF05 đã thiết kế | `model/ngam_servo-srf05.FCStd` giữ nguyên, App::Link vào assembly; chỉ sửa đường dẫn STEP trong `model/scripts/lib_hinh.py` |
| R2 | Chỉ dùng linh kiện trong `model/realife_available` | BOM §2; vít/đai ốc/dây rút là vật tư tiêu hao (giả định có sẵn) |
| R3 | Linh kiện bố trí, tính toán hợp lý; caster có thể sai kích thước | Spreadsheet `Params`; caster tham số + đệm in 3D (§4.5); kiểm tra 3 điểm tiếp đất đồng phẳng |
| R4 | SRF05 quét được cục tẩy cao không quá gầm xe | Cần hạ đưa tâm chùm xuống `h_chum` = 28 mm; script kiểm tra nón chùm (§7) |
| R5 | Nối bằng jumper, Blue Pill cắm breadboard | Dây 3D + bảng đấu dây theo lỗ breadboard (§5) |
| R6 | 2 phiên bản: 1 STM32; 2 STM32 (lái + định vị) | 2 file assembly dùng chung cơ khí, khác breadboard + dây (§5) |
| R7 | Chi tiết in 3D để file riêng | `model/agv/chi_tiet_in_3d.FCStd` + mỗi chi tiết 1 STL/STEP (§4) |
| R8 | Assembly thấy được hoạt động | Khớp Revolute servo/bánh + macro mô phỏng quét → quay → tiến (§6) |

Đã chốt với người dùng: caster tham số + đệm in; bàn in ≥ 220×220 (khung 1 mảnh); pin 3×18650 nối tiếp 3S;
cục tẩy 40×20×12 mm (tham số); phương án cảm biến = cần hạ in 3D.

Ngoài phạm vi: firmware, PCB, FEM, mô phỏng động lực học.

## 2. Linh kiện (BOM) và dữ liệu đầu vào

| Linh kiện | SL | Nguồn mô hình | Kích thước đo/chuẩn |
|---|---|---|---|
| STM32F103C8T6 Blue Pill | 1 (A) / 2 (B) | `stm32f103c8t6-…/STM32F103C8T6_Blue_Pill v6.step` | 53 × 22.9 mm, header 2×20 bước 2.54, hàng cách 15.24 |
| Breadboard 830 lỗ | 1 | chỉ có `.SLDPRT` → **tự dựng** | 165.1 × 54.6 × 8.5 mm, 63 cột, 2 nửa A–E / F–J, 4 rail |
| L298N | 1 | `l298n-…/L298N Driver.STEP` | 44 × 43.5 × 29.5 mm, 4 lỗ M3 |
| Motor TT 1:48 | 2 | chỉ có `.SLDPRT` → **tự dựng** | thân 70 × 22.5 × 18.8 mm, trục đôi, 2 lỗ M3 xuyên thân (đo lại bằng thước kẹp, ô `tt_*` trong `Params`) |
| Bánh D65×25 | 2 | `motor-tt-2…/Wheel D65x25.STEP` | Ø69 (lốp), rộng 25 |
| Caster | 1 | `furniture-wheels/Rodízio.STEP` cao 101 mm → **không dùng**; tự dựng caster tham số | `h_caster` mặc định 32 mm, đế 32 × 32, 4 lỗ |
| Hộp pin 3×18650 (3S) | 1 | `18650-3-batteries.step` | 75 × 62 × 19 mm |
| SG90 + ngàm + HY-SRF05 | 1 bộ | `model/ngam_servo-srf05.FCStd`, STEP gốc | xem spec ngàm 2026-10-17 |
| Jumper Dupont M-M / F-M / F-F | theo §5 | `dupont-jumper-wire-…/*.step` (lấy đầu cắm) | dài 20 cm (kiểm tra ảnh `Wire Length.png`) |
| Cục tẩy (vật mục tiêu) | 1 | tự dựng | 40 × 20 × 12 mm (`tay_*`) |

SG90 (hệ STEP): đáy thân y = 6.17, mặt trên horn y = 37.92 → horn cao hơn đáy thân 31.75 mm. Bán kính
lớn nhất của thân quanh trục ra ≈ 22.4 mm (phía tai dài), ≈ 11.1 mm (phía ngắn).

## 3. Hệ tọa độ và bố trí

### 3.1 Hệ xe
- Gốc O: trên sàn, ngay dưới trung điểm trục 2 bánh chủ động. +X tiến, +Y trái, +Z lên.
- Xe quay tại chỗ quanh trục Z qua O → tọa độ vật quy về O.
- Tâm bánh: (0, ±`track`/2, 34.5). `track` tính từ bề rộng khung + khe 3 mm + nửa bề rộng bánh.

### 3.2 Bố trí theo chiều cao
| Thành phần | Z (mm) |
|---|---|
| Motor TT (trục đặt ở tâm cao thân) | 23.25 … 45.75 → **gầm xe ≈ 23 mm** (> cục tẩy 12) |
| Mặt dưới khung (`z_khung`) | 46 |
| Khung dày (`khung_day`) | 4 → mặt trên 50 |
| Hộp pin (treo dưới khung) | 27 … 46 |
| Breadboard, L298N | trên mặt khung (Z ≥ 50) |
| Đáy thân SG90 (đặt trên mặt khung qua gá servo) | 50 + `gs_de` |
| Mặt trên horn | 81.75 + `gs_de` |
| Tâm chùm SRF05 (sau cần hạ) | `h_chum` = 28 |

### 3.3 Bố trí trên mặt phẳng (giá trị khởi điểm, script kiểm tra quyết định)
- Trục servo tại X = `L_mui` = 65, Y = 0. Giữ quan hệ horn–thân của assembly ngàm cũ (servo 90° ⇔ tay horn
  ⊥ hướng tiến), nên đuôi dài của thân SG90 (21.45 mm từ trục) nằm về **−Y** xe, phía ngắn (11.1 mm) về +Y.
  Ở góc quét 0° cảm biến chĩa sang −Y, đi qua phía trên đuôi thân → cần hạ phải ở bán kính đủ lớn (§4.4).
- Đổi hệ ngàm → hệ xe: `P_xe = Rz(−90°)·P_ngàm + (L_mui, 0, z_horn)`, `z_horn = 50 + gs_de + 31.75`.
  Hướng chiếu ở góc servo `a`: `(sin a, −cos a, 0)` (0° = phải, 90° = tiến, 180° = trái).
- Mũi khung: phần nhô ra bao gá servo (hình chữ nhật bao thân SG90 + thành 3 mm, bo góc), không vượt bán
  kính 27 mm quanh trục servo; phần còn lại của khung là chữ nhật X −123 … +43, rộng `khung_rong` = 130.
  Tổng dài ≈ 200 mm (≤ 215 để vừa bàn 220).
- Motor TT dọc theo X sát 2 mép khung, trục ra ±Y. Hộp pin giữa 2 motor, càng gần trục bánh càng tốt
  (dồn tải lên bánh chủ động).
- Caster ở đuôi, tâm X ≈ −105, Y = 0.
- Breadboard nằm dọc X, lệch một bên; L298N bên cạnh breadboard, gần đuôi (gần pin); cổng USB và
  header SWD của Blue Pill hướng ra mép khung để cắm nạp.
- Lỗ luồn dây Ø12 trên khung: cạnh L298N (dây motor, dây pin) và gần mũi (dây servo, SRF05).

Ràng buộc để chọn `L_mui`: hình trụ quét của {cần hạ, SRF05, dây SRF05} (bán kính ≈ 55, Z 10…60; ở góc 0°/180° PCB lùi tới X = `L_mui` − 22.65) không
chứa motor, hộp pin, bánh, khung; nón chùm ±15° ở góc quét 0° và 180° (nhìn ngang) không chạm bánh.
Ước tính thủ công: `L_mui` ≥ 62 → chọn 65.

## 4. Chi tiết in 3D

Tất cả trong `model/agv/chi_tiet_in_3d.FCStd`, Part CSG có biểu thức ràng buộc vào Spreadsheet `Params`
(giống cách làm ngàm). Mỗi chi tiết xuất `model/agv/export/in3d/<ten>.stl` (đã xoay đúng hướng in) và `.step`.
PLA/PETG, đầu phun 0.4, lớp 0.2, 3–4 vòng tường.

### 4.1 Khung (`Khung`)
- Tấm 4 mm theo §3.3, gân dưới 3 × 6 mm dọc 2 mép và ngang trước/sau trục bánh.
- Lỗ bắt: gá motor (M3), L298N (4 × M3, đo từ STEP), hộp pin (2 khe dây rút), caster (theo `caster_lo_*`),
  gá servo (2 × M3), viền giữ breadboard (gờ cao 3 mm quanh breadboard, khe 0.3 mm; breadboard còn băng
  keo mặt dưới).
- In: mặt dưới úp xuống bàn, gân hướng lên → in lật (mặt trên xuống bàn) để không cần support.

### 4.2 Gá motor TT ×2 (`Ga_Motor_T`, `Ga_Motor_P`, đối xứng gương)
- Tấm chữ L: cánh ngang bắt khung bằng 2 vít M3, cánh đứng dày 4 mm áp vào mặt thân motor, 2 lỗ M3 trùng
  2 lỗ xuyên thân motor (vít M3 × 30 + đai ốc). Khoét chỗ cho núm định vị nhỏ trên thân motor.

### 4.3 Gá servo SG90 (`Ga_Servo`)
- Tháp đế cao `gs_de` (mặc định 0, chỉ là khung kẹp) ôm thân SG90, đỡ 2 tai bằng 2 cột có lỗ mồi Ø1.7
  cho vít đi kèm servo; đế bắt khung 2 × M3. Toàn bộ nằm trong bán kính ≤ 27 mm quanh trục servo.

### 4.4 Cần hạ SRF05 (`Can_Ha`)
- Đầu trên: tấm bắt vào 2 boss M2 của ngàm cũ đúng vị trí SRF05 cũ (2 lỗ Ø2.2 trùng boss, vít M2 × 8).
- Thân: tấm đứng dày `ch_day` = 3 đi ra +Y (hệ ngàm) tới bán kính `ch_r` = 36 (mặt sau PCB SRF05; mặt sau cần ≈ 28.5 > góc gá servo 27), rồi đi xuống tới
  khi tâm đầu dò cao `h_chum` = 28 so với sàn. Có gân chữ T dọc thân để cứng.
- Đầu dưới: 2 boss Ø5 lỗ mồi Ø1.7 giữ SRF05 theo đúng mẫu lỗ M2 của PCB (như ngàm cũ), header hướng lên.
- **Kênh dẫn dây**: 3 khe dây rút dọc thân, dẫn 4 jumper từ header SRF05 lên đỉnh cần rồi về trục quay
  phía trên ngàm (vùng cũ của header) → dây ra khỏi cụm quay ở trên trục, chỉ xoắn, không quét (giữ
  nguyên tinh thần yêu cầu R3 của ngàm).
- Kiểm tra độ cứng bằng ước lượng dầm công xôn (như ngàm §4.6): tần số riêng ≫ 50 Hz.
- In: nằm nghiêng mặt sau xuống bàn, không support.

### 4.5 Đệm caster (`Dem_Caster`)
- Khối có chiều cao `dem_cao = z_khung − h_caster` (mặc định 46 − 32 = 14), cùng mẫu lỗ đế caster,
  khoét rỗng giảm vật liệu. Người dùng đo caster thật → sửa `h_caster`, `caster_de_*`, `caster_lo_*` → Recompute.

## 5. Điện và 2 phiên bản

### 5.1 Nguồn (chung)
- Hộp pin 3S (≈ 11.1 V, tối đa 12.6 V) → cổng 12V/GND của L298N; giữ jumper 5V-EN → ổn áp trên L298N cấp
  5 V ra cổng 5V.
- 5V L298N → rail (+) breadboard; GND L298N → rail (−). Rail cấp Blue Pill (chân 5V, G), SG90, SRF05.
- Ghi chú rủi ro: ổn áp 78M05 ~0.5 A; SG90 kẹt ~0.6 A → không chạy servo cùng lúc tăng tốc motor (ghi
  vào hướng dẫn, firmware ngoài phạm vi).
- Motor TT ở 3S qua L298N (sụt ~2 V) nhận ~9–10 V > định mức 6 V → ghi chú giới hạn duty PWM ≤ 60 %.

### 5.2 Phiên bản A – 1 STM32 (`agv_v1_1stm32.FCStd`)
| Chức năng | Chân Blue Pill | Nối tới |
|---|---|---|
| SRF05 TRIG | PB15 | SRF05 Trig |
| SRF05 ECHO | PA8 (chịu 5 V) | SRF05 Echo – nối thẳng, không cầu phân áp (không có điện trở) |
| Servo PWM | PA1 (TIM2_CH2) | SG90 dây cam |
| ENA / ENB | PA6 / PA7 (TIM3_CH1/CH2) | L298N ENA / ENB (tháo 2 jumper EN) |
| IN1…IN4 | PB12, PB13, PB14, PB1 | L298N IN1…IN4 |
| 5V / GND | 5V, G | rail breadboard |

SRF05 chân OUT để trống.

### 5.3 Phiên bản B – 2 STM32 (`agv_v2_2stm32.FCStd`)
- Cùng 1 breadboard 830: 2 Blue Pill cắm nối tiếp dọc (mỗi con 20 cột, cách nhau ≥ 3 cột).
- **MCU_DV (định vị)**: SRF05 (PB15, PA8) + SG90 (PA1), như bản A.
- **MCU_LAI (lái)**: L298N (PA6, PA7, PB12, PB13, PB14, PB1), như bản A.
- Liên lạc UART1: MCU_DV PA9 (TX) → MCU_LAI PA10 (RX); MCU_LAI PA9 → MCU_DV PA10. GND chung qua rail.

### 5.4 Jumper trong mô hình
- Mỗi jumper = 2 đầu cắm (đầu Dupont lấy từ STEP M-M/F-M/F-F) + thân dây Ø1.4 quét theo đường gấp khúc bo
  góc R5 (Part Sweep), cắm đúng lỗ breadboard / chân linh kiện. Màu theo net: đỏ 5V, đen GND, vàng PWM
  servo, xanh lá TRIG, xanh dương ECHO, cam ENA/ENB, trắng IN1–4, tím UART.
- Bảng đấu dây sinh tự động từ cùng 1 nguồn dữ liệu (`model/agv/scripts/day_noi.py`) ra
  `docs/agv/dau_day_v1.md`, `docs/agv/dau_day_v2.md`: từ (linh kiện/chân hoặc lỗ breadboard, ví dụ `J15`)
  → tới, loại jumper, chiều dài đường đi.
- Dây đi vào cụm quay chừa vòng dây (mô hình hóa thành 1 vòng cung phía trên trục servo).

## 6. Assembly và mô phỏng

### 6.1 File
| File | Nội dung |
|---|---|
| `model/agv/linh_kien_tu_dung.FCStd` | Motor TT, breadboard 830, caster tham số, cục tẩy (có `Params` riêng) |
| `model/agv/chi_tiet_in_3d.FCStd` | 5 chi tiết in (§4) + `Params` chung bố trí |
| `model/agv/agv_v1_1stm32.FCStd` | Assembly bản A |
| `model/agv/agv_v2_2stm32.FCStd` | Assembly bản B |
| `model/agv/mo_phong_agv.FCMacro` | Macro mô phỏng hoạt động (chạy được trên cả 2 file) |
| `model/agv/kiem_tra_agv.py` | Script kiểm chứng §7, in bảng + PASS/FAIL |
| `model/agv/export/in3d/*.stl, *.step` | File in |
| `model/agv/export/anh/*.png` | Ảnh xem nhanh |
| `model/agv/scripts/` | `lib_agv.py`, `build_linh_kien.py`, `build_in3d.py`, `day_noi.py`, `build_assembly.py`, `test_agv.py` (dựng lại toàn bộ bằng script; test chạy trong thư mục tạm) |
| `docs/agv/dau_day_v1.md`, `dau_day_v2.md`, `huong_dan_lap.md` | Đấu dây, lắp ráp |

### 6.2 Assembly (Assembly workbench FreeCAD 1.1)
- `Khung` Grounded. Gá motor, motor, gá servo, thân SG90, hộp pin, L298N, breadboard, Blue Pill, caster,
  đệm caster: Fixed.
- Revolute: thân SG90 ↔ horn (0…180°, như assembly ngàm); gá motor ↔ bánh trái / phải (không giới hạn).
- Ngàm, cần hạ, SRF05, jumper SRF05: Fixed vào horn.
- Cục tẩy nằm ngoài assembly (đối tượng riêng trong document, Placement do macro đặt).

### 6.3 Macro mô phỏng (`mo_phong_agv.FCMacro`)
Chạy bằng QTimer, thay đổi góc khớp/Placement rồi giải lại assembly; cục tẩy mặc định tại 40 cm, lệch
trái 30° (ô `vat_r`, `vat_goc`).
1. Quét: servo 0 → 180° bước 2°; hiển thị nón chùm ±15° (khối nón bán trong suốt, dài 1 m); nón đổi màu
   đỏ khi giao với cục tẩy.
2. Ước lượng tọa độ: lấy góc giữa của dải góc "trúng" + khoảng cách từ đầu dò tới mặt gần nhất của vật,
   đổi sang hệ xe (cộng vị trí trục servo + bán kính cần hạ). Hiển thị nhãn tọa độ.
3. Quay tại chỗ: thân xe xoay quanh O tới góc vật; 2 bánh quay ngược chiều nhau đúng góc lăn tương ứng.
4. Tiến: xe chạy thẳng, bánh quay theo quãng đường; dừng khi mặt đầu dò cách vật `d_dung` = 8 cm.
5. Kết thúc, đưa servo về 90°. Có nút/biến dừng timer an toàn (sửa lỗi M9 của macro cũ).

Đây là mô phỏng hình học minh họa thuật toán, không phải mô phỏng vật lý.

## 7. Kiểm chứng (bắt buộc trước khi báo hoàn thành)

`kiem_tra_agv.py` chạy trên từng file assembly:
1. **Va chạm cụm quay**: góc 0…180° bước 2°, khoảng cách nhỏ nhất giữa {ngàm, cần hạ, SRF05, dây SRF05}
   và {thân SG90, gá servo, khung, motor, bánh, pin} ≥ 2 mm.
2. **Chùm sóng**:
   - Nón ±15° từ mỗi đầu dò không giao thân xe/bánh ở mọi góc quét (cắt nón từ mặt đầu dò ra 1 m).
   - Cục tẩy 40×20×12 đặt trước mũi, khoảng cách 8 / 10 / 20 / 50 / 100 cm: giao nón ở góc 90° → PASS.
   - In thêm khoảng cách mép dưới nón chạm sàn (thông tin rủi ro tiếng vang sàn, không phải tiêu chí).
3. **Lắp ghép tĩnh**: không cặp linh kiện nào giao nhau (thể tích giao < 1 mm³, trừ chân cắm vào lỗ
   breadboard được miễn); gầm xe ≥ 15 mm; 2 điểm thấp nhất của bánh và điểm thấp nhất caster cùng Z = 0
   (±0.5 mm).
4. **Đấu dây**: mọi dòng trong bảng đấu dây có jumper trong mô hình; mọi jumper dài ≤ 200 mm; không chân
   nào bị 2 tín hiệu khác net.
5. **In 3D**: mỗi chi tiết vừa 215 × 215 × 200 mm theo hướng in; in thể tích + khối lượng PLA ước tính.
6. **Ngàm cũ**: `model/scripts/test_ngam.py` vẫn chạy PASS sau khi sửa đường dẫn STEP.
7. **Ảnh**: isometric / trên / bên cho cả 2 bản, và 3 khung hình mô phỏng (đang quét, đã quay, đã dừng).

## 8. Rủi ro

| Rủi ro | Xử lý |
|---|---|
| Kích thước motor TT / caster thực tế khác giá trị chuẩn | Ô `tt_*`, `h_caster`, `caster_*` trong `Params`; hướng dẫn đo |
| Chùm sóng thấp dính tiếng vang sàn | `h_chum` là tham số; script in khoảng cách nón chạm sàn; thử thực tế 25–40 mm |
| Cần hạ dài gây rung theo servo | Gân chữ T, kiểm tra tần số riêng ước lượng; firmware chờ 50–100 ms sau mỗi bước (ghi chú) |
| Ổn áp 5V L298N quá tải khi servo kẹt | Ghi chú vận hành §5.1 |
| Jumper sang cụm quay bị kéo | Kênh dẫn dây về trục + vòng dây chừa; kiểm tra §7.1 gồm dây |
| Assembly API / Draft gây crash FreeCAD 1.1 (đã gặp ở ngàm) | Không dùng Draft; dùng Part primitive; lưu có thử lại (`lib_hinh.luu`) |
| File SLDPRT không đọc được | Tự dựng motor TT, breadboard từ kích thước chuẩn |
| Khung 210 mm sát bàn in 220 | Kiểm tra §7.5; nếu vượt, xoay chéo khi in hoặc rút `L_mui` sau khi chạy kiểm tra va chạm |

## 9. Kết quả triển khai (2026-10-17)

**Kiểm chứng** (`kiem_tra_agv.main(1)` và `main(2)` → PASS cả hai):

| Mục | Kết quả |
|---|---|
| Va chạm cụm quay 0…180° (bước 2°) | d_min 2.50 mm (cần hạ ↔ mũi khung, 22–38°) ≥ 2 |
| Nón chùm ±15° | không chạm thân xe/bánh ở góc nào; thấy cục tẩy ở 80/100/200/500/1000 mm; mép dưới nón chạm sàn cách mặt dò 75 mm |
| Giao nhau tĩnh | không có (miễn chân Blue Pill trong lỗ breadboard, trục ra SG90 trong thân) |
| Tiếp đất / gầm | bánh −0.03 / −0.03, caster 0.00 mm; gầm 15.6 mm (đáy cần hạ) |
| Dây | dài nhất 167 mm (jumper 20 cm); bản 1: 7 M-M + 10 F-M, bản 2: 11 M-M + 10 F-M (gồm 2 jumper bắc cầu rail) |
| In 3D (PLA đặc) | Khung 207 × 130 × 14 (122 g), Gá motor ×2 (4.2 g), Gá servo (1.9 g), Đệm caster (13.7 g), Cần hạ 48 × 35 × 92 (21 g); tần số riêng cần hạ 168 Hz |
| Mô phỏng (cục tẩy r = 400, 30°) | ước lượng (341, 208) mm, quay 31.4°, tiến 205 mm, dừng cách vật 82 mm (mục tiêu 80) |

**Khác so với mục 3–5** (chi tiết trong ledger `.superpowers/sdd/2026-10-17-agv-srf05/progress.md`):
- Khung X −126…46 (+ mũi tới 81), mũi = hộp ∩ tròn R26 (không phải R27) để cần hạ cách ≥ 2 mm; bỏ gân dọc, thêm 4 trụ đỡ L298N.
- Cần hạ: tấm trên + cầu nối + cột rãnh chữ U + tấm dưới, bán kính `ch_r` = 36; in úp mặt đỉnh.
- Bánh: tâm lốp Y ±83.5 (moay-ơ cách mặt motor 1 mm); trục TT có 2 mặt vát khớp lỗ chữ D của bánh.
- Blue Pill: bản 1 cột 63→44 (xoay 180°, USB về mũi xe); bản 2 MCU_DV cột 63→44, MCU_LAI cột 16–35. Thứ tự
  chân theo pinout stm32-base (STEP không có silkscreen) – kiểm tra bo thật.
- Nguồn SRF05 + SG90 lấy ở rail dưới B+/B−, bắc cầu từ rail trên T+/T− (L298N 5V).
- `h_caster` chỉ sửa ở `chi_tiet_in_3d`; assembly tự đồng bộ sang mô hình caster.
