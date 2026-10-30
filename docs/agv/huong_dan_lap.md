# AGV dò vật SG90 + HY-SRF05 – hướng dẫn lắp và dùng file

Thiết kế: `docs/superpowers/specs/2026-10-17-agv-srf05-design.md` + bản rút gọn
`docs/superpowers/specs/2026-10-18-agv-rut-gon-design.md` (3 chi tiết in). Đấu dây: [dau_day_v1.md](dau_day_v1.md) (1 STM32),
[dau_day_v2.md](dau_day_v2.md) (2 STM32).

## 1. Linh kiện

| Linh kiện | SL | Ghi chú |
|---|---|---|
| STM32F103C8T6 Blue Pill | 1 (bản 1) / 2 (bản 2) | cắm breadboard, nạp qua header SWD (ST-Link) |
| Breadboard 830 lỗ | 1 | dán băng keo 2 mặt vào khung, nằm trong gờ |
| L298N | 1 | **giữ** jumper 5V-EN; **tháo** 2 jumper ENA/ENB |
| Motor TT 1:48 + bánh D65 | 2 + 2 | |
| Caster | 1 | chiều cao bù bằng đệm in 3D (mục 3) |
| Hộp pin 3 × 18650 nối tiếp (3S) | 1 | treo dưới khung bằng 2 dây rút |
| SG90 + HY-SRF05 | 1 + 1 | gắn bằng chi tiết in `Ngam_Lien` (thay ngàm cũ + cần hạ) |
| Jumper Dupont 20 cm | M-M ×9, F-M ×10 (bản 1); M-M ×13, F-M ×10 (bản 2) | dài nhất cần 159 mm |

Vật tư tiêu hao (không có trong `model/realife_available`): vít M3 × 8 (≈ 8: L298N, đệm caster + caster),
M3 × 30 + đai ốc (4, xuyên motor), vít M2 × 8 tự ren (2, bắt SRF05), vít đi kèm SG90 (2 vít tai + 2 vít horn +
vít trục horn), dây rút 2.5 mm (2, hộp pin), băng keo 2 mặt.

## 2. Chi tiết in 3D – `model/agv/export/in3d/`

| File | Ghi chú in (PLA/PETG, đầu 0.4, lớp 0.2, 3–4 vòng tường, 20 % infill) |
|---|---|
| `Khung.stl` | 207 × 130 × 28 mm, liền 2 gá motor và khe servo; STL đã lật mặt trên xuống bàn (gá chĩa lên), không support |
| `Ngam_Lien.stl` | ngàm + cần hạ + ống dẫn dây một khối (48 × 43 × 62 mm, ~12 g); đã lật mặt đỉnh đế xuống bàn, không support |
| `Dem_Caster.stl` | khối đệm rời – in lại riêng sau khi sửa `h_caster` |

File `.step` cùng tên ở hệ tọa độ xe (để mở trong CAD khác).

## 3. Đo linh kiện thật rồi sửa tham số

Mở `model/agv/chi_tiet_in_3d.FCStd`, sửa ô trong Spreadsheet `Params`, Recompute, rồi chạy lại
`build_in3d.export_in3d(doc)` (hoặc Export STL bằng tay):

| Ô | Mặc định | Đo gì |
|---|---|---|
| `h_caster` | 32 | từ sàn tới mặt bắt vít của caster thật |
| `caster_de`, `caster_x` | 32, −105 | cạnh đế caster (giả định đế vuông, 4 lỗ cách mép 4 mm) |
| `tt_lo1_x`, `tt_lo2_x` | −9, −26.5 | vị trí 2 lỗ M3 xuyên thân motor TT, tính từ tâm trục (âm = về phía motor) |
| `h_chum` | 28 | độ cao tâm SRF05; tăng lên 35–40 nếu SRF05 bắt tiếng vang của sàn |
| `L_mui` | 65 | khoảng cách trục bánh → trục servo |
| `gs_de` | −10.3 | độ tụt của servo trong khe (đáy grommet tai → đáy thân); đo lại nếu tai servo thật khác |

Motor TT, breadboard và caster là mô hình tự dựng (`linh_kien_tu_dung.FCStd`, có `Params` riêng cho kích thước
motor). `h_caster` chỉ sửa trong `chi_tiet_in_3d.FCStd`: khi dựng lại assembly (mục 6), mô hình caster tự lấy theo.

## 4. Thứ tự lắp

1. Áp motor TT vào vách gá liền dưới khung (trục lọt lỗ Ø8), xỏ 2 vít M3 × 30 qua vách và thân motor, siết đai ốc.
   Cắm 2 bánh.
2. Bắt đệm caster + caster vào đuôi khung (4 vít).
3. Treo hộp pin dưới khung giữa 2 motor bằng 2 dây rút qua 4 khe. Luồn dây pin và dây motor qua lỗ Ø12 bên
   phải, sau hộp pin.
4. Bắt L298N lên 4 trụ đỡ (M3 × 8), cầu đấu 12V/GND/5V quay về phía breadboard.
5. Dán breadboard vào trong gờ. Cắm Blue Pill theo đúng cột/hàng ghi trong bảng đấu dây. Blue Pill gần mũi xe
   xoay 180°: cổng USB quay về phía servo (cấp nguồn/nạp bằng SWD ở đầu kia).
6. Thả SG90 từ trên xuống khe ở mũi khung (dây cáp ra phía dưới khung), tai tì mặt khung, bắt 2 vít tai vào
   2 lỗ mồi trên khung.
7. Cấp xung 1.5 ms (servo 90°) rồi mới cắm horn 2 tay, tay horn vuông góc hướng tiến; siết vít trục horn.
8. Úp `Ngam_Lien` lên horn (horn nằm gọn trong hốc mặt dưới, ống dẫn dây quay về phía trước), bắt 2 vít horn qua
   2 lỗ Ø2.2. Bắt SRF05 vào 2 boss dưới (2 vít M2 × 8, đầu dò ra ngoài, header hướng lên).
9. Luồn từng jumper F-M: đầu đực đẩy từ dưới vào lòng ống, ra ở rãnh trên cầu, đặt dây vào rãnh, luồn qua đường
   hầm dưới đế, ra ở mặt sau đế sát trục quay. Cắm đầu cái vào header SRF05. Chừa một vòng dây nhỏ trên trục rồi
   mới về breadboard.
10. Cắm đầu cáp SG90 ngay trước breadboard, nối các dây còn lại theo bảng đấu dây.

## 5. Ghi chú điện (cho phần firmware sau này)

- Ổn áp 5V trên L298N chịu ~0.5 A; SG90 lúc kẹt có thể kéo ~0.6 A. Không cho servo quay trong lúc motor
  tăng tốc.
- Pin 3S qua L298N đưa ~9–10 V tới motor TT (định mức 6 V): giới hạn duty PWM ≤ 60 %.
- PA8 chịu được 5 V nên ECHO của SRF05 cắm thẳng, không cần cầu phân áp.
- Sau mỗi bước quay servo, chờ 50–100 ms rồi mới đo (độ rơ bánh răng SG90, ngàm rung > 150 Hz).
- Bản 2: MCU_DV gửi tọa độ vật cho MCU_LAI qua UART1 (PA9 → PA10 chéo), chung GND.

## 6. File FreeCAD

| File | Dùng để |
|---|---|
| `model/agv/agv_v1_1stm32.FCStd` | Assembly bản 1 STM32 |
| `model/agv/agv_v2_2stm32.FCStd` | Assembly bản 2 STM32 |
| `model/agv/mo_phong_agv.FCMacro` | Mở 1 trong 2 assembly rồi chạy macro: quét → ước lượng → quay → tiến, dừng cách vật 80 mm. Đổi vị trí `Cuc_Tay` rồi chạy lại để thử |
| Assembly → Simulation | Xem riêng servo quét 0–180° |
| `model/agv/chi_tiet_in_3d.FCStd` | 3 chi tiết in (`Khung`, `Ngam_Lien`, `Dem_Caster`), tham số trong `Params` |
| `model/agv/linh_kien_tu_dung.FCStd` | Motor TT, breadboard, caster, cục tẩy |
| `model/agv/kiem_tra_agv.py` | `kiem_tra_agv.main(1)` / `main(2)`: va chạm, chùm sóng, giao nhau, dây, in 3D → PASS/FAIL |
| `model/agv/scripts/` | Dựng lại toàn bộ bằng script; `test_agv.run("")` chạy bộ test (ghi vào thư mục tạm) |
| `model/agv/export/anh/` | Ảnh 2 phiên bản và 3 khung hình mô phỏng |

Dựng lại toàn bộ (FreeCAD → Python console):

```python
import sys; sys.path.insert(0, r"<thư mục dự án>/model/agv/scripts")
import build_linh_kien, build_in3d, build_assembly_agv, day_noi
build_linh_kien.build_linh_kien(); d = build_in3d.build_in3d(); build_in3d.export_in3d(d)
build_assembly_agv.build_assembly(1); build_assembly_agv.build_assembly(2)
```
