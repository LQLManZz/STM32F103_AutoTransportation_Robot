# AGV – rút gọn chi tiết in 3D: thiết kế

Ngày: 2026-10-18 · Trạng thái: đã duyệt trong chat · Bổ sung cho `2026-10-17-agv-srf05-design.md` (các mục không nhắc ở đây giữ nguyên)

## 1. Mục tiêu

| # | Yêu cầu (người dùng) | Cách đáp ứng / kiểm chứng |
|---|---|---|
| G1 | Tinh gọn ngàm servo, gộp chi tiết để tránh sai số lắp | Ngàm cũ + cần hạ → 1 khối `Ngam_Lien`; bỏ mối ghép 2 vít M2 + 2 boss |
| G2 | Thu nhỏ chỗ trống đi jumper | Rãnh U 20 mm + 6 khe dây rút → ống kín lòng 8 × 3.6 mm + rãnh trên cầu + đường hầm qua đế |
| G3 | Giảm số chi tiết in | 7 → 3: `Khung` (liền 2 gá motor + khe servo), `Ngam_Lien`, `Dem_Caster` (giữ rời để chỉnh chiều cao caster) |

File ngàm cũ `model/ngam_servo-srf05.FCStd` và `model/scripts/` giữ nguyên làm tham khảo; AGV không còn phụ thuộc vào chúng
(chỉ dùng `lib_hinh` để nạp STEP SG90/SRF05 và `build_ngam.horn_bien_dang/_hoc_horn` cho hốc horn).

## 2. Servo lọt khe khung

- Mũi khung có khe X `L_mui` ± 6.5, Y −16.7 … 6.4 (thân SG90 + 0.3). Thân thả từ trên; đáy grommet tai tì mặt khung (Z 50).
- 2 lỗ mồi Ø1.7 xuyên khung tại (L_mui, −19.05) và (L_mui, 8.75) cho vít tai servo.
- Servo tụt 10.3 mm so với bản trước: `gs_de` = −10.3 → `z_horn` = 71.45; đáy thân Z 39.7 (dưới khung, trong bán kính 22.4
  quanh trục). Cáp servo ra dưới mặt khung.

## 3. Khung liền gá motor

- Vách + bích của 2 gá motor (hình học như bản trước) hợp vào `Khung`; bỏ 4 lỗ M3 bắt bích. Lỗ M3 xuyên motor và lỗ Ø8 cho trục
  giữ nguyên trên vách. In úp mặt trên khung xuống bàn: vách chĩa lên, không support.
- Bỏ 2 lỗ bắt gá servo; giữ gờ breadboard, trụ L298N, khe pin, lỗ Ø12, lỗ caster.

## 4. Ngàm liền khối `Ngam_Lien` (hệ ngàm, servo 90°; `z_pcb` = 4.0 cố định)

| Phần | Hình học |
|---|---|
| Đế ôm horn | X ±17, Y −4.55 … 4.5, Z −1 … 6.4; hốc horn (`hoc_sau` 1.0, `khe_horn` 0.2) từ đáy; 2 lỗ Ø2.2 tại X ±12.45 + hốc đầu vít Ø4.4 từ Z 2 lên; lỗ tâm Ø5.5 xuyên |
| Cầu | X ±8, Y −4.55 … 38.05, Z −1 … 6.4 |
| Rãnh dây | X ±4, Y −4.55 … 35.85, Z 1.2 … 4.8 mở trên cầu, kín dưới mái đế (Y ≤ 4.5) = đường hầm, ra ở mặt sau đế |
| Ống dẫn dây | ngoài X ±7, Y 30.05 … 38.05; lòng X ±4, Y 32.25 … 35.85 (tâm Y 34.05 = tâm header SRF05); Z `pcb_z` + 34.82 (≈ −19) … 6.4, lòng thông lên rãnh |
| Tấm sau | X ±7, Y `ch_sau` … `ch_truoc` (28.5 … 31.5), nối ống với tấm dưới |
| Tấm dưới + 2 boss | như cần hạ cũ: X ±24, Z `pcb_z` − 2 … `pcb_z` + 23, boss Ø5 lỗ mồi Ø1.7 tại lỗ M2 SRF05 |

- `pcb_z` = 4 − (4 + 10.42 + z_horn − h_chum) = −53.87; tâm chùm vẫn cao `h_chum` = 28.
- Đầu Dupont đực (2.54 × 2.8) luồn thẳng qua lòng ống; dây quay về trục rồi ra sau đế, ở bán kính ≤ 7 mm quanh trục.
- In úp mặt Z = 6.4 xuống bàn (đế + thành cầu đồng phẳng), không support.
- Cột từ đáy cầu (Z −1) tới tâm đầu dò dài ≈ 42.5 mm (cần hạ cũ ≈ 72) → tần số riêng tăng.

## 5. Đi dây SRF05

Header → lòng ống (X co về ±3) → rãnh trên cầu → đường hầm → ra sau đế → lên đỉnh trục ở Z ngàm 11.4 (đỉnh cụm quay 6.4 + 5)
→ ngang 27 mm → breadboard. 6 điểm đầu quay cùng servo.

## 6. Kiểm chứng

Như spec gốc §7 (va chạm ≥ 2 mm, chùm, giao nhau, tiếp đất, dây ≤ 200 mm, vừa bàn in) + mới:
- Hộp 2.6 × 2.9 (đầu Dupont) trượt dọc lòng ống từ đáy tới đỉnh không giao `Ngam_Lien`.
- Dây SRF05 phần quay không giao `Ngam_Lien`; ra khỏi đế ở bán kính ≤ 7 mm quanh trục.
- Khe servo: khung ∩ thân SG90 < 1 mm³; tai servo tì mặt khung; 2 lỗ mồi trùng lỗ tai.
- `Ngam_Lien`: horn ∩ ngàm < 1 mm³, 2 lỗ vít horn thông, tần số riêng > 150 Hz.
- Đúng 3 chi tiết trong `HUONG_IN` và trong `export/in3d`.

## 7. Kết quả (2026-10-18)

`kiem_tra_agv.main(1)` và `main(2)` → PASS cả hai:

| Mục | Kết quả |
|---|---|
| Chi tiết in | 3: `Khung` 207 × 130 × 28 (130 g PLA đặc, liền gá motor), `Ngam_Lien` 48 × 43 × 62 (12.4 g; trước: ngàm 3 g + cần hạ 21 g), `Dem_Caster` (13.7 g) |
| Va chạm cụm quay 0…180° | d_min 2.50 mm (Ngam_Lien ↔ mũi khung, 22–38°) |
| Chùm sóng | không chạm xe; thấy cục tẩy 80…1000 mm |
| Giao nhau tĩnh / tiếp đất / gầm | không có / −0.03, −0.03, 0.00 / 15.6 mm |
| Dây | jumper dài nhất 159 mm; bản 1: 9 M-M + 10 F-M, bản 2: 13 M-M + 10 F-M |
| Mô phỏng (r 400, 30°) | dừng cách vật 82 mm (mục tiêu 80) |
| Vít bỏ được | 2 × M2 (ngàm ↔ cần hạ), 4 × M3 (bích gá motor), 2 × M3 (gá servo), ~6 dây rút |
