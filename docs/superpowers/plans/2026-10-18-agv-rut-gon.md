# AGV rút gọn chi tiết in Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans (người dùng chọn thực thi inline). Steps use checkbox (`- [ ]`) syntax.

**Goal:** 7 chi tiết in → 3 (`Khung` liền gá motor + khe servo, `Ngam_Lien` có ống dẫn dây, `Dem_Caster`), AGV không còn phụ thuộc file ngàm cũ, mọi kiểm chứng PASS.

**Architecture:** Sửa tại chỗ các script AGV hiện có (`model/agv/scripts/`). `build_in3d` dựng 3 chi tiết; `lib_agv` bỏ phụ thuộc document ngàm (`Z_PCB = 4.0`); `day_noi` đổi đường dây SRF05 qua ống; `build_assembly_agv`, `kiem_tra_agv` theo danh sách chi tiết mới.

**Tech Stack:** FreeCAD 1.1 GUI qua MCP `execute_code`; `freecadcmd` cho hình học thuần chạy nền.

**Spec:** `docs/superpowers/specs/2026-10-18-agv-rut-gon-design.md` (+ spec gốc `2026-10-17-agv-srf05-design.md`)

## Global Constraints

- Giữ nguyên `model/ngam_servo-srf05.FCStd`, `model/scripts/*` (test_ngam vẫn 29/29).
- `lib_agv.P["gs_de"] = -10.3`; `z_pcb` = 4.0 hằng (`lib_agv.Z_PCB`).
- Ngưỡng như plan gốc: va chạm ≥ 2 mm, giao < 1 mm³, dây ≤ 200 mm, chi tiết ≤ 215 × 215 × 200, tần số riêng > 150 Hz.
- Test: `test_agv.run(pattern)` trên GUI, chia lô < 30 phút; không ghi file thật trong test.
- Không git: "Lưu" + dòng ledger `.superpowers/sdd/2026-10-18-agv-rut-gon/progress.md`.

## Review Focus

1. Người dùng luồn jumper: đầu Dupont đực phải đi thẳng qua lòng ống → `test_ong_luon_duoc_dupont` (R2).
2. Đổi `h_chum` 25…40 → ống, tấm dưới, boss theo; dây vẫn ≤ 200 → `test_ngam_lien_doi_h_chum` (R2), `test_jumper_dai_toi_da` (R3).
3. Vít tai servo vào khung: lỗ mồi trùng tâm lỗ tai, không đâm khe → `test_khe_servo` (R1).
4. Mở assembly sau khi copy thư mục: chỉ còn 2 document nguồn → `test_asm_mo_lai_noi_khac` (R3).
5. Dây SRF05 quay không cọ thành ống/rãnh → `test_day_srf05_trong_ong` (R3).

---

### Task R1: Khung liền gá motor + khe servo

**Files:** Modify `model/agv/scripts/build_in3d.py`, `lib_agv.py` (gs_de), `test_agv.py`.
**Produces:** `IN3D = ("Khung", "Dem_Caster", "Ngam_Lien")` (Ngam_Lien ở R2); `build_in3d.KHE_SERVO = (x0, x1, y0, y1)` tương đối trục servo; `SG90_TAI_Y` giữ nguyên.

- [ ] Sửa test: `IN3D` mới; `test_khung_mui_trong_ban_kinh` (đỉnh khung X > 46.01 trong R26); `test_khe_servo` (khung ∩ than_sg90 < 1; không có vật liệu tại tâm 2 lỗ tai ở Z 48; có vật liệu cách tâm lỗ 1.3 mm; đáy grommet = Z 50 ± 0.05); `test_ga_motor_lo_trung_motor` trên `d.Khung` cả 2 phía; bỏ test gá servo riêng.
- [ ] Chạy → FAIL.
- [ ] Gộp `_ga_motor` thân/lỗ vào `_khung`, bỏ lỗ bích + lỗ gá servo, thêm khe + 2 lỗ mồi; `gs_de` = −10.3.
- [ ] Chạy lại → PASS. Lưu + ledger.

### Task R2: `Ngam_Lien`

**Files:** Modify `build_in3d.py` (thay `_can_ha`), `test_agv.py`.
**Produces:** object `Ngam_Lien` (hệ ngàm); `ngam_lien_shape(doc)`; `lo_srf05_ha(p)` (giữ); `ONG = dict(x=4, y0=32.25, y1=35.85)`; `ong_z0(p)`; `tan_so_rieng_ngam(p) -> float`.

- [ ] Test: `test_ngam_lien_hop_le` (1 solid, vừa bàn in theo `HUONG_IN`); `test_ngam_lien_om_horn` (horn ∩ < 1, khoảng cách < 0.5; không vật liệu tại (±12.45, 0, 3)); `test_ngam_lien_lo_srf05`; `test_ngam_lien_srf05_khe_header` (∩ SRF05 < 1, cách header ≥ 0.3, boss chạm PCB); `test_ong_luon_duoc_dupont`; `test_ngam_lien_doi_h_chum` (25, 40); `test_ngam_lien_tan_so` (> 150).
- [ ] FAIL → dựng theo spec §4 → PASS. Xuất STL/STEP. Lưu + ledger.

### Task R3: Thư viện, dây, assembly, kiểm tra

**Files:** Modify `lib_agv.py`, `day_noi.py`, `build_assembly_agv.py`, `kiem_tra_agv.py`, `test_agv.py`.
**Produces:** `lib_agv.ngam_tren_xe(a, p, in3d_doc=None)` → khóa `than_sg90, horn, ngam, srf05`; `day_noi.SO_DIEM_QUAY = 6`; `build_assembly(ver, path, doc_name, in3d_doc=None, lk_doc=None)`.

- [ ] Test: sửa `test_ngam_tren_xe_chum` (khóa mới), `test_asm_servo_quay` (Horn, Ngam), `test_asm_mo_lai_noi_khac` (2 file nguồn); thêm `test_khong_phu_thuoc_ngam_cu` (dựng assembly khi document ngàm cũ đóng, không mở lại nó), `test_day_srf05_trong_ong` (DayQ ∩ Ngam_Lien < 1; điểm ra sau đế cách trục ≤ 7).
- [ ] FAIL → sửa code → PASS các lô test; dựng lại file thật (linh kiện, in3d, 2 assembly), bảng đấu dây.

### Task R4: Kiểm chứng + tài liệu

- [ ] `kiem_tra_agv.main(1)`, `main(2)` (freecadcmd) → PASS; `test_quet_va_cham` trên GUI; `test_ngam.run("")` 29/29.
- [ ] Mô phỏng + ảnh lại; xóa STL/STEP cũ (Ga_Motor_*, Ga_Servo, Can_Ha) khỏi `export/in3d`; cập nhật `docs/agv/huong_dan_lap.md`, mục kết quả trong spec rút gọn.
