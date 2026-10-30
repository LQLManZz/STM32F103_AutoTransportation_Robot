# Ngàm SG90 – HY-SRF05: thiết kế

Ngày: 2026-10-17 · Trạng thái: đã duyệt phương án A, chờ duyệt spec

## 1. Mục tiêu

Ngàm in 3D bắt lên horn 2 tay của servo SG90, giữ cảm biến HY-SRF05 để quét ngang (kiểu radar) 0–180°.

Tiêu chí thành công:

| # | Yêu cầu (người dùng) | Cách đáp ứng / kiểm chứng |
|---|---|---|
| R1 | Servo quay 0–180° | Khớp Revolute giới hạn 0–180°; script quét va chạm từng 2° |
| R2 | Cảm biến thẳng đứng (đã chọn 0°), luôn chiếu cùng hướng ra của ngàm | Mặt đầu dò vuông góc +Y của ngàm; không có chi tiết ngàm nào phía trước đầu dò |
| R3 | Jumper không bị servo làm vướng | Tâm header nằm trên trục quay → dây chỉ xoắn ±90°, không quét; kiểm chứng khoảng cách dây–thân servo |
| R4 | Ít vật liệu nhất, đủ cứng để rung servo ít ảnh hưởng | Khối lượng ≤ 4 g PLA; tần số riêng ước tính ≫ 50 Hz; bắt vít, không có khe hở lắp |
| R5 | Assembly để xem chuyển động | Assembly workbench FreeCAD 1.1.1 + Simulation; macro dự phòng |

Ngoài phạm vi: đế giữ thân servo (người dùng tự cố định), góc nghiêng ≠ 0°, firmware.

## 2. Dữ liệu đầu vào (đo từ STEP)

Nguồn: `model/sg90-micro-servo-9g-tower-pro-1.snapshot.3/SG90 - Micro Servo 9g - Tower Pro.STEP`,
`model/hy-srf05-ultrasonic-1.snapshot.5/HY-SRF05/HY-SRF05.STEP`.

**SG90** (hệ STEP, trục ra theo +Y):
- Tâm trục: x = 1.52, z = 27.78. Mặt trên horn: y = 37.92. Mặt trên thân: y = 33.02.
- Solid 0–11, 16–17: thân + phụ kiện; solid 18: vít trục horn (quay cùng horn); solid 11 là tai bắt vít (rộng 32.1 mm).
- Solid 12: trục ra (Ø4.9, đỉnh y = 36.12) – quay cùng horn.
- Horn: solid 13 (1 tay), **solid 14 (2 tay – dùng)**, solid 15 (chữ thập).
- Horn 2 tay: dài 32.1 mm (đầu tay R2.0 tại ±14.05 mm), hub Ø6.9 cao 3.8 mm, tay dày 1.4 mm,
  lỗ Ø1.2 tại ±4.45 + 2k mm (k = 0…5) tính từ trục.

**HY-SRF05** (hệ STEP, đầu dò hướng +Z, chân hướng +Y):
- PCB 45.3 × 20.84 × 1.6 mm (x ±22.65, y ±10.42, z 0.1…1.7).
- 2 lỗ Ø2.16 (M2): (20.52, 8.32) và (−20.70, −8.12).
- Đầu dò Ø15.9 tại x = ±12.2, y = 0, nhô tới z = 13.7.
- Header 5 chân thẳng ở mặt sau: x −6.4…6.3, y 8.4…16.4, z −3.8…0.1 (khối dày 3.9 mm).

## 3. Hệ tọa độ thiết kế (hệ "ngàm")

- Gốc O: giao trục servo với mặt trên horn.
- +Z: theo trục quay, hướng lên. +Y: hướng chiếu của SRF05. +X: dọc tay horn.
- Đổi hệ SG90 STEP → ngàm: dịch (−1.52, −37.92, −27.78), rồi quay +90° quanh X sao cho
  STEP +Y → +Z và STEP +Z → −Y.
- Đổi hệ SRF05 STEP → ngàm: STEP +Z → +Y, STEP +Y → +Z, STEP +X → −X (giữ hệ thuận tay phải);
  dịch sao cho tâm header (STEP x −0.05, z −1.85) nằm trên trục Z và mép dưới PCB ở Z = `z_pcb`.

Vị trí servo 90° = hướng thẳng phía trước của robot. Khi lắp: đưa servo về 90° rồi mới cắm horn, với
tay horn vuông góc hướng phía trước.

## 4. Hình học ngàm

Tất cả kích thước nằm trong Spreadsheet `Params` (tên ô ghi trong ngoặc).

### 4.1 Đế (ôm horn)
- Hộp X ±17 (`de_dai` = 34), Y −4.55…+4.5, Z −1.0…+2.0 (`de_day` = 3).
- Hốc mặt dưới sâu 1.0 mm (`hoc_sau`): biên dạng tiết diện horn 2 tay tại Z = −0.5, nới đều
  0.2 mm (`khe_horn`). Horn tay dày 1.4 mm nên đế còn cách thân servo 3.9 mm theo Z.
- 2 lỗ Ø2.2 xuyên tại X = ±12.45, Y = 0 (trùng lỗ horn k = 4) cho vít tự ren M2×6 hoặc vít nhỏ
  đi kèm horn.
- Lỗ Ø5.5 tại tâm để luồn tua vít siết vít trục horn (`lo_tam`).

### 4.2 Tấm đứng
- X ±23.2, Y −4.55…−2.55 (dày `tam_day` = 2), Z +2.0…+25.3; mặt sau thẳng hàng mép sau đế.
- 2 cửa sổ giảm vật liệu: X ±(2…15.5), Z 7…19, bo góc R2. Gân giữa rộng 4 mm trên trục.
- Vát 2 × 2 mm ở góc trong giữa đế và tấm đứng (mặt trước), thay cho fillet.

### 4.3 Boss giữ SRF05
- 2 boss Ø5 (`boss_d`), nhô từ mặt trước tấm đứng ra +Y 4.5 mm (`boss_cao`): Y −2.55…+1.95.
  4.5 mm > 3.9 mm (khối header) → header cách tấm 0.6 mm.
- Lỗ mồi Ø1.7 sâu 4.5 mm (`lo_moi`) cho vít M2×8 tự ren bắt từ mặt trước PCB.
- Tọa độ (với `z_pcb` = 4.0): boss trên tại (X −20.57, Z 22.74), boss dưới tại (X +20.65, Z 6.30).
- Đầu vít M2 (Ø3.8) cách vỏ đầu dò ≥ 1.9 mm (khoảng cách tâm lỗ–tâm đầu dò 11.77 mm, R đầu dò 7.95).

### 4.4 Vị trí SRF05 so với ngàm
- `z_pcb` = 4.0 (mép dưới PCB cách mặt trên đế 2.0 mm → đầu vít horn cao ~1.5 mm không chạm PCB).
- Kết quả: mặt sau PCB Y = 1.95, mặt trước Y = 3.55, đỉnh đầu dò Y = 15.55;
  PCB Z 4.0…24.84; đầu chân header Z = 30.82.
- Trọng tâm SRF05 cách trục ≈ 7 mm theo Y.

### 4.5 Vùng cấm cho jumper
- Hộp X ±7.5, Y −1.6…+1.6, Z ≥ 24.84: không có hình học ngàm (tấm đứng ở Y ≤ −2.55 nên thỏa).
- Jumper đi thẳng lên dọc trục. Người dùng chừa vòng dây ≥ 10 cm, buộc dây rút vào khe trên
  đỉnh tấm đứng là tùy chọn (không mô hình hóa).

### 4.6 Ước lượng độ cứng và vật liệu
- Thể tích ≈ 2.4 cm³ → ≈ 3 g PLA (100 % infill); in 4 vòng tường + 20 % infill còn nhẹ hơn.
- Tấm đứng coi như dầm công xôn (dày 2, rộng 34, cao 13 tới trọng tâm), E_PLA ≈ 3.5 GPa, khối lượng
  cảm biến 5 g → k ≈ 1.1×10⁵ N/m, tần số riêng ≈ 740 Hz, cao hơn nhiều so với PWM 50 Hz và rung
  cơ khí của SG90. **Vì vậy bỏ 2 gân tam giác sau** đã nêu trong bản phác thảo: đế không có chỗ đỡ
  gân phía sau, và gân không cải thiện đáng kể. Thay bằng vát 2 × 2 ở mục 4.2.
- Nguồn rung còn lại chủ yếu là độ rơ bánh răng SG90; ngàm không khắc phục được, firmware nên chờ
  ~50–100 ms sau mỗi bước quay trước khi đo (ghi chú, ngoài phạm vi).

### 4.7 In 3D
- Đặt mặt sau Y = −4.55 (mặt sau tấm đứng, phẳng liền với mép sau đế) xuống bàn in: lớp in song
  song mặt tấm đứng nên ứng suất uốn nằm trong lớp; boss hướng lên; không cần support.
  (Mặt bên X = −17 không dùng được vì tấm đứng rộng hơn đế.) File STL xuất sẵn theo hướng này.
- PLA hoặc PETG, đầu phun 0.4, lớp 0.2, 4 vòng tường.

## 5. Mô hình FreeCAD

### 5.1 File
| File | Nội dung |
|---|---|
| `model/ngam_servo-srf05.FCStd` | Ngàm: Spreadsheet `Params` + Part CSG (Box, Cylinder, Cut, MultiFuse) có ràng buộc biểu thức vào `Params`; hốc horn là Part::Feature cố định lấy từ horn STEP. Ghi đè file trống hiện có. |
| `model/assembly_servo_srf05.FCStd` | Assembly: thân SG90, horn 2 tay, App::Link tới ngàm, SRF05, 5 jumper giả lập |
| `model/export/ngam_servo-srf05.stl`, `.step` | File in / trao đổi |
| `model/quay_servo.FCMacro` | Macro quay 0→180→0° (dự phòng nếu Simulation không chạy) |
| `model/kiem_tra_va_cham.py` | Script quét va chạm |

Dùng Part CSG thay PartDesign vì dựng bằng script ổn định hơn, vẫn tham số qua Spreadsheet.

### 5.2 Assembly (Assembly workbench, FreeCAD 1.1.1)
- Chi tiết:
  - `SG90_Than`: solid 0–11, 16–17 của STEP SG90 (bỏ 3 horn, trục ra, vít horn), joint **Grounded**.
  - `Horn2Tay`: solid 14 + solid 12 (trục ra) + solid 18 (vít trục horn, đầu vít nhô 0.3 mm trên horn, nằm trong lỗ tâm Ø5.5) – quay cùng horn.
  - `Ngam`: App::Link tới body ngàm.
  - `SRF05`: STEP SRF05.
  - `Jumper`: 5 khối 2.54 × 2.54 × 14 mm tại X = 0, ±2.54, ±5.08, Y = 0, Z 24.84…38.84,
    mỗi khối có trụ Ø1.4 dài 40 mm hướng lên (dây).
- Khớp:
  - `SG90_Than` ↔ `Horn2Tay`: **Revolute** quanh trục servo, giới hạn góc 0…180°.
  - `Horn2Tay` ↔ `Ngam`, `Ngam` ↔ `SRF05`, `SRF05` ↔ `Jumper`: **Fixed**.
- Chuyển động: Assembly Simulation chạy khớp Revolute 0→180→0°. Dự phòng: `quay_servo.FCMacro` dùng
  QTimer đổi góc khớp và giải lại assembly.

## 6. Kiểm chứng (bắt buộc trước khi báo hoàn thành)

1. **Va chạm**: `kiem_tra_va_cham.py` quét góc 0…180° bước 2°. Ở mỗi góc, khoảng cách nhỏ nhất
   (`distToShape`) giữa nhóm quay {ngàm, SRF05, jumper} và `SG90_Than` ≥ 2.0 mm (dự kiến ≈ 3.9 mm,
   giữa đáy đế và mặt trên thân). Horn + trục ra là chi tiết gốc của servo, không xét.
   In bảng min-distance theo góc.
2. **Jumper**: tại mọi góc, tâm các jumper cách trục ≤ 5.1 mm (chỉ xoắn, không quét), và khoảng cách
   jumper–thân servo ≥ 20 mm.
3. **Hướng chiếu**: không có hình học ngàm trong hình trụ Ø16 trước mỗi đầu dò (Y > 3.55).
4. **Lắp ghép tĩnh**: Ngàm ∩ SRF05 = rỗng; Ngàm ∩ Horn = rỗng (nhờ khe 0.2 mm); vít M2 không chạm
   đầu dò.
5. **Khối lượng**: thể tích ngàm × 1.24 g/cm³ ≤ 4 g.
6. **Ảnh**: chụp Isometric/Front/Top ở góc 0°, 90°, 180° để người dùng xem.

FEM tĩnh là tùy chọn, chỉ chạy nếu người dùng yêu cầu.

## 7. Rủi ro

| Rủi ro | Xử lý |
|---|---|
| Vít horn M2 làm nứt tay horn | Dùng đúng vít nhỏ đi kèm SG90; hoặc khoan lỗ horn Ø1.5 |
| Lỗ mồi Ø1.7 quá chặt/lỏng tùy máy in | Ô `lo_moi` trong Params, đổi 1.6–1.9 |
| Script tạo joint Assembly lỗi API | Tham khảo `Mod/Assembly/AssemblyTests`; dự phòng App::Part + Placement theo biểu thức góc |
| Header của mẫu SRF05 thực tế khác STEP (chân cong 90°) | Kiểm tra bo thật trước khi in; nếu chân cong ra sau, tăng `boss_cao` và đổi vùng cấm |

## 8. Hướng dẫn lắp và sử dụng file

**Lắp:**
1. Cấp xung 1.5 ms (servo 90°) cho SG90, rồi mới cắm horn 2 tay sao cho tay horn vuông góc với hướng
   phía trước của robot. Siết vít trục horn.
2. Úp ngàm lên horn (horn nằm gọn trong hốc mặt dưới), bắt 2 vít nhỏ đi kèm SG90 qua 2 lỗ Ø2.2 tại
   X = ±12.45 vào lỗ tay horn.
3. Đặt SRF05 lên 2 boss (mặt đầu dò quay ra ngoài, header ở trên), bắt 2 vít M2×8 tự ren.
4. Cắm jumper thẳng đứng lên trên, chừa vòng dây ≥ 10 cm trước khi đi về mạch.

**File (thực tế đã tạo):**

| File | Dùng để |
|---|---|
| `model/ngam_servo-srf05.FCStd` | Ngàm tham số – sửa Spreadsheet `Params`, Recompute |
| `model/assembly_servo_srf05.FCStd` | Assembly; Assembly → Simulation để xem quay 0–180° |
| `model/quay_servo.FCMacro` | Macro quay 0→180→0 (2 chu kỳ), dừng ở 90° |
| `model/kiem_tra_va_cham.py` | Quét va chạm từng 2°, in bảng + PASS/FAIL (~30 s) |
| `model/export/ngam_servo-srf05.stl` | In 3D (đã xoay: mặt sau nằm trên bàn) |
| `model/export/ngam_servo-srf05.step` | Trao đổi CAD |
| `model/export/anh/goc_{000,090,180}_{iso,mat_truoc,tren}.png` | Ảnh xem nhanh |
| `model/scripts/` | `lib_hinh.py`, `build_ngam.py`, `build_assembly.py`, `test_ngam.py` (dựng lại toàn bộ bằng script; test chạy trong thư mục tạm, không ghi đè file thật) |

**Kết quả đo:** ngàm 2423 mm³ ≈ 3.0 g PLA; khe tấm đứng–header 0.63 mm; nhóm quay cách thân servo
≥ 3.90 mm và jumper cách thân servo ≥ 29.74 mm ở mọi góc 0–180°; tâm jumper cách trục ≤ 5.08 mm.

**Ghi chú triển khai:** cửa sổ giảm vật liệu dựng bằng Part primitive (không dùng Draft) vì Draft gây
crash FreeCAD 1.1 khi document assembly vừa đóng; mô phỏng dùng Motion `Angular` với công thức
`pi/2 + pi/2*sin(pi/2*time)` (rad, góc khớp tuyệt đối), 0–4 s.

**Sửa sau review cuối:** 2 vết khoét Ø4.4 (`dau_vit_d`) qua phần vát để đầu vít horn nằm phẳng; hốc horn
theo `hoc_sau`/`khe_horn` (Offset2D + Extrusion); `tam_z_top = z_pcb + 21.3`, assembly và kiểm tra va chạm
đọc `z_pcb` từ ngàm; STEP xuất qua NURBS.
