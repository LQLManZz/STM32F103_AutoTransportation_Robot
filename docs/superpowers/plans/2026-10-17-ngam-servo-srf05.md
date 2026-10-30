# Ngàm SG90 – HY-SRF05 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Dựng ngàm in 3D (tham số) giữ HY-SRF05 trên horn 2 tay SG90, assembly có khớp xoay 0–180° để xem chuyển động, và script chứng minh không va chạm / jumper không vướng.

**Architecture:** Mọi hình học sinh bằng Python chạy trong FreeCAD GUI qua MCP `execute_code`. `lib_hinh.py` nạp 2 file STEP và đưa về "hệ ngàm" (spec §3); `build_ngam.py` dựng ngàm Part CSG ràng buộc vào Spreadsheet `Params`; `build_assembly.py` dựng Assembly workbench; `kiem_tra_va_cham.py` quét góc bằng hình học thuần (không phụ thuộc solver). Test là các hàm `test_*` trong `test_ngam.py`, chạy trong FreeCAD.

**Tech Stack:** FreeCAD 1.1.1 (Part, Draft, Spreadsheet, Assembly workbench), Python của FreeCAD, freecad-mcp (`execute_code`, `execute_code_async`, `get_view`). Không có `freecadcmd` (headless không dùng được).

**Spec:** `docs/superpowers/specs/2026-10-17-ngam-servo-srf05-design.md`

## Global Constraints

- Thư mục gốc: `M = C:/Users/long5/Documents/TKVM/__ProjectHOC__/STM32_scanner_srf05/model`; script ở `M/scripts/`.
- Hệ tọa độ "ngàm" (spec §3): gốc = trục servo ∩ mặt trên horn; +Z trục quay lên; +Y hướng chiếu; +X dọc tay horn.
- SG90 STEP → ngàm: dịch (−1.52, −37.92, −27.78) rồi `Rotation(Vector(1,0,0), 90)` (STEP +Y→+Z, +Z→−Y).
- SRF05 STEP → ngàm: `X = −x − 0.05`, `Y = z + 1.85`, `Z = y + 10.42 + z_pcb` (Matrix hàng: [−1,0,0,−0.05], [0,0,1,1.85], [0,1,0,10.42+z_pcb]).
- Góc servo `a` ∈ [0, 180]; nhóm quay = `Rotation(Vector(0,0,1), a − 90)` quanh trục Z; hướng chiếu = (cos a, sin a, 0). 90° = thẳng phía trước.
- Thể tích ngàm ≤ 3226 mm³ (= 4 g PLA ở 1.24 g/cm³).
- Dung sai so sánh kích thước: 0.05 mm; "không giao nhau" = `common().Volume < 0.01`.
- Lệnh chạy test (gửi qua `mcp__freecad__execute_code`, `include_screenshot=false`):
  ```python
  import sys, importlib; P=r"C:/Users/long5/Documents/TKVM/__ProjectHOC__/STM32_scanner_srf05/model/scripts"
  sys.path.insert(0, P) if P not in sys.path else None
  import test_ngam; importlib.reload(test_ngam); test_ngam.run("<pattern>")
  ```
  `test_ngam.py` import `FreeCAD, Part, Mesh, math` và tự thêm cả `M` lẫn `M/scripts` vào `sys.path`.
  `run(pattern)` reload các module `lib_hinh`, `build_ngam`, `build_assembly`, `kiem_tra_va_cham`, chạy mọi `test_*` có tên chứa `pattern`, in `PASS name` / `FAIL name: <lỗi>` và dòng cuối `N passed, M failed`.
- Thư mục dự án không phải git repo: bước "Commit" thay bằng "Lưu" (save FCStd / file script). Không tự `git init`.
- Không sửa 2 thư mục STEP gốc.

## Review Focus

1. Người dùng đổi `boss_cao` nhỏ hơn mặc định → tấm đứng đè lên khối header 3.9 mm; mong đợi: test báo khe header < 0.3 mm. → `test_khe_header` (Task 2).
2. Đổi `lo_moi` trong dải 1.6–1.9 hoặc `tam_day` → recompute vẫn ra solid hợp lệ, thể tích đổi đúng chiều. → `test_doi_tham_so` (Task 2).
3. Chạy lại `build_ngam` lần hai → không nhân đôi object, file ghi đè sạch. → `test_build_lap_lai` (Task 2).
4. Mở lại `assembly_servo_srf05.FCStd` từ đĩa (FreeCAD mới mở) → Link tới ngàm vẫn phân giải (đường dẫn tương đối). → `test_asm_mo_lai` (Task 3).
5. Kéo khớp trong GUI vượt 0…180° → giới hạn góc của khớp Revolute chặn lại. → `test_asm_gioi_han_goc` (Task 3).

---

### Task 1: Nạp STEP và đổi hệ tọa độ (`lib_hinh.py`)

**Files:**
- Create: `model/scripts/lib_hinh.py`
- Create: `model/scripts/test_ngam.py` (runner `run()` + test của task này)

**Interfaces:**
- Produces (trong `lib_hinh`):
  - `MODEL_DIR: str`, `SG90_STEP: str`, `SRF05_STEP: str`
  - `load_sg90() -> dict` với khóa `"than"` (solid 0–11, 16–18), `"horn"` (solid 14 + solid 12), cả hai là `Part.Shape` ở hệ ngàm. Cache mức module.
  - `load_srf05(z_pcb: float = 4.0) -> Part.Shape` ở hệ ngàm. Cache STEP thô mức module (file 17 MB).
  - `srf05_header(z_pcb: float = 4.0) -> Part.Shape` — solid 0–4 của STEP SRF05, ở hệ ngàm.
  - `srf05_lo(z_pcb: float = 4.0) -> list[tuple[float, float]]` — [(X, Z)] tâm 2 lỗ M2: `[(-20.57, z_pcb+18.74), (20.65, z_pcb+2.30)]`.
  - `dau_do(z_pcb: float = 4.0) -> list[FreeCAD.Vector]` — tâm mặt trước 2 đầu dò: `(±12.2, 15.55, z_pcb+10.42)`.
  - `make_jumpers(z_pcb: float = 4.0) -> Part.Shape` — 5 hộp 2.54×2.54×14 tâm X ∈ {0, ±2.54, ±5.08}, Y = 0, Z từ `z_pcb+20.84` tới `+14`, mỗi hộp có trụ Ø1.4 dài 40 hướng +Z nối trên đỉnh.
  - `quay(shape: Part.Shape, a: float) -> Part.Shape` — bản sao quay `a − 90` độ quanh trục Z.

- [ ] **Step 1: Viết test fail** trong `test_ngam.py`:
  ```python
  def test_sg90_he_ngam():
      s = lib_hinh.load_sg90(); h, t = s["horn"].BoundBox, s["than"].BoundBox
      assert abs(h.ZMax - 0.0) < 0.05 and abs(t.ZMax - (-4.9)) < 0.1
      assert abs(h.XLength - 32.1) < 0.1 and abs((h.XMin + h.XMax)/2) < 0.05
  def test_srf05_he_ngam():
      b = lib_hinh.load_srf05(4.0).BoundBox
      assert abs(b.ZMin - 4.0) < 0.05 and abs(b.ZMax - 30.82) < 0.05
      assert abs(b.YMax - 15.55) < 0.05 and abs(b.XMax - 22.60) < 0.05 and abs(b.XMin + 22.70) < 0.05
  def test_srf05_lo_va_header():
      lo = lib_hinh.srf05_lo(4.0)
      assert lo == [(-20.57, 22.74), (20.65, 6.30)]                      # giá trị đã làm tròn 2 chữ số
      hb = lib_hinh.srf05_header(4.0).BoundBox
      assert abs((hb.YMin + hb.YMax)/2) < 0.05 and abs((hb.XMin + hb.XMax)/2) < 0.1
      for x, z in lo:                                                     # đầu vít M2 Ø3.8 không chạm vỏ đầu dò R7.95
          assert min(math.hypot(x - p.x, z - p.z) for p in lib_hinh.dau_do(4.0)) >= 7.95 + 1.9
  def test_jumper_tren_truc():
      j = lib_hinh.make_jumpers(4.0)
      assert all(abs(s.CenterOfMass.y) < 0.05 for s in j.Solids)
      assert max(abs(s.CenterOfMass.x) for s in j.Solids) < 5.1
  def test_quay_huong_chieu():
      v = lib_hinh.quay(Part.Vertex(0, 1, 0), 0).Point
      assert (v - FreeCAD.Vector(1, 0, 0)).Length < 1e-6
  ```
- [ ] **Step 2: Chạy `test_ngam.run("")`** — mong đợi FAIL (`lib_hinh` chưa tồn tại).
- [ ] **Step 3: Viết `lib_hinh.py`** theo Interfaces và Global Constraints. `srf05_lo` trả giá trị làm tròn 2 chữ số. Đọc STEP bằng `Part.Shape().read(path)`.
- [ ] **Step 4: Chạy `test_ngam.run("")`** — mong đợi `5 passed, 0 failed`.
- [ ] **Step 5: Lưu** 2 file script.

---

### Task 2: Ngàm tham số, test hình học, xuất file in (`build_ngam.py`)

**Files:**
- Create: `model/scripts/build_ngam.py`
- Modify: `model/scripts/test_ngam.py` (thêm test)
- Overwrite: `model/ngam_servo-srf05.FCStd` (đã xác nhận đang rỗng: 0 object)
- Create: `model/export/ngam_servo-srf05.stl`, `model/export/ngam_servo-srf05.step`

**Interfaces:**
- Consumes: `lib_hinh.load_sg90`, `load_srf05`, `srf05_header`, `srf05_lo`, `dau_do`.
- Produces:
  - `NGAM_PATH = MODEL_DIR + "/ngam_servo-srf05.FCStd"`; tên document `ngam_servo_srf05`.
  - `build_ngam(path: str = NGAM_PATH) -> App.Document` — đóng document cùng tên nếu đang mở, tạo mới, dựng, recompute, `saveAs(path)`.
  - Trong document: `Params` (Spreadsheet::Sheet), object kết quả tên **`Ngam`** (Part::Feature hoặc Part::Cut sau Refine).
  - `ngam_shape(doc) -> Part.Shape` = `doc.getObject("Ngam").Shape`.
  - `horn_pocket_tool(khe: float, hoc_sau: float) -> Part.Shape` — tiết diện horn tại Z = −0.5, chỉ lấy wire ngoài lớn nhất, `makeOffset2D(khe)`, đùn từ Z = −hoc_sau − 0.01 tới Z = 0.
  - `export_ngam(doc, out_dir: str = MODEL_DIR + "/export") -> tuple[str, str]` — STEP ở hệ ngàm; STL đã xoay để mặt Y = −4.55 nằm ở Z = 0 (spec §4.7).

**Ô `Params` (alias = giá trị mặc định, đơn vị mm):**
`de_dai 34`, `de_day 3`, `de_y_truoc 4.5`, `hoc_sau 1.0`, `khe_horn 0.2`, `lo_vit_horn 2.2`, `vit_horn_x 12.45`, `lo_tam 5.5`, `tam_day 2`, `tam_rong 46.4`, `tam_z_top 25.3`, `cs_x0 2`, `cs_x1 15.5`, `cs_z0 7`, `cs_z1 19`, `bo_goc 2`, `vat 2`, `boss_d 5`, `boss_cao 4.5`, `lo_moi 1.7`, `z_pcb 4.0`.
Ô dẫn xuất (công thức): `tam_y_truoc = 1.95 - boss_cao`, `tam_y_sau = tam_y_truoc - tam_day`, `boss_tren_z = z_pcb + 18.74`, `boss_duoi_z = z_pcb + 2.30`. Boss X cố định −20.57 (trên), +20.65 (dưới).

**Cấu trúc CSG (mọi Placement/kích thước gắn `setExpression` vào `Params`):**
- `De`: Part::Box, X ±de_dai/2, Y `tam_y_sau`…`de_y_truoc`, Z −1…−1+de_day.
- `Tam`: Part::Box, X ±tam_rong/2, Y `tam_y_sau`…`tam_y_truoc`, Z (−1+de_day)…`tam_z_top`.
- `Boss_Tren`, `Boss_Duoi`: Part::Cylinder trục +Y, Ø`boss_d`, cao `boss_cao`, chân tại `tam_y_truoc`.
- `Vat`: tam giác vuông cạnh `vat` đùn dọc X ±de_dai/2 ở góc (Y `tam_y_truoc`, Z −1+de_day).
- Cắt: `CuaSo_Trai/Phai` (Draft Rectangle `FilletRadius = bo_goc` + Part::Extrusion xuyên tấm), `Hoc_Horn` (Part::Feature từ `horn_pocket_tool`), `Lo_Vit_Horn` ×2 (Ø`lo_vit_horn` tại X ±`vit_horn_x`, trục Z), `Lo_Tam` (Ø`lo_tam`), `Lo_Moi` ×2 (Ø`lo_moi`, trục Y, sâu hết boss + tấm).

- [ ] **Step 1: Viết test fail:**
  ```python
  def _ngam(): return build_ngam.ngam_shape(build_ngam.build_ngam())
  def test_ngam_hop_le_va_the_tich():
      s = _ngam(); assert s.isValid() and len(s.Solids) == 1 and 1500 < s.Volume <= 3226
  def test_ngam_bao():
      b = _ngam().BoundBox
      assert abs(b.XMin + 23.2) < .05 and abs(b.XMax - 23.2) < .05
      assert abs(b.YMin + 4.55) < .05 and abs(b.YMax - 4.5) < .05   # boss Y max 1.95 < 4.5
      assert abs(b.ZMin + 1.0) < .05 and abs(b.ZMax - 25.3) < .05
  def test_ngam_khong_giao_horn_srf05():
      s = _ngam(); g = lib_hinh.load_sg90()
      assert s.common(g["horn"]).Volume < .01 and s.common(lib_hinh.load_srf05()).Volume < .01
  def test_khe_header():
      assert _ngam().distToShape(lib_hinh.srf05_header())[0] >= 0.3      # mặc định 0.6
  def test_vung_cam_jumper_trong():
      cam = Part.makeBox(15, 3.2, 40, FreeCAD.Vector(-7.5, -1.6, 24.84))
      assert _ngam().common(cam).Volume < .01
  def test_tia_dau_do_khong_bi_che():
      s = _ngam()
      for p in lib_hinh.dau_do():
          tru = Part.makeCylinder(8, 200, p, FreeCAD.Vector(0, 1, 0))
          assert s.common(tru).Volume < .01
  def test_lo_vit_va_lo_moi():
      s = _ngam()
      for x in (-12.45, 12.45): assert not s.isInside(FreeCAD.Vector(x, 0, 0.5), .01, True)
      for x, z in lib_hinh.srf05_lo():
          assert not s.isInside(FreeCAD.Vector(x, 0.0, z), .01, True)       # tâm lỗ mồi rỗng
          assert s.isInside(FreeCAD.Vector(x + 1.7, 0.0, z), .01, True)      # thành boss đặc
  def test_doi_tham_so():
      d = build_ngam.build_ngam(); v0 = build_ngam.ngam_shape(d).Volume
      d.Params.set("tam_day", "2.4"); d.Params.set("lo_moi", "1.9"); d.recompute()
      s = build_ngam.ngam_shape(d); assert s.isValid() and s.Volume > v0
  def test_build_lap_lai():
      n1 = len(build_ngam.build_ngam().Objects); n2 = len(build_ngam.build_ngam().Objects)
      assert n1 == n2
  def test_xuat_file():
      d = build_ngam.build_ngam(); step, stl = build_ngam.export_ngam(d)
      r = Part.Shape(); r.read(step)
      assert abs(r.Volume - build_ngam.ngam_shape(d).Volume) / r.Volume < .01
      m = Mesh.Mesh(stl); assert m.CountFacets > 0 and abs(m.BoundBox.ZMin) < .05
      assert abs(m.BoundBox.ZLength - 9.05) < .1    # Y cũ: -4.55 (mặt sau, nằm trên bàn) … +4.5 (mép trước đế)
  ```
- [ ] **Step 2: Chạy `test_ngam.run("ngam")`, `run("khe")`, …** — mong đợi FAIL (`build_ngam` chưa có).
- [ ] **Step 3: Viết `build_ngam.py`** theo Interfaces, bảng `Params` và cấu trúc CSG. Kết quả cuối `Ngam` = Refine (`removeSplitter`) của Cut. `test_doi_tham_so` chứng minh biểu thức gắn đúng — nếu Volume không đổi là biểu thức chưa gắn.
- [ ] **Step 4: Chạy `test_ngam.run("")`** — mong đợi tất cả PASS (Task 1 + Task 2 = 15).
- [ ] **Step 5: Chụp kiểm tra bằng mắt** `get_view` Isometric + Front + Top trên document `ngam_servo_srf05`; ngàm có đủ đế, hốc horn, 2 cửa sổ, 2 boss, vát.
- [ ] **Step 6: Lưu** — file `ngam_servo-srf05.FCStd` và 2 file export đã tồn tại trên đĩa.

---

### Task 3: Assembly có khớp xoay 0–180° (`build_assembly.py`)

**Files:**
- Create: `model/scripts/build_assembly.py`
- Modify: `model/scripts/test_ngam.py`
- Create: `model/assembly_servo_srf05.FCStd`

**Interfaces:**
- Consumes: `lib_hinh.load_sg90`, `load_srf05`, `make_jumpers`, `quay`; `build_ngam.NGAM_PATH`, object `Ngam`.
- Produces:
  - `ASM_PATH = MODEL_DIR + "/assembly_servo_srf05.FCStd"`; tên document `assembly_servo_srf05`.
  - `build_assembly(path: str = ASM_PATH) -> App.Document` — mở `NGAM_PATH` nếu chưa mở, tạo Assembly `Assembly` chứa `SG90_Than`, `Horn2Tay`, `Ngam` (App::Link, đường dẫn tương đối — lưu ASM cùng thư mục với ngàm), `SRF05`, `Jumper`; khớp: Grounded(`SG90_Than`), Revolute `Khop_Servo`(`SG90_Than`↔`Horn2Tay`, giới hạn 0…180°), Fixed(`Horn2Tay`↔`Ngam`), Fixed(`Ngam`↔`SRF05`), Fixed(`SRF05`↔`Jumper`); có Simulation chạy `Khop_Servo` 0→180→0.
  - `set_goc(doc, a: float) -> None` — đặt góc servo `a` và giải assembly.
  - `vi_tri(doc, ten: str) -> FreeCAD.Placement` — placement toàn cục của part `ten`.

- [ ] **Step 1: Khảo sát API (chỉ đọc).** Đọc `<FreeCAD>/Mod/Assembly/AssemblyTests/TestCore.py`, `JointObject.py`, `CommandCreateSimulation.py` để lấy: cách tạo `Assembly::AssemblyObject`, cách tạo joint bằng script (Grounded, Revolute, Fixed, thuộc tính giới hạn góc), và cách đặt góc của khớp Revolute (Motion `Angular` trong Simulation hoặc offset của joint). Ghi kết quả thành docstring đầu `build_assembly.py`.
  **Quy tắc quyết định:** nếu không đặt được góc Revolute bằng script → dùng phương án dự phòng (spec §7): `App::Part` `CumQuay` chứa Horn/Ngam/SRF05/Jumper, `Placement.Rotation.Angle` gắn biểu thức `Params.goc_servo - 90` (Spreadsheet riêng trong document assembly); `set_goc` khi đó chỉ ghi ô `goc_servo` rồi recompute. Báo người dùng phương án nào được dùng.
- [ ] **Step 2: Viết test fail:**
  ```python
  def test_asm_khop():
      d = build_assembly.build_assembly()
      kinds = sorted(j.JointType for j in d.findObjects("App::FeaturePython") if hasattr(j, "JointType"))
      assert kinds.count("Fixed") == 3 and kinds.count("Revolute") == 1   # + 1 GroundedJoint
  def test_asm_huong_chieu():
      d = build_assembly.build_assembly()
      for a in (0, 45, 90, 180):
          build_assembly.set_goc(d, a)
          v = build_assembly.vi_tri(d, "SRF05").Rotation.multVec(FreeCAD.Vector(0, 1, 0))
          assert (v - FreeCAD.Vector(math.cos(math.radians(a)), math.sin(math.radians(a)), 0)).Length < 1e-3
  def test_asm_gioi_han_goc():
      j = build_assembly.build_assembly().getObject("Khop_Servo")
      assert j.EnableAngleMin and j.EnableAngleMax       # tên thuộc tính đã xác nhận trong JointObject.py
      assert abs(j.AngleMin.Value - 0) < 1e-6 and abs(j.AngleMax.Value - 180) < 1e-6
  def test_asm_mo_lai():
      build_assembly.build_assembly(); App.closeDocument("assembly_servo_srf05"); App.closeDocument("ngam_servo_srf05")
      d = App.openDocument(build_assembly.ASM_PATH); d.recompute()
      assert not d.getObject("Ngam").Shape.isNull()
  ```
  Nếu dùng phương án dự phòng: `test_asm_khop` thay bằng kiểm tra `CumQuay` chứa đúng 4 part; `test_asm_gioi_han_goc` kiểm tra `set_goc` từ chối (`ValueError`) với a < 0 hoặc a > 180.
- [ ] **Step 3: Chạy `test_ngam.run("asm")`** — mong đợi FAIL.
- [ ] **Step 4: Viết `build_assembly.py`.** Part `SG90_Than`, `Horn2Tay`, `SRF05`, `Jumper` là `Part::Feature` mang shape từ `lib_hinh` (đã ở hệ ngàm, nên placement ban đầu = identity, tương ứng góc 90°).
- [ ] **Step 5: Chạy `test_ngam.run("")`** — mong đợi tất cả PASS.
- [ ] **Step 6: Lưu** `assembly_servo_srf05.FCStd`.

---

### Task 4: Kiểm tra va chạm theo góc (`kiem_tra_va_cham.py`)

**Files:**
- Create: `model/kiem_tra_va_cham.py` (spec §5.1 đặt ở `model/`)
- Modify: `model/scripts/test_ngam.py`

**Interfaces:**
- Consumes: `lib_hinh.load_sg90`, `load_srf05`, `make_jumpers`, `quay`; `build_ngam.ngam_shape`, `build_ngam.build_ngam` hoặc document đang mở.
- Produces:
  - `quet(buoc: float = 2.0) -> list[dict]` — mỗi phần tử `{"goc": a, "d_than": float, "d_jumper": float, "r_jumper": float}`; `d_than` = min khoảng cách {ngàm, SRF05, jumper} quay góc a tới `SG90_Than`; `d_jumper` = khoảng cách jumper quay tới `SG90_Than`; `r_jumper` = max khoảng cách tâm jumper tới trục Z. Góc gồm cả 0 và 180.
  - `main() -> bool` — in bảng `goc | d_than | d_jumper | r_jumper`, dòng cuối `PASS`/`FAIL` theo ngưỡng spec §6, trả bool.

- [ ] **Step 1: Viết test fail:**
  ```python
  def test_quet_va_cham():
      kq = kiem_tra_va_cham.quet(2.0)
      assert len(kq) == 91 and kq[0]["goc"] == 0 and kq[-1]["goc"] == 180
      assert min(r["d_than"] for r in kq) >= 2.0
      assert min(r["d_jumper"] for r in kq) >= 20.0
      assert max(r["r_jumper"] for r in kq) <= 5.1
  ```
- [ ] **Step 2: Chạy `test_ngam.run("quet")`** — mong đợi FAIL.
- [ ] **Step 3: Viết `kiem_tra_va_cham.py`.** Hợp nhất trước {ngàm, SRF05, jumper} thành một compound rồi quay compound (không fuse — chỉ cần `distToShape`). Nếu chạy quá 90 s thì gọi qua `execute_code_async` với `quet` chạy nền rồi lấy kết quả từ biến module.
- [ ] **Step 4: Chạy `test_ngam.run("")`** — mong đợi tất cả PASS; ghi lại `min d_than` thực tế (dự kiến ≈ 3.9 mm).
- [ ] **Step 5: Lưu.**

---

### Task 5: Xem chuyển động + bàn giao

**Files:**
- Create: `model/quay_servo.FCMacro`
- Create: `model/export/anh/goc_{000,090,180}_{iso,front,top}.png`

**Interfaces:**
- Consumes: `build_assembly.set_goc`, document `assembly_servo_srf05`.
- Produces: macro chạy được bằng Macro → Macros… → Execute.

- [ ] **Step 1: Viết `quay_servo.FCMacro`**: mở `assembly_servo_srf05.FCStd` nếu chưa mở; `QtCore.QTimer` 30 ms, mỗi nhịp `set_goc` tăng/giảm 2° giữa 0 và 180, 2 chu kỳ rồi dừng ở 90°.
- [ ] **Step 2: Chạy Simulation của Assembly** (nếu Task 3 dùng Assembly) và macro; xác nhận bằng `get_view` tại 3 thời điểm rằng ngàm + SRF05 + jumper quay quanh trục, thân servo đứng yên.
- [ ] **Step 3: Chụp 9 ảnh** (`set_goc` 0/90/180 × Isometric/Front/Top) lưu vào `model/export/anh/`; gửi người dùng bằng SendUserFile.
- [ ] **Step 4: Chạy toàn bộ `test_ngam.run("")` lần cuối + `kiem_tra_va_cham.main()`**; báo cáo số PASS/FAIL, thể tích (g) và `min d_than` thực tế.
- [ ] **Step 5: Hướng dẫn lắp** (ghi ở cuối spec, mục 8): đưa servo về 90° → cắm horn vuông góc hướng trước → siết vít trục → bắt ngàm bằng 2 vít horn → bắt SRF05 bằng 2 vít M2×8 → cắm jumper, chừa vòng dây ≥ 10 cm.
