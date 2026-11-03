# Panduan Casing Simple HC-SR04 (KatroKit ESP32)

Berkas desain laser cut: [`docs/hcsr04_casing_cutting.svg`](file:///Volumes/Data%20Shared/Project/KatroKit_esp32/docs/hcsr04_casing_cutting.svg)

---

## 1. Spesifikasi Geometri Sensor & Box

* **Sensor HC-SR04:**
  - Modul PCB: 45 × 21 mm.
  - Lubang Transducer: Ø 16.0 mm ($r = 8.0\text{ mm}$).
  - Jarak Sisi Luar Lingkaran: 42.0 mm (Pitch Antar-Pusat = 26.0 mm).
* **Tinggi Dinding (Clearance Bawah):** Ditinggikan menjadi **20.0 mm** (rongga bawah sangat lega untuk tonjolan IC, kristal, solderan kaki komponen, serta kepala soket jumper kabel).
* **Tebal Bahan Akrilik:** 3.0 mm.
* **Sistem Pemasangan:** Butt joint sederhana, direkatkan dengan **lem tembak (hot glue)** ke papan kayu (tanpa baut/sekrup).

---

## 2. Dimensi 6 Panel Box

| No | Panel | Dimensi (mm) | Keterangan |
|---|---|---|---|
| 1 | **Tutup Atas** | 52 × 28 | 2 lubang Ø 16 mm (jarak luar 42 mm) |
| 2 | **Alas Bawah** | 52 × 28 | Polos solid (sisi bawah untuk lem tembak ke kayu) |
| 3 | **Dinding Depan** | 52 × 20 | Polos solid (tinggi 20 mm) |
| 4 | **Dinding Belakang** | 52 × 20 | U-Slot kabel 16 × 10 mm di tengah |
| 5 | **Dinding Samping Kiri** | 22 × 20 | Polos solid (tinggi 20 mm) |
| 6 | **Dinding Samping Kanan** | 22 × 20 | Polos solid (tinggi 20 mm) |

*Dimensi rongga dalam terpasang: **46 × 22 × 20 mm** (tinggi luar total dengan alas + tutup = 26 mm).*

---

## 3. Cara Merakit

1. Rakit 4 dinding (depan, belakang, samping L/R) ke atas **Alas Bawah** menggunakan Lem G / Lem Akrilik.
2. Colokkan kabel jumper 4-pin ke sensor HC-SR04 dan keluarkan melalui U-slot di dinding belakang.
3. Masukkan modul sensor ke dalam box (mata menghadap ke atas).
4. Tutup dengan **Tutup Atas** (moncong sensor akan pas keluar melalui 2 lubang 16 mm). Beri sedikit lem tembak di pojok tutup agar mudah dibuka jika perlu servis.
5. Beri lem tembak pada alas bawah, lalu tempelkan langsung ke lantai kayu di bawah beban katrol.
