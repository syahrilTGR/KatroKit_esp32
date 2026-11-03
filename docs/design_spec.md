# Spesifikasi Desain: PCB Matrix & Box Akrilik Alat Uji Katrol & Beban (Load Cell & HC-SR04)

## 1. Intent & The Why
Mengalihkan rangkaian prototipe breadboard dari alat praktikum/uji katrol beban ke papan **PCB Matrix (Perfboard)** permanen yang rapi, serta merancang **box akrilik 2D skala 1:1** siap potong (*laser cutting*) via Affinity Designer (ekspor DXF).

Sistem fisik terdiri dari:
- **Load Cell** di tiang atas untuk mengukur gaya tarik tali katrol, dihubungkan melalui modul **HX711** di tiang kayu.
- **Sensor Ultrasonik (HC-SR04)** di alas kayu di bawah beban silinder untuk mengukur ketinggian/jarak beban.
- **ESP32 DevKit (30-pin, Port USB Type-C)**, **OLED 0.96"**, **Push Button**, dan **Pembagi Tegangan** di dalam box akrilik.

---

## 2. Goals, Non-Goals, & Anti-Goals

### Goals
- **Tata Letak PCB Matrix (25 × 29 lubang):**
  - Dudukan ESP32 menggunakan **2x Female Header 15-Pin (pitch 2.54 mm)** agar ESP32 bisa dicabut-pasang.
  - Dudukan OLED menggunakan Female Header 4-Pin.
  - Soket Push Button tactile switch.
  - Resistor pembagi tegangan 1 kΩ & 2 kΩ untuk Echo HC-SR04.
  - 2 Soket JST-XH 4-pin (`J_HX711`, `J_HCSR`).
  - **Pemasangan PCB:** Diletakkan langsung di atas lantai alas akrilik (tanpa spacer / standoff baut).
- **Desain Box Akrilik 2D (120 × 80 × 50 mm):**
  - Menggunakan format vektor SVG murni skala 1:1 (satuan mm).
  - 6 panel datar (butt joint untuk direkatkan dengan Lem G pada 4 dinding samping + alas).
  - **Tutup atas removable (sistem lem tembak di pojok-pojok)** agar mudah diservis, tidak butuh baut/standoff, dan aman dari uap lem G (*no blooming*).
  - Lubang jendela untuk layar OLED (31 × 26 mm) di tutup atas.
  - Lubang tombol push button (diameter 6.5 mm) di tutup atas.
  - Coakan port USB Type-C (14 × 9 mm, R=2 mm) di dinding kiri pada ketinggian pas (Z=14.5 mm dari sisi bawah dinding).
  - 1 U-Slot bersama kabel sensor di Dinding Belakang (14 × 8 mm) untuk bundel kabel HX711 & HC-SR04 bersolasi bakar.
  - **Alas Bawah:** 100% polos rata (solid).

### Non-Goals
- Pemrograman/firmware ESP32 (kodingan belum diterima dari owner).
- Sistem spacer/standoff baut untuk PCB.
- Sistem sambungan *finger-joint* gerigi luar.

---

## 3. Constraints & Z-Stackup Mekanik (Tanpa Spacer)

1. **Tebal Akrilik:** 3 mm.
2. **Dimensi Luar Box:** Panjang 120 mm, Lebar 80 mm, Tinggi 50 mm.
3. **Kalkulasi Z-Stackup (Ketinggian Vertikal Tanpa Spacer):**
   - Tebal alas akrilik: 3.0 mm
   - Spacer PCB: 0 mm (PCB ditaruh langsung di atas alas)
   - Toleransi solder kaki bawah PCB: ~1.5 mm
   - Tebal PCB Matrix: 1.6 mm
   - Tinggi Female Header ESP32: 8.5 mm
   - Tebal PCB ESP32: 1.2 mm
   - Posisi Tengah Port USB Type-C: `3.0 + 1.5 + 1.6 + 8.5 + 1.2 + 1.6 mm = ~17.4 mm` (~17.5 mm dari dasar luar box).
   - Coakan Type-C (12 × 8 mm) diposisikan pada ketinggian Z = 13.5 mm s/d 21.5 mm dari dasar luar (atau 10.5 mm s/d 18.5 mm dari sisi bawah dinding akrilik kiri).
4. **Konstruksi 6 Panel Datar (Tebal 3 mm):**
   - **Tutup Atas:** 120 mm × 80 mm (cut-out OLED 31 × 26 mm, lubang tombol d=6.5 mm, direkatkan lem tembak di pojok).
   - **Alas Bawah:** 120 mm × 80 mm (100% polos rata solid).
   - **Dinding Depan:** 120 mm × 44 mm (polos bersih).
   - **Dinding Belakang:** 120 mm × 44 mm (1 U-slot kabel 14 × 8 mm di sisi kanan).
   - **Dinding Kiri:** 74 mm × 44 mm (coakan Type-C 14 × 9 mm pada Z_center=14.5 mm dinding).
   - **Dinding Kanan:** 74 mm × 44 mm (polos bersih).
   *(Total tinggi terpasang: tebal alas 3 mm + tinggi dinding 44 mm + tebal tutup 3 mm = 50 mm)*.

---

## 5. Rincian Pemetaan Komponen & Pinout (ESP32 DevKit 30-Pin Type-C)

| Komponen | Pin Modul | Terhubung ke ESP32 / PCB | Keterangan |
| :--- | :--- | :--- | :--- |
| **HX711 (JST 4-Pin)** | Pin 1 (VCC) | 3.3V ESP32 | Tegangan logika HX711 |
| | Pin 2 (SCK) | GPIO 18 | Clock pulsa HX711 |
| | Pin 3 (DT) | GPIO 19 | Data sinyal bobot load cell |
| | Pin 4 (GND) | GND Bersama | Ground bersama |
| **HC-SR04 (JST 4-Pin)** | Pin 1 (VCC) | VIN (5V) | Suplai 5V dari USB Type-C |
| | Pin 2 (TRIG) | GPIO 5 | Trigger pulsa ultrasonik 10 µs |
| | Pin 3 (ECHO) | Divider 1 kΩ & 2 kΩ -> GPIO 17 | Pembagi tegangan aman ke GPIO 3.3V |
| | Pin 4 (GND) | GND Bersama | Ground bersama |
| **OLED 0.96" I2C** | GND | GND Bersama | Pin 1 soket OLED |
| | VCC | 3.3V ESP32 | Pin 2 soket OLED |
| | SCL | GPIO 22 | Pin 3 soket OLED (I2C SCL) |
| | SDA | GPIO 21 | Pin 4 soket OLED (I2C SDA) |
| **Push Button** | Pin 1 | GPIO 4 | Input push button (Internal Pull-Up) |
| | Pin 2 | GND | Kaki tombol ke ground |
