# Panduan Wiring & Layout PCB Matrix: Alat Uji Katrol & Beban

Dokumen ini memuat panduan tata letak (*layout*) dan penyolderan soket **JST-XH** pada **PCB Matrix / Perfboard** untuk sistem praktikum uji katrol beban (**ESP32 Type-C + HX711 Load Cell + HC-SR04 + OLED 0.96" + Tombol**).

Diagram denah visual vektor resolusi penuh dapat dibuka pada:
👉 **[docs/pcb_matrix_loadcell_layout.svg](file:///Volumes/Data%20Shared/Project/KatroKit_esp32/docs/pcb_matrix_loadcell_layout.svg)**

---

## 1. Orientasi & Penempatan Komponen Fisik di PCB Matrix

* **Dimensi Grid Lubang:** Kolom **A – Y (25 Lubang)** × Baris **1 – 29 (29 Lubang)**.
* **4 Lubang Baut / Standoff (2×2 Lubang Bebas di Tiap Pojokan):**
  * Pojok Kiri Atas : Kolom **A – B**, Baris **1 – 2**
  * Pojok Kanan Atas : Kolom **X – Y**, Baris **1 – 2**
  * Pojok Kiri Bawah : Kolom **A – B**, Baris **28 – 29**
  * Pojok Kanan Bawah : Kolom **X – Y**, Baris **28 – 29**
* **Aturan Spasi Antar-Soket JST:** Minimal menyisakan **1 lubang kosong (2.54mm)** di antara dua soket JST agar bodi plastik konektor tidak saling menjepit/bertabrakan saat kabel dicabut-pasang.
* **Posisi Port USB Type-C ESP32:** Menghadap ke **Kiri** (dekat Kolom C).
* **Sisi Atas (Top Row ESP32):** Baris **10** (Kolom E s/d S).
* **Sisi Bawah (Bottom Row ESP32):** Baris **20** (Kolom E s/d S).

---

## 2. Pengelompokan Soket JST-XH di PCB Matrix

### A. Sisi Atas PCB Matrix (Baris 1 – 2): Sensor Eksternal
1. **`J_HX711` — Modul Load Cell (JST 4-Pin) [Kolom C, D, E, F]**
   * Pin 1 (Kolom C) : VCC (3.3V)
   * Pin 2 (Kolom D) : SCK (ke GPIO 18 / Kolom M Baris 20)
   * Pin 3 (Kolom E) : DT (ke GPIO 19 / Kolom N Baris 20)
   * Pin 4 (Kolom F) : GND (ke Rel Ground Sisi Atas)
2. **`[SPASI]` — Kolom G (Kosong 1 Lubang)**
3. **`J_HCSR` — Sensor Ultrasonik Beban (JST 4-Pin) [Kolom H, I, J, K]**
   * Pin 1 (Kolom H) : VCC / VIN (+5V Daya Utama)
   * Pin 2 (Kolom I) : TRIG (ke GPIO 5 / Kolom L Baris 20)
   * Pin 3 (Kolom J) : ECHO (ke Resistor 1 kΩ pembagi tegangan)
   * Pin 4 (Kolom K) : GND (ke Rel Ground Sisi Atas)
4. **`AREA R DIVIDER` — Pembagi Tegangan Echo (Kolom M – O, Baris 5 – 6):**
   * Resistor 1 kΩ: dari Pin 3 `J_HCSR` (Kolom J) ke GPIO 17 (Kolom K Baris 20).
   * Resistor 2 kΩ: dari GPIO 17 ke GND.

---

### B. Sisi Bawah PCB Matrix (Baris 28 – 29): Antarmuka Pengguna (UI)
1. **`J_OLED` — Layar OLED 0.96" I2C (JST 4-Pin) [Kolom C, D, E, F]**
   * Pin 1 (Kolom C) : GND
   * Pin 2 (Kolom D) : VCC (3.3V)
   * Pin 3 (Kolom E) : SCL (ke GPIO 22 / Kolom P Baris 20)
   * Pin 4 (Kolom F) : SDA (ke GPIO 21 / Kolom O Baris 20)
2. **`[SPASI]` — Kolom G (Kosong 1 Lubang)**
3. **`J_BTN` — Push Button Taktil (JST 2-Pin) [Kolom H, I]**
   * Pin 1 (Kolom H) : Sinyal Tombol (ke GPIO 4 / Kolom I Baris 20, `INPUT_PULLUP`)
   * Pin 2 (Kolom I) : GND

---

## 3. Pemetaan Pin Lengkap ESP32 DevKit V1 30-Pin Type-C

### Baris Pin Atas (Baris 10, Y = 514):
| Kolom | Nama Pin | Fungsi di Rangkaian | Keterangan |
| :-: | :--- | :--- | :--- |
| **E** | **VIN** | Rel Daya +5V Utama | Menyuplai Pin 1 `J_HCSR` |
| **F** | **GND** | Rel Ground Sisi Atas | Bus Ground Utama |
| **G - J** | **D13 s/d D27** | *Tidak Digunakan (NC)* | Kosong |
| **K - P** | **D26 s/d D34** | *Tidak Digunakan (NC)* | Kosong |
| **Q - S** | **VN, VP, EN** | *Tidak Digunakan (NC)* | Kosong |

### Baris Pin Bawah (Baris 20, Y = 934):
| Kolom | Nama Pin | Fungsi di Rangkaian | Keterangan |
| :-: | :--- | :--- | :--- |
| **E** | **3V3** | Output Daya 3.3V ESP32 | Menyuplai Pin 1 `J_HX711` & Pin 2 `J_OLED` |
| **F** | **GND** | Rel Ground Sisi Bawah | Bus Ground Utama |
| **G - H** | **D15, D2** | *Tidak Digunakan (NC)* | Kosong |
| **I** | **D4** | Input Push Button | Terhubung ke Pin 1 `J_BTN` (`INPUT_PULLUP`) |
| **J** | **RX2** | *Tidak Digunakan (NC)* | Kosong |
| **K** | **TX2 (GPIO 17)**| Input Echo Ultrasonik | Menerima sinyal Echo dari titik tengah Divider R 1k/2k (dari Pin 3 `J_HCSR`) |
| **L** | **D5** | Trigger Ultrasonik | Terhubung ke Pin 2 `J_HCSR` (TRIG) |
| **M** | **D18** | Clock HX711 | Terhubung ke Pin 2 `J_HX711` (SCK) |
| **N** | **D19** | Data HX711 | Terhubung ke Pin 3 `J_HX711` (DT) |
| **O** | **D21** | I2C Data (SDA) | Terhubung ke Pin 4 `J_OLED` (SDA) |
| **P** | **D22** | I2C Clock (SCL) | Terhubung ke Pin 3 `J_OLED` (SCL) |
| **Q - S** | **TX0, RX0, D23**| *Tidak Digunakan (NC)* | Kosong |

---

## 4. Skematik Pembagi Tegangan (*Voltage Divider*) Echo 5V ke 3.3V

```text
Pin ECHO (Pin 3 J_HCSR) (5V) ────[ Resistor 1 kΩ ]────┬──── Ke Pin GPIO 17 ESP32 (~3.3V)
                                              │
                                      [ Resistor 2 kΩ ]
                                              │
                                             GND
```

* Perhitungan Tegangan Input: `V_out = 5V × (2 kΩ / (1 kΩ + 2 kΩ)) = 3.33V` (Aman untuk mikrokontroler 3.3V).
