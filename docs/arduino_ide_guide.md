# 🛠️ Panduan Instalasi & Flashing via Arduino IDE

Panduan ini ditujukan bagi Anda yang ingin meng-compile dan meng-upload firmware **KatroKit ESP32** menggunakan aplikasi **Arduino IDE** (versi 2.x direkomendasikan).

File sketch Arduino IDE telah disiapkan dan berada di folder:
👉 **[`arduino/KatroKit_esp32/KatroKit_esp32.ino`](file:///Volumes/Data%20Shared/Project/KatroKit_esp32/arduino/KatroKit_esp32/KatroKit_esp32.ino)**

---

## 1. Instalasi Board ESP32 di Arduino IDE

Jika Anda belum menginstal dukungan board ESP32 di Arduino IDE:

1. Buka **Arduino IDE**.
2. Masuk ke menu **File** > **Preferences** (atau **Arduino IDE** > **Settings** di macOS).
3. Pada kolom **Additional Boards Manager URLs**, tempelkan URL resmi Espressif berikut:
   ```text
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
   *(Jika sudah ada URL lain, pisahkan dengan tanda koma).*
4. Klik **OK**.
5. Buka **Boards Manager** (ikon chip di bilah sisi kiri, atau menu **Tools** > **Board** > **Boards Manager...**).
6. Ketik `esp32` pada kotak pencarian, temukan package **esp32 by Espressif Systems**, lalu klik **Install**.

---

## 2. Daftar Library yang Harus Diinstall

Buka **Library Manager** di Arduino IDE (ikon buku di bilah sisi kiri, atau menu **Sketch** > **Include Library** > **Manage Libraries...**). Cari dan instal 6 library berikut:

| No | Nama Library | Author / Pembuat | Versi yang Dianjurkan | Catatan Penting |
| :-: | :--- | :--- | :-: | :--- |
| **1** | **Adafruit SSD1306** | Adafruit | Terbaru (v2.5.x) | Jika muncul jendela pop-up *"Install all dependencies"*, klik **Install All** (akan menginstal otomatis *Adafruit BusIO*). |
| **2** | **Adafruit GFX Library** | Adafruit | Terbaru (v1.11.x+) | Library grafis dasar untuk layar OLED SSD1306. |
| **3** | **HX711 Arduino Library** | Bogdan Necula | Terbaru (v0.7.5) | Driver komunikasi serial Load Cell ADC 24-bit. |
| **4** | **WiFiManager** | tzapu | v2.0.16 atau v2.0.17 | Menangani captive portal hotspot `KatroKit-AP` saat WiFi belum terhubung. |
| **5** | **PubSubClient** | Nick O'Leary | v2.8.0 | Client protokol MQTT untuk streaming data ke HiveMQ. |
| **6** | **ArduinoJson** | Benoit Blanchon | **Wajib Versi 7.x** | Parser dan serialisasi JSON payload MQTT telemetri. |

> [!WARNING]
> Untuk **ArduinoJson**, pastikan menginstal **versi 7.x** (bukan versi 6.x), karena kode menggunakan sintaks modern `JsonDocument doc;`.

---

## 3. Pengaturan Menu Tools (Board Settings)

Buka sketch [`arduino/KatroKit_esp32/KatroKit_esp32.ino`](file:///Volumes/Data%20Shared/Project/KatroKit_esp32/arduino/KatroKit_esp32/KatroKit_esp32.ino) di Arduino IDE, lalu sesuaikan pengaturan pada menu **Tools**:

- **Board:** `ESP32 Arduino` > `ESP32 Dev Module`
- **CPU Frequency:** `240MHz (WiFi/BT)`
- **Flash Frequency:** `80MHz`
- **Flash Mode:** `QIO`
- **Flash Size:** `4MB (32Mb)`
- **Partition Scheme:** `Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)` ⭐ **(PENTING!)**
  > *Ukuran biner firmware KatroKit adalah ~974 KB karena mencakup driver FreeRTOS dual-core, WiFi, OTA, dan MQTT. Skema "Minimal SPIFFS" memberi ruang partisi aplikasi hingga 1.9MB sehingga fitur nirkabel OTA muat sempurna.*
- **Core Debug Level:** `None`
- **Upload Speed:** `921600` (atau turunkan ke `460800` jika kabel USB Anda panjang)
- **Port:** Pilih port USB serial mikrokontroler Anda (contoh: `/dev/cu.usbserial-xxxx` di Mac atau `COM3` / `COM4` di Windows).

---

## 4. Cara Upload (Flashing)

1. Sambungkan ESP32 ke laptop menggunakan kabel data USB Type-C.
2. Pastikan port COM yang benar telah dipilih pada menu **Tools** > **Port**.
3. Klik tombol **Upload** (ikon panah ke kanan `➜`).
4. Tunggu hingga proses kompilasi dan upload selesai bertuliskan *"Leaving... Hard resetting via RTS pin..."*.
5. Layar OLED KatroKit akan menyala dan menampilkan logo serta animasi splash!

---

## 5. Fitur Reset WiFi (Jika Ingin Pindah Router)

Firmware ini sudah dilengkapi fitur **Reset WiFi via Tombol D4**:
1. Cabut kabel daya/USB ESP32.
2. Tekan dan tahan tombol **D4**, lalu colokkan kembali kabel USB.
3. Layar OLED akan menampilkan countdown 3 detik:
   ```text
        RESET WIFI?
   ---------------------
   Tahan tombol: 3 dtk
   Lepas utk batalkan
   [====               ]
   ```
4. Tahan terus selama 3 detik hingga muncul teks **"WIFI DIRESET! Restarting..."**.
5. Kredensial WiFi lama terhapus, dan ESP32 akan menyalakan hotspot `KatroKit-AP` (IP `192.168.4.1`) untuk konfigurasi jaringan baru.

---

## 6. Operasi Tombol D4 & Pemilihan Mode Katrol

Tombol **D4** mendukung multi-fungsi cerdas:
- **Klik 1x:** `TARE GAYA` (Nol-kan sensor gaya saja, tanpa mengubah titik referensi jarak `distRef`).
- **Klik 2x (Cepat <380ms):** `GANTI MODE KATROL` bergantian:
  1. **Katrol Tetap:** $KM\text{ Teori} = 1.0$, $F_{\text{kuasa}} = \frac{F_{\text{sensor}}}{2}$ *(mengoreksi beban tali ganda pada katrol gantung)*.
  2. **Katrol Bebas:** $KM\text{ Teori} = 2.0$, $F_{\text{kuasa}} = F_{\text{sensor}}$.
  3. **Katrol Majemuk:** $KM\text{ Teori} = 2.0$, $F_{\text{kuasa}} = F_{\text{sensor}}$.
  > *Mode katrol otomatis tersimpan ke Flash NVS dan akan terus diingat saat alat dimatikan.*
- **Klik 3x (Cepat <380ms):** `SET JARAK 0 cm` (Menyimpan jarak sensor ultrasonik saat ini sebagai referensi nol `distRef` ke Flash NVS; nilai default awal `3.0 cm`).
- **Tahan 1.5 - 3 detik:** Timbang bobot gantung saat ini sebagai nilai beban $W$.
- **Tahan 3 - 5 detik:** Menu Preset Beban $W$ cepat ($50\text{g}$, $100\text{g}$, $150\text{g}$, $200\text{g}$, $250\text{g}$, $500\text{g}$).
- **Tahan > 5 detik:** Wizard Kalibrasi Load Cell.


