/*
  ==============================================================================
  KatroKit ESP32 - Program Pengujian Sensor Load Cell (HX711)
  ==============================================================================
  Program ini khusus untuk menguji modul HX711 & Load Cell secara mandiri,
  dilengkapi fitur KALIBRASI 2-SEGMEN (MULTI-POINT) untuk mengatasi ketidaklinieran
  beban sedang (tengah) dan beban maksimal.

  KONEKSI PIN:
    HX711 DT  -> GPIO 19
    HX711 SCK -> GPIO 18
    HX711 VCC -> 3.3V
    HX711 GND -> GND
    Tombol    -> BOOT Button bawaan ESP32 (GPIO 0)
    OLED (opsional): SDA -> GPIO 21, SCL -> GPIO 22, VCC -> 3.3V, GND -> GND

  PENGGUNAAN TOMBOL BOOT (GPIO 0):
    * Tekan singkat (< 2 detik) : TARE (Nol-kan timbangan).
    * Tahan lama   (> 3 detik) : Kalibrasi cepat 1 titik (default 240 gram).

  PERINTAH SERIAL MONITOR (115200 baud):
    * 'm'     : KALIBRASI 2-SEGMEN (Panduan langkah: 0g -> Tengah [100g] -> Max [240g]).
    * 't'     : TARE (Nol-kan timbangan saat kosong).
    * 'i'     : INVERT arah tanda gaya (balik nilai positif/negatif).
    * 'c 240' : Kalibrasi standar 1 titik (contoh: beban 240 gram).
    * 'r'     : Reset kalibrasi ke default pabrik.
    * '?'     : Tampilkan menu bantuan.
  ==============================================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HX711.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <ArduinoOTA.h>

// ---------- Konfigurasi Pin ----------
const int HX_DT   = 19;
const int HX_SCK  = 18;
const int BTN     = 0;   // Menggunakan tombol BOOT bawaan ESP32
const int SDA_PIN = 21;
const int SCL_PIN = 22;

// ---------- Objek & Variabel ----------
HX711 scale;
Preferences prefs;

// OLED (Opsional)
const int SCREEN_W = 128;
const int SCREEN_H = 64;
const uint8_t OLED_ADDR = 0x3C;
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);
bool oledOk = false;

// Kalibrasi Standar & Invert
float calFactor = 420.0f;          // Nilai awal aproksimasi
float forceSign = -1.0f;           // Pengali arah tanda gaya
const float CAL_REF_G = 240.0f;    // Beban referensi default (gram)
bool isCalibrated = false;

// Kalibrasi 2-Segmen (Multi-Point)
bool isMultiCal = false;
float cal1 = 420.0f;               // Faktor skala segmen 1 (0 -> mid)
float cal2 = 420.0f;               // Faktor skala segmen 2 (mid -> max)
float midGrams = 100.0f;           // Titik berat beban tengah (gram)
long midRaw = 0;                   // Nilai ADC pada beban tengah

// Filter Sederhana
const int N_AVG = 5;
float buf[N_AVG];
int bufIdx = 0;
bool bufFull = false;

long rawValue = 0;
float currentGrams = 0.0f;
float currentNewton = 0.0f;

// Tombol BOOT
bool btnDown = false;
unsigned long btnDownAt = 0;
unsigned long lastPrint = 0;

// ---------- Deklarasi Fungsi ----------
void printHelp();
void tareScale();
void calibrateScale(float knownGrams);
void calibrateMulti();
bool readSampleAverage(long &avgRaw, int samples);
void handleSerial();
void updateOLED();

float getFilteredGrams(float newGrams) {
  buf[bufIdx] = newGrams;
  bufIdx++;
  if (bufIdx >= N_AVG) {
    bufIdx = 0;
    bufFull = true;
  }
  int count = bufFull ? N_AVG : bufIdx;
  float sum = 0;
  for (int i = 0; i < count; i++) {
    sum += buf[i];
  }
  return sum / count;
}

// ---------- WiFiManager & ArduinoOTA (Non-Blocking Asinkron) ----------
WiFiManager wm;
bool otaStarted = false;
bool apStarted = false;

enum WiFiState {
  WIFI_CONNECTING,      // Sedang mencoba koneksi router di background (max 15 detik)
  WIFI_HOTSPOT_NOTIF,   // Hotspot aktif, tampilkan notifikasi di OLED selama 1 detik
  WIFI_CONNECTED_NOTIF, // WiFi terhubung, tampilkan notifikasi di OLED selama 1 detik
  WIFI_RUNNING          // Hotspot / WiFi aktif di background, OLED normal
};

WiFiState wifiState = WIFI_CONNECTING;
unsigned long wifiStartTime = 0;
unsigned long wifiNotifStart = 0;

// Setup dan Handler ArduinoOTA
void setupOTA() {
  ArduinoOTA.setHostname("katrokit-test");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {
      type = "filesystem";
    }
    Serial.println("\n[OTA] Mulai update firmware uji: " + type);
    if (oledOk) {
      display.clearDisplay();
      display.setTextColor(SSD1306_WHITE);
      display.setTextSize(1);
      display.setCursor(0, 15);
      display.println("OTA UPDATE (TEST)...");
      display.setCursor(0, 35);
      display.println("Mohon Tunggu...");
      display.display();
    }
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\n[OTA] Update Selesai & Berhasil! Restarting...");
    if (oledOk) {
      display.clearDisplay();
      display.setCursor(0, 20);
      display.println("UPDATE SUKSES!");
      display.setCursor(0, 40);
      display.println("Restarting ESP32...");
      display.display();
    }
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    unsigned int pct = progress / (total / 100);
    Serial.printf("[OTA] Progress: %u%%\r", pct);
    if (oledOk) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0, 10);
      display.println("UPDATING FIRMWARE");
      display.setCursor(45, 26);
      display.setTextSize(2);
      display.printf("%d%%", pct);
      display.setTextSize(1);
      int w = (pct * 128) / 100;
      display.drawRect(0, 50, 128, 10, SSD1306_WHITE);
      display.fillRect(2, 52, (w > 4 ? w - 4 : 0), 6, SSD1306_WHITE);
      display.display();
    }
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("[OTA] Error [%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Gagal");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Gagal");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Gagal");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Gagal");
    else if (error == OTA_END_ERROR) Serial.println("End Gagal");
  });

  ArduinoOTA.begin();
  Serial.println("[OTA] ArduinoOTA siap. Hostname: katrokit-test.local");
}

void initWiFiAsync() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(); // Non-blocking: hardware ESP32 mencoba koneksi di background FreeRTOS
  wifiStartTime = millis();
  wifiState = WIFI_CONNECTING;
  apStarted = false;
  Serial.println("[WIFI] Memulai pencarian WiFi tersimpan di background (timeout 15s)...");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n==================================================");
  Serial.println("  KATROKIT ESP32 - TEST SENSOR LOAD CELL (HX711)  ");
  Serial.println("==================================================");

  // Inisialisasi Tombol BOOT
  pinMode(BTN, INPUT_PULLUP);
  Serial.println("[OK] Tombol BOOT (GPIO 0) siap.");

  // Inisialisasi OLED (Jika terpasang)
  Wire.begin(SDA_PIN, SCL_PIN);
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (oledOk) {
    Serial.println("[OK] Layar OLED terdeteksi (0x3C).");
  } else {
    Serial.println("[INFO] OLED tidak terdeteksi (abaikan jika tidak dipasang).");
  }

  // Inisialisasi WiFi di background (0 ms blocking)
  initWiFiAsync();

  if (oledOk) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(10, 15);
    display.println("TEST LOAD CELL");
    display.setCursor(10, 35);
    display.println("Menyiapkan HX711...");
    display.display();
  }

  // Muat Kalibrasi dari Non-Volatile Memory (NVS)
  prefs.begin("katrokit", false);
  calFactor = prefs.getFloat("cal", 420.0f);
  forceSign = prefs.getFloat("sign", -1.0f);
  isMultiCal = prefs.getBool("is_multi", false);

  if (isMultiCal) {
    cal1 = prefs.getFloat("cal1", 420.0f);
    cal2 = prefs.getFloat("cal2", 420.0f);
    midGrams = prefs.getFloat("mid_w", 100.0f);
    midRaw = prefs.getLong("mid_raw", 0);
    isCalibrated = true;
    Serial.printf("[INFO] Mode Kalibrasi: 2-SEGMEN (0 -> %.0fg -> MAX)\n", midGrams);
    Serial.printf("       cal1 = %.2f | cal2 = %.2f | Arah = %.1f\n", cal1, cal2, forceSign);
  } else {
    isCalibrated = (calFactor != 1.0f && calFactor != 420.0f);
    Serial.printf("[INFO] Mode Kalibrasi: 1-TITIK STANDAR (calFactor = %.2f, Arah = %.1f)\n", calFactor, forceSign);
  }

  // Inisialisasi HX711
  Serial.print("[...] Menginisialisasi HX711 (DT=19, SCK=18)... ");
  scale.begin(HX_DT, HX_SCK);

  if (scale.wait_ready_timeout(2500)) {
    scale.tare(10);
    Serial.println("BERHASIL!");
  } else {
    Serial.println("GAGAL!");
    Serial.println(">>> PERINGATAN: Modul HX711 tidak merespons!");
    Serial.println("    1. Pastikan VCC ke 3.3V dan GND ke GND.");
    Serial.println("    2. Pastikan DT ke GPIO 19 dan SCK ke GPIO 18.");
  }

  printHelp();
}

void loop() {
  // --- MANAJEMEN WIFI & HOTSPOT ASINKRON (NON-BLOCKING) ---
  if (wifiState == WIFI_CONNECTING) {
    if (WiFi.status() == WL_CONNECTED) {
      setupOTA();
      otaStarted = true;
      wifiState = WIFI_CONNECTED_NOTIF;
      wifiNotifStart = millis();
      Serial.print("[WIFI] Terhubung ke WiFi! IP: ");
      Serial.println(WiFi.localIP());
    } else if (millis() - wifiStartTime >= 15000) {
      // 15 detik timeout: Aktifkan Hotspot AP di background
      wm.setConfigPortalBlocking(false);
      wm.startConfigPortal("KatroKit-Test-AP");
      apStarted = true;
      wifiState = WIFI_HOTSPOT_NOTIF;
      wifiNotifStart = millis();
      Serial.println("[WIFI] Timeout 15s tercapai. Hotspot 'KatroKit-Test-AP' aktif di background.");
    }
  } else if (wifiState == WIFI_HOTSPOT_NOTIF) {
    if (apStarted) wm.process();
    if (millis() - wifiNotifStart >= 1000) {
      wifiState = WIFI_RUNNING;
      lastPrint = 0; // Segera refresh OLED
    }
  } else if (wifiState == WIFI_CONNECTED_NOTIF) {
    if (millis() - wifiNotifStart >= 1000) {
      wifiState = WIFI_RUNNING;
      lastPrint = 0; // Segera refresh OLED
    }
  } else { // WIFI_RUNNING
    if (apStarted) {
      wm.process(); // Layani web server portal di background
    }
    if (WiFi.status() == WL_CONNECTED) {
      if (!otaStarted) {
        setupOTA();
        otaStarted = true;
        wifiState = WIFI_CONNECTED_NOTIF;
        wifiNotifStart = millis();
        Serial.print("[WIFI] Terhubung via Portal! IP: ");
        Serial.println(WiFi.localIP());
      }
      ArduinoOTA.handle();
    }
  }

  handleSerial();

  // Pembacaan Data Sensor
  if (scale.is_ready()) {
    rawValue = scale.get_value(1);
    float raw_adj = forceSign * (float)rawValue;
    float rawGrams = 0.0f;

    // Kalkulasi Berat: 2-Segmen vs 1-Titik
    if (isMultiCal && midRaw > 0 && cal1 > 0 && cal2 > 0) {
      if (raw_adj <= (float)midRaw) {
        rawGrams = raw_adj / cal1;
      } else {
        rawGrams = midGrams + ((raw_adj - (float)midRaw) / cal2);
      }
    } else {
      rawGrams = raw_adj / calFactor;
    }

    currentGrams = getFilteredGrams(rawGrams);
    if (fabs(currentGrams) < 0.5f) currentGrams = 0.0f; // Deadband filter
    currentNewton = (currentGrams / 1000.0f) * 9.81f;
  }

  // Pengendalian Tombol BOOT (GPIO 0: LOW saat ditekan)
  bool pressed = (digitalRead(BTN) == LOW);
  if (pressed && !btnDown) {
    btnDown = true;
    btnDownAt = millis();
  } else if (!pressed && btnDown) {
    btnDown = false;
    unsigned long heldMs = millis() - btnDownAt;
    if (heldMs >= 50) { // Debounce
      if (heldMs < 2000) {
        tareScale();
      } else {
        Serial.printf("\n[TOMBOL] Tahan %lu ms -> Memulai Kalibrasi 1-Titik %.0f g...\n", heldMs, CAL_REF_G);
        calibrateScale(CAL_REF_G);
      }
    }
  }

  // Output Serial & OLED tiap 250 ms
  if (millis() - lastPrint >= 250) {
    lastPrint = millis();

    // Serial Output
    Serial.printf("ADC_RAW: %8ld | Berat: %6.1f g | Gaya: %5.2f N | Mode: %s\n",
                  rawValue,
                  currentGrams,
                  currentNewton,
                  isMultiCal ? "MULTI" : (isCalibrated ? "1-TITIK" : "DEFAULT"));

    // OLED Output
    if (oledOk) {
      if (wifiState == WIFI_HOTSPOT_NOTIF) {
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(1);
        display.setCursor(18, 6);
        display.println("HOTSPOT AKTIF");
        display.drawFastHLine(0, 18, 128, SSD1306_WHITE);
        display.setCursor(0, 26);
        display.println("AP: KatroKit-Test-AP");
        display.setCursor(0, 42);
        display.println("IP: 192.168.4.1");
        display.display();
      } else if (wifiState == WIFI_CONNECTED_NOTIF) {
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(1);
        display.setCursor(14, 6);
        display.println("WIFI TERHUBUNG");
        display.drawFastHLine(0, 18, 128, SSD1306_WHITE);
        display.setCursor(0, 26);
        display.println(WiFi.SSID());
        display.setCursor(0, 42);
        display.println(WiFi.localIP().toString());
        display.display();
      } else {
        updateOLED();
      }
    }
  }
}

void tareScale() {
  Serial.print("\n>>> Melakukan TARE (Nol-kan timbangan)... ");
  if (scale.wait_ready_timeout(1000)) {
    scale.tare(10);
    for (int i = 0; i < N_AVG; i++) buf[i] = 0;
    Serial.println("SUKSES! Bobot di-nolkan.");
  } else {
    Serial.println("GAGAL! Sensor tidak siap.");
  }
}

bool readSampleAverage(long &avgRaw, int samples = 20) {
  if (!scale.wait_ready_timeout(2000)) return false;
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += scale.get_value(1);
    delay(50);
  }
  avgRaw = sum / samples;
  return true;
}

void calibrateScale(float knownGrams) {
  if (knownGrams <= 0) {
    Serial.println("[ERROR] Berat referensi harus lebih dari 0 gram.");
    return;
  }

  Serial.println("\n==================================================");
  Serial.printf("  PROSES KALIBRASI 1-TITIK (Beban: %.1f gram)    \n", knownGrams);
  Serial.println("==================================================");
  Serial.println("1. Pastikan beban tergantung diam di load cell...");
  delay(1500);

  long avgRaw = 0;
  if (!readSampleAverage(avgRaw, 20)) {
    Serial.println("[GAGAL] HX711 tidak siap untuk kalibrasi.");
    return;
  }

  if (labs(avgRaw) < 100) {
    Serial.println("[GAGAL] Perubahan nilai ADC terlalu kecil. Periksa beban atau kabel!");
    return;
  }

  forceSign = (avgRaw < 0) ? -1.0f : 1.0f;
  calFactor = fabs((float)avgRaw / knownGrams);
  isCalibrated = true;
  isMultiCal = false; // Mode 1-titik aktif

  // Simpan ke Flash NVS
  prefs.putBool("is_multi", false);
  prefs.putFloat("cal", calFactor);
  prefs.putFloat("sign", forceSign);

  Serial.printf(">> Kalibrasi 1-Titik SELESAI!\n");
  Serial.printf("   Nilai ADC Rata-rata : %ld\n", avgRaw);
  Serial.printf("   Faktor Kalibrasi    : %.2f\n", calFactor);
  Serial.printf("   Arah Gaya (Sign)    : %.1f\n", forceSign);
  Serial.println(">> Tersimpan di Flash memory ESP32.\n");
}

void calibrateMulti() {
  Serial.println("\n========================================================");
  Serial.println("  KALIBRASI 2-SEGMEN (MULTI-POINT: 0g -> TENGAH -> MAX) ");
  Serial.println("========================================================");
  Serial.println("Pilihan kombinasi beban Anda (50gx4, 20gx2):");
  Serial.println("  * Beban Tengah  : misal 100g (2x 50g) atau 120g (2x 50g + 1x 20g)");
  Serial.println("  * Beban Maksimal: 240g (semua beban dipasang)");
  Serial.println("--------------------------------------------------------");

  // Langkah 1: Tare titik nol
  Serial.println("\n[LANGKAH 1/3] KOSONGKAN KAIT (0 gram).");
  Serial.println(">> Ketik sembarang karakter lalu Enter setelah kait kosong:");
  while (true) {
    if (Serial.available()) {
      String s = Serial.readStringUntil('\n');
      s.trim();
      if (s.length() > 0) break;
    }
    delay(50);
  }
  Serial.print(">> Mengunci titik nol (Tare 20 sampel)... ");
  scale.tare(20);
  for (int i = 0; i < N_AVG; i++) buf[i] = 0;
  Serial.println("SUKSES (Titik 0g terkunci).");

  // Langkah 2: Beban Tengah
  Serial.println("\n[LANGKAH 2/3] GANTUNG BEBAN TENGAH (misal 100g atau 120g).");
  Serial.println(">> Masukkan angka berat dalam gram (contoh: 100):");
  float wMid = 0;
  while (wMid <= 0) {
    if (Serial.available()) {
      String s = Serial.readStringUntil('\n');
      s.trim();
      wMid = s.toFloat();
      if (wMid <= 0) {
        Serial.println(">> Angka tidak valid. Masukkan angka (misal 100):");
      }
    }
    delay(50);
  }
  Serial.printf(">> Beban tengah: %.1f g. Mengambil 20 sampel stabil...\n", wMid);
  delay(1500);

  long rawMid = 0;
  if (!readSampleAverage(rawMid, 20)) {
    Serial.println("[GAGAL] HX711 tidak merespons.");
    return;
  }
  long adjMid = labs(rawMid);
  if (adjMid < 100) {
    Serial.println("[GAGAL] Nilai ADC terlalu kecil. Periksa gantungan beban!");
    return;
  }
  float c1 = (float)adjMid / wMid;
  Serial.printf(">> Titik Tengah OK! ADC: %ld | cal1: %.2f\n", rawMid, c1);

  // Langkah 3: Beban Maksimal
  Serial.println("\n[LANGKAH 3/3] GANTUNG BEBAN MAKSIMAL (contoh: 240g).");
  Serial.println(">> Masukkan angka berat maksimal dalam gram (contoh: 240):");
  float wMax = 0;
  while (wMax <= wMid) {
    if (Serial.available()) {
      String s = Serial.readStringUntil('\n');
      s.trim();
      wMax = s.toFloat();
      if (wMax <= wMid) {
        Serial.printf(">> Beban maksimal harus lebih besar dari %.1f g (contoh: 240):\n", wMid);
      }
    }
    delay(50);
  }
  Serial.printf(">> Beban maksimal: %.1f g. Mengambil 20 sampel stabil...\n", wMax);
  delay(1500);

  long rawMax = 0;
  if (!readSampleAverage(rawMax, 20)) {
    Serial.println("[GAGAL] HX711 tidak merespons.");
    return;
  }
  long adjMax = labs(rawMax);
  if (adjMax <= adjMid + 50) {
    Serial.println("[GAGAL] Nilai ADC maksimal tidak bertambah secara wajar.");
    return;
  }
  float c2 = (float)(adjMax - adjMid) / (wMax - wMid);
  Serial.printf(">> Titik Maksimal OK! ADC: %ld | cal2: %.2f\n", rawMax, c2);

  // Hitung polaritas arah
  forceSign = (rawMax < 0) ? -1.0f : 1.0f;

  // Simpan data Multi-Segmen
  isMultiCal = true;
  cal1 = c1;
  cal2 = c2;
  midGrams = wMid;
  midRaw = adjMid;
  calFactor = (float)adjMax / wMax; // Fallback untuk mode single
  isCalibrated = true;

  prefs.putBool("is_multi", true);
  prefs.putFloat("cal1", cal1);
  prefs.putFloat("cal2", cal2);
  prefs.putFloat("mid_w", midGrams);
  prefs.putLong("mid_raw", midRaw);
  prefs.putFloat("cal", calFactor);
  prefs.putFloat("sign", forceSign);

  Serial.println("\n========================================================");
  Serial.println(">> KALIBRASI 2-SEGMEN BERHASIL DISIMPAN KE FLASH ESP32!");
  Serial.printf("   Segmen 1 (0 s/d %.0f g)    : cal1 = %.2f\n", midGrams, cal1);
  Serial.printf("   Segmen 2 (%.0f s/d %.0f g) : cal2 = %.2f\n", midGrams, wMax, cal2);
  Serial.printf("   Arah Tanda (Sign)         : %.1f\n", forceSign);
  Serial.println(">> Sekarang beban tengah maupun beban maksimal akan presisi!");
  Serial.println("========================================================\n");
}

void handleSerial() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  char c = cmd.charAt(0);
  if (c == 't' || c == 'T') {
    tareScale();
  } else if (c == 'm' || c == 'M') {
    calibrateMulti();
  } else if (c == 'i' || c == 'I') {
    forceSign = -forceSign;
    prefs.putFloat("sign", forceSign);
    Serial.printf("[INVERT] Arah tanda dibalik: %.1f (%s)\n",
                  forceSign, forceSign > 0 ? "Normal / Positif" : "Dibalik / Inverted");
  } else if (c == 'c' || c == 'C') {
    float grams = cmd.substring(1).toFloat();
    if (grams <= 0) grams = CAL_REF_G;
    calibrateScale(grams);
  } else if (c == 'r' || c == 'R') {
    prefs.putBool("is_multi", false);
    prefs.putFloat("cal", 420.0f);
    prefs.putFloat("sign", -1.0f);
    calFactor = 420.0f;
    forceSign = -1.0f;
    isMultiCal = false;
    isCalibrated = false;
    Serial.println("[RESET] Kalibrasi dikembalikan ke default (420.0).");
  } else if (c == '?' || c == 'h' || c == 'H') {
    printHelp();
  }
}

void printHelp() {
  Serial.println("\n--- MENU BANTUAN UJI LOAD CELL ---");
  Serial.println("  t       : TARE (Nol-kan pembacaan saat kosong)");
  Serial.println("  m       : KALIBRASI 2-SEGMEN (Panduan langkah: 0g -> Tengah -> Max 240g)");
  Serial.println("  i       : INVERT arah tanda (+ / -)");
  Serial.println("  c <g>   : Kalibrasi standar 1 titik (contoh: c 240)");
  Serial.println("  r       : Reset kalibrasi ke nilai awal");
  Serial.println("  ?       : Tampilkan menu ini lagi");
  Serial.println("  Tombol  : Tekan singkat BOOT = TARE | Tahan >3s = Kalibrasi 240g");
  Serial.println("-----------------------------------\n");
}

void updateOLED() {
  display.clearDisplay();

  // Header
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("LOAD CELL");

  display.setCursor(65, 0);
  display.print(isMultiCal ? "[MULTI]" : "[1-PT]");

  display.setCursor(108, 0);
  display.print(scale.is_ready() ? "OK" : "ERR");

  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  // Nilai Berat
  display.setTextSize(2);
  display.setCursor(0, 16);
  display.printf("%6.1f g", currentGrams);

  // Nilai Gaya (Newton) & ADC
  display.setTextSize(1);
  display.setCursor(0, 36);
  display.printf("Gaya : %.2f N", currentNewton);

  display.setCursor(0, 47);
  display.printf("ADC  : %ld", rawValue);

  // Footer petunjuk tombol
  display.setCursor(0, 57);
  display.print("BOOT: Tekan -> Tare");

  display.display();
}
