/*
  KatroKit - ESP32  v2  (tanpa laptop / tanpa Serial Monitor)
  Load cell + HX711 (gaya tarik), HC-SR04 (jarak/perpindahan), OLED SSD1306 I2C 0.96"

  PIN KONEKTOR JST-XH (PCB Matrix):
    HX711   : Pin 1:VCC(3.3V), Pin 2:SCK(GPIO 18), Pin 3:DT(GPIO 19), Pin 4:GND
    HC-SR04 : Pin 1:VCC(5V), Pin 2:TRIG(GPIO 5), Pin 3:ECHO(GPIO 17 via Divider 1k/2k), Pin 4:GND
    OLED    : Pin 1:GND, Pin 2:VCC(3.3V), Pin 3:SCL(GPIO 22), Pin 4:SDA(GPIO 21)
    Tombol  : Push button eksternal box akrilik (soket JST GPIO 4)

  SATU TOMBOL, BEBERAPA AKSI (tahan lalu LEPAS; layar menunjukkan aksinya):
    < 1,5 dtk   -> TARE gaya + nol-kan jarak
    1,5 - 3 dtk -> TIMBANG W   (beban digantung langsung di load cell)
    3 - 5 dtk   -> PILIH W     (ganti W ke preset berikutnya: 100/200/250/300/500 g)
    > 5 dtk     -> KALIBRASI   (pakai beban referensi CAL_REF_G, default 200 g)

  URUTAN PAKAI PERTAMA KALI (tanpa laptop):
    1. Kait kosong -> tekan singkat (TARE).
    2. Gantung beban 200 g di load cell, diam -> tahan BOOT > 5 dtk, lepas (KALIBRASI).
    3. Kait kosong -> TARE lagi. Alat siap.
    4. Pilih W: timbang (1,5-3 dtk) ATAU pilih preset (3-5 dtk).
    5. Pasang beban di katrol -> TARE -> tarik.

  Perintah Serial Monitor tetap ada (opsional):
    t z r i d m | c<gram> | c0 | b<gram>
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

// ---------- Pin ----------
const int HX_DT   = 19;
const int HX_SCK  = 18;
const int TRIG    = 5;
const int ECHO    = 17;
const int SDA_PIN = 21;
const int SCL_PIN = 22;
const int BTN     = 4;   // tombol push button eksternal (soket JST GPIO 4)

// ---------- Pengaturan ----------
const float CAL_REF_G = 240.0f;                       // beban referensi kalibrasi 1-titik (gram)
const float PRESETS[] = {50, 100, 120, 140, 200, 240}; // pilihan W lewat tombol (gram)
const int   N_PRESETS = 6;

const unsigned long T_WEIGH  = 1500;   // batas tahan: tare -> timbang
const unsigned long T_PRESET = 3000;   // timbang -> pilih W
const unsigned long T_CAL    = 5000;   // pilih W -> kalibrasi

// ---------- OLED ----------
const int SCREEN_W = 128;
const int SCREEN_H = 64;
const uint8_t OLED_ADDR = 0x3C;   // jika layar kosong, coba 0x3D
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);
bool oledOk = false;

// ---------- HX711 & Kalibrasi ----------
HX711 scale;
Preferences prefs;
const float G = 9.81f;
float calFactor = 1.0f;
float forceSign = -1.0f;

// Kalibrasi 2-Segmen (Multi-Point)
bool isMultiCal = false;
float cal1 = 1.0f;
float cal2 = 1.0f;
float midGrams = 100.0f;
long midRaw = 0;

// Konversi RAW ADC ke Newton & Gram
float rawToNewtons(float raw) {
  float raw_adj = forceSign * raw;
  if (isMultiCal && midRaw > 0 && cal1 > 0 && cal2 > 0) {
    float grams = 0.0f;
    if (raw_adj <= (float)midRaw) {
      grams = raw_adj / cal1;
    } else {
      grams = midGrams + ((raw_adj - (float)midRaw) / cal2);
    }
    return (grams / 1000.0f) * G;
  }
  return raw_adj / calFactor;
}

float rawToGrams(float raw) {
  return (rawToNewtons(raw) / G) * 1000.0f;
}

// ---------- Filter ----------
const int F_N = 3;
float fBuf[F_N];
int fIdx = 0;
bool fFull = false;
long lastRaw = 0;
float force = 0, forceMax = 0, usaha = 0;
float bebanGram = 500.0f;
float bebanW = 0.0f;
float km = 0.0f;
bool isCalibrated = false;

const int D_N = 3;
float dBuf[D_N];
int dIdx = 0;
bool dFull = false;
float distCm = 0;
float distRef = 0;
bool distValid = false;

unsigned long lastDist = 0, lastDraw = 0;

// ---------- Tombol ----------
bool btnDown = false;
unsigned long btnDownAt = 0;

// ---------- Timbang beban ----------
const float WEIGH_MIN_G    = 20.0f;
const float WEIGH_MAX_G    = 1000.0f;
const float WEIGH_STABLE_N = 0.08f;
const int   SAMPLES        = 20;

// ---------- Fungsi bantu ----------
float avgBuf(float *buf, int n, bool full, int idx) {
  int cnt = full ? n : idx;
  if (cnt == 0) return 0;
  float s = 0;
  for (int i = 0; i < cnt; i++) s += buf[i];
  return s / cnt;
}

float readDistanceCm() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  unsigned long dur = pulseIn(ECHO, HIGH, 15000UL);
  if (dur == 0) return -1;
  return dur * 0.0343f / 2.0f;
}

bool hxReady() {
  if (scale.wait_ready_timeout(1000)) return true;
  Serial.println("HX711 tidak merespons! Cek DT/SCK, VCC 3.3V dan GND.");
  return false;
}

// Pesan 3 baris di OLED + tahan sebentar
void showMsg(const char *l1, const char *l2 = "", const char *l3 = "", int holdMs = 0) {
  if (oledOk) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 8);  display.print(l1);
    display.setCursor(0, 26); display.print(l2);
    display.setCursor(0, 44); display.print(l3);
    display.display();
  }
  if (holdMs > 0) delay(holdMs);
}

// Ambil SAMPLES bacaan mentah (sudah dikurangi tare). true jika cukup data.
bool sampleRaw(float &mean, float &spread) {
  float sum = 0, mn = 1e12f, mx = -1e12f;
  int got = 0;
  unsigned long t0 = millis();
  while (got < SAMPLES && millis() - t0 < 4000) {
    if (scale.is_ready()) {
      float r = (float)scale.get_value(1);
      sum += r;
      if (r < mn) mn = r;
      if (r > mx) mx = r;
      got++;
    }
  }
  if (got < SAMPLES) return false;
  mean = sum / got;
  spread = mx - mn;
  return true;
}

void tareForce() {
  if (!hxReady()) return;
  scale.tare(10);
  fIdx = 0; fFull = false;
  force = 0; forceMax = 0; km = 0; usaha = 0;
  Serial.println("Tare selesai.");
}

void zeroDistance() {
  if (distValid) {
    distRef = distCm;
    Serial.print("Jarak awal diset: ");
    Serial.print(distRef, 1);
    Serial.println(" cm");
  }
}

void resetCalibration() {
  calFactor = 1.0f;
  isMultiCal = false;
  prefs.putBool("is_multi", false);
  prefs.putFloat("cal", 1.0f);
  isCalibrated = false;
  Serial.println("Kalibrasi di-reset (!CAL).");
}

// Kalibrasi dengan beban referensi yang SEDANG menggantung di load cell
// (load cell sudah di-tare saat kosong).
void calibrate(float grams) {
  if (grams > 1000.0f) {
    Serial.println("Kapasitas load cell 1 kg! Pakai beban referensi <= 1000 g.");
    showMsg("MELEBIHI 1 KG", "Pakai beban <= 1kg", "", 1800);
    return;
  }
  if (!hxReady()) return;

  char b1[24];
  snprintf(b1, sizeof(b1), "Referensi %.0f g", grams);
  showMsg("KALIBRASI...", b1, "Jaga beban diam");
  Serial.println("Kalibrasi dimulai... beban harus menggantung diam.");
  delay(800);

  float raw, spread;
  if (!sampleRaw(raw, spread)) {
    showMsg("GAGAL", "HX711 lambat/putus", "", 1800);
    return;
  }
  if (fabs(raw) < 50.0f) {
    Serial.println("Gagal: tidak ada beban terdeteksi. Tare dulu saat kosong, lalu gantung beban.");
    showMsg("BEBAN TAK TERBACA", "TARE saat kosong,", "lalu gantung beban", 2200);
    return;
  }
  if (spread > 0.05f * fabs(raw)) {
    Serial.println("Gagal: beban belum diam.");
    showMsg("BEBAN GOYANG", "Tunggu diam lalu", "ulangi kalibrasi", 2000);
    return;
  }

  float newton = (grams / 1000.0f) * G;
  forceSign = (raw < 0) ? -1.0f : 1.0f;
  prefs.putFloat("sign", forceSign);
  calFactor = fabs(raw) / newton;
  prefs.putFloat("cal", calFactor);
  isMultiCal = false;
  prefs.putBool("is_multi", false);
  fIdx = 0; fFull = false; forceMax = 0;
  isCalibrated = true;

  Serial.println("=== KALIBRASI SUKSES ===");
  Serial.print("Massa referensi: "); Serial.print(grams, 1); Serial.println(" g");
  Serial.print("Nilai RAW ADC  : "); Serial.println(raw, 0);
  Serial.print("Faktor Skala   : "); Serial.println(calFactor, 2);
  Serial.print("Arah Gaya      : "); Serial.println(forceSign > 0 ? "Normal (+)" : "Dibalik (-)");

  showMsg("KALIBRASI OK", b1, "Kait kosong -> TARE", 2500);
}

// Kalibrasi 2-Segmen (0g -> Tengah -> Maksimal)
void calibrateMulti() {
  Serial.println("\n========================================================");
  Serial.println("  KALIBRASI 2-SEGMEN (MULTI-POINT: 0g -> TENGAH -> MAX) ");
  Serial.println("========================================================");
  Serial.println("Panduan beban yang Anda miliki (50gx4, 20gx2):");
  Serial.println("  * Beban Tengah  : misal 100g (2x 50g) atau 120g (2x 50g + 1x 20g)");
  Serial.println("  * Beban Maksimal: 240g (semua beban dipasang)");
  Serial.println("--------------------------------------------------------");

  showMsg("KALIBRASI MULTI", "Langkah 1/3", "Kait kosong -> Enter", 1000);

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
  fIdx = 0; fFull = false; forceMax = 0;
  Serial.println("SUKSES (Titik 0g terkunci).");

  showMsg("LANGKAH 2/3", "Gantung Beban Mid", "Ketik gram di Serial", 1000);

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

  float rawMidF = 0, spreadMid = 0;
  if (!sampleRaw(rawMidF, spreadMid)) {
    Serial.println("[GAGAL] HX711 tidak merespons atau beban goyang.");
    showMsg("GAGAL", "HX711 lambat/goyang", "", 1800);
    return;
  }
  long adjMid = labs((long)rawMidF);
  if (adjMid < 50) {
    Serial.println("[GAGAL] Nilai ADC terlalu kecil. Periksa gantungan beban!");
    showMsg("BEBAN TAK TERBACA", "Periksa beban", "", 2000);
    return;
  }
  float c1 = (float)adjMid / wMid;
  Serial.printf(">> Titik Tengah OK! ADC: %ld | cal1: %.2f\n", (long)rawMidF, c1);

  showMsg("LANGKAH 3/3", "Gantung Beban Max", "Ketik gram di Serial", 1000);

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

  float rawMaxF = 0, spreadMax = 0;
  if (!sampleRaw(rawMaxF, spreadMax)) {
    Serial.println("[GAGAL] HX711 tidak merespons atau beban goyang.");
    showMsg("GAGAL", "HX711 lambat/goyang", "", 1800);
    return;
  }
  long adjMax = labs((long)rawMaxF);
  if (adjMax <= adjMid + 50) {
    Serial.println("[GAGAL] Nilai ADC maksimal tidak bertambah secara wajar.");
    showMsg("GAGAL", "ADC tdk bertambah", "", 2000);
    return;
  }
  float c2 = (float)(adjMax - adjMid) / (wMax - wMid);
  Serial.printf(">> Titik Maksimal OK! ADC: %ld | cal2: %.2f\n", (long)rawMaxF, c2);

  // Hitung polaritas arah
  forceSign = (rawMaxF < 0) ? -1.0f : 1.0f;

  // Simpan data Multi-Segmen
  isMultiCal = true;
  cal1 = c1;
  cal2 = c2;
  midGrams = wMid;
  midRaw = adjMid;
  calFactor = ((float)adjMax / (wMax / 1000.0f)) / G;
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

  showMsg("KALIBRASI MULTI OK", "Tersimpan di Flash", "Alat siap digunakan", 2500);
}

void setBeban(float grams) {
  bebanGram = grams;
  bebanW = (bebanGram / 1000.0f) * G;
  prefs.putFloat("beban", bebanGram);
}

// Timbang beban yang menggantung langsung di load cell -> simpan sebagai W
void weighLoad() {
  if (!isCalibrated) {
    Serial.println("Timbang gagal: belum dikalibrasi.");
    showMsg("BELUM KALIBRASI", "Tahan BOOT >5 dtk", "dgn beban 200 g", 2500);
    return;
  }
  if (!hxReady()) return;

  showMsg("Menimbang beban...", "Jaga beban diam");
  delay(800);

  float raw, spread;
  if (!sampleRaw(raw, spread)) {
    showMsg("GAGAL", "HX711 lambat/putus", "", 1800);
    return;
  }
  float newton = rawToNewtons(raw);
  float grams = rawToGrams(raw);
  float spreadN = (spread / 1000.0f) * G / (isMultiCal ? cal1 : calFactor);

  if (spreadN > WEIGH_STABLE_N) {
    showMsg("BEBAN GOYANG", "Tunggu diam lalu", "ulangi timbang", 2000);
    return;
  }
  if (grams < WEIGH_MIN_G) {
    Serial.print("Terbaca hanya "); Serial.print(grams, 0); Serial.println(" g");
    char b2[24];
    snprintf(b2, sizeof(b2), "Terbaca %.0f g", grams);
    showMsg("BEBAN TAK TERBACA", b2, "Tare saat kosong?", 2500);
    return;
  }
  if (grams > WEIGH_MAX_G) {
    showMsg("MELEBIHI 1 KG", "Turunkan beban!", "", 1800);
    return;
  }

  setBeban(grams);
  char b1[24], b2[24];
  snprintf(b1, sizeof(b1), "W = %.0f g", bebanGram);
  snprintf(b2, sizeof(b2), "= %.2f N", bebanW);
  Serial.print("W terbaca: "); Serial.print(bebanGram, 0); Serial.println(" g (tersimpan)");
  showMsg("BEBAN TERSIMPAN", b1, b2, 2200);
}

// Ganti W ke preset berikutnya (untuk kasus beban tidak bisa digantung di load cell)
void nextPreset() {
  int cur = -1;
  for (int i = 0; i < N_PRESETS; i++) if (fabs(PRESETS[i] - bebanGram) < 1.0f) cur = i;
  int nxt = (cur + 1) % N_PRESETS;
  setBeban(PRESETS[nxt]);
  char b1[24], b2[24];
  snprintf(b1, sizeof(b1), "W = %.0f g", bebanGram);
  snprintf(b2, sizeof(b2), "= %.2f N", bebanW);
  Serial.print("W preset: "); Serial.print(bebanGram, 0); Serial.println(" g");
  showMsg("W DIGANTI (PRESET)", b1, b2, 1500);
}

void printDiagnostics() {
  if (!hxReady()) return;
  float rawVal = scale.get_value(5);
  Serial.println("--- DIAGNOSTIK SENSOR HX711 ---");
  Serial.print("Nilai RAW ADC saat ini : "); Serial.println(rawVal, 0);
  if (isMultiCal) {
    Serial.println("Mode Kalibrasi         : 2-SEGMEN (MULTI-POINT)");
    Serial.printf("Segmen 1 (0 -> %.0fg)   : cal1 = %.2f\n", midGrams, cal1);
    Serial.printf("Segmen 2 (%.0fg -> MAX): cal2 = %.2f\n", midGrams, cal2);
  } else {
    Serial.println("Mode Kalibrasi         : 1-TITIK STANDAR");
    Serial.print("Faktor Kalibrasi       : "); Serial.println(calFactor, 2);
  }
  Serial.print("Status Kalibrasi       : "); Serial.println(isCalibrated ? "TERKALIBRASI (OK)" : "BELUM (!CAL)");
  Serial.print("Arah Tanda (Sign)      : "); Serial.println(forceSign > 0 ? "+1.0 (Normal)" : "-1.0 (Inverted)");
  Serial.print("Pembacaan Gaya (N)     : "); Serial.println(force, 2);
  Serial.print("Beban W                : "); Serial.print(bebanGram, 0); Serial.println(" g");
  Serial.println("-------------------------------");
}

void handleSerial() {
  if (!Serial.available()) return;
  String s = Serial.readStringUntil('\n');
  s.trim();
  s.toLowerCase();
  if (s.length() == 0) return;
  char c = s.charAt(0);
  String arg = s.substring(1);

  if (c == 't') tareForce();
  else if (c == 'z') zeroDistance();
  else if (c == 'r') { forceMax = 0; Serial.println("Maks direset."); }
  else if (c == 'i') {
    forceSign = -forceSign;
    prefs.putFloat("sign", forceSign);
    Serial.println(forceSign > 0 ? "Arah: Normal (+)" : "Arah: Dibalik (-)");
  }
  else if (c == 'd') printDiagnostics();
  else if (s == "m" || s == "mcal") calibrateMulti();
  else if (s == "w" || s == "weigh") weighLoad();
  else if (c == 'c') {
    if (arg == "0") resetCalibration();
    else if (arg.toFloat() > 0) calibrate(arg.toFloat());
    else Serial.println("Format salah. Contoh: c240 atau c0");
  }
  else if (c == 'b') {
    float bg = arg.toFloat();
    if (bg > 0) {
      setBeban(bg);
      Serial.print("Beban W diatur: "); Serial.print(bebanGram, 1); Serial.println(" g");
    } else {
      Serial.println("Massa beban tidak valid. Contoh: b240");
    }
  }
  else Serial.println("Perintah: t (tare) | z (zero dist) | m (multi-cal) | w (timbang) | i (invert) | d (diag) | c<g> | b<g>");
}

// Layar saat tombol ditahan: tunjukkan aksi yang terjadi jika dilepas sekarang
void drawHold(unsigned long held) {
  if (!oledOk) return;
  const char *act;
  if (held < T_WEIGH)       act = "TARE";
  else if (held < T_PRESET) act = "TIMBANG W";
  else if (held < T_CAL)    act = "PILIH W";
  else                      act = "KALIBRASI";

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);  display.print("Tombol ditahan");
  display.setCursor(0, 12); display.print("Lepas sekarang =");
  display.setTextSize(2);
  display.setCursor(0, 26); display.print(act);
  display.setTextSize(1);
  display.setCursor(0, 56); display.print(held / 1000.0f, 1); display.print(" dtk");
  unsigned long h = held > T_CAL ? T_CAL : held;
  int w = (int)((h * 128UL) / T_CAL);
  display.fillRect(0, 48, w, 5, SSD1306_WHITE);
  display.display();
}

void draw() {
  if (!oledOk) return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // === ZONA ATAS: F (Kuasa) & W (Beban) ===
  display.setTextSize(1);
  display.setCursor(2, 0);
  display.print("F(Kuasa)");
  display.setCursor(68, 0);
  display.print("W(Beban)");

  display.setCursor(2, 10);
  if (!isCalibrated) {
    display.print("!CAL");
  } else if (fabs(force) > 9.99f) {
    display.print("OVLD");
  } else {
    display.print(force, 2);
    display.print("N");
  }

  display.setCursor(68, 10);
  display.print(bebanW, 2);
  display.print("N");

  display.drawFastHLine(0, 20, 128, SSD1306_WHITE);

  // === ZONA TENGAH: KM = W / F ===
  display.setTextSize(1);
  display.setCursor(34, 23);
  display.print("KM = W / F");

  display.setTextSize(2);
  display.setCursor(40, 33);
  if (!isCalibrated || force < 0.30f) {
    display.print("---");
  } else if (km >= 10.0f) {
    display.print(">10");
  } else {
    display.print(km, 2);
  }

  display.drawFastHLine(0, 50, 128, SSD1306_WHITE);

  // === ZONA BAWAH: d, s, U ===
  display.setTextSize(1);
  display.setCursor(0, 54);
  display.print("d:");
  if (distValid) { display.print(distCm, 0); display.print("cm"); }
  else display.print("---");

  display.setCursor(43, 54);
  display.print("s:");
  if (distValid) { display.print(fabs(distCm - distRef), 0); display.print("cm"); }
  else display.print("---");

  display.setCursor(86, 54);
  display.print("U:");
  if (isCalibrated) {
    if (usaha > 99.9f) display.print(">99J");
    else { display.print(usaha, 1); display.print("J"); }
  } else {
    display.print("---");
  }

  display.display();
}

// Setup dan Handler ArduinoOTA
void setupOTA() {
  ArduinoOTA.setHostname("katrokit-esp32");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {
      type = "filesystem";
    }
    Serial.println("\n[OTA] Mulai update firmware: " + type);
    if (oledOk) {
      display.clearDisplay();
      display.setTextColor(SSD1306_WHITE);
      display.setTextSize(1);
      display.setCursor(0, 15);
      display.println("OTA UPDATE...");
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
    Serial.printf("[OTA] Terjadi Error [%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Gagal");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Gagal");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Gagal");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Gagal");
    else if (error == OTA_END_ERROR) Serial.println("End Gagal");
  });

  ArduinoOTA.begin();
  Serial.println("[OTA] ArduinoOTA siap. Hostname: katrokit-esp32.local");
}

void initWiFiAndOTA() {
  WiFiManager wm;
  wm.setConfigPortalTimeout(60); // Timeout 60 detik agar tidak freeze saat offline
  wm.setConnectTimeout(10);

  if (oledOk) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 5);
    display.println("Menghubungkan WiFi...");
    display.setCursor(0, 22);
    display.println("Jika belum connect:");
    display.setCursor(0, 36);
    display.println("Buka WiFi HP / Laptop");
    display.setCursor(0, 50);
    display.println("AP: KatroKit-AP");
    display.display();
  }

  Serial.println("[WIFI] Memulai WiFiManager...");
  bool connected = wm.autoConnect("KatroKit-AP");

  if (connected) {
    Serial.println("[WIFI] Berhasil terhubung ke jaringan!");
    Serial.print("[WIFI] IP Address: ");
    Serial.println(WiFi.localIP());

    setupOTA();

    if (oledOk) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0, 5);
      display.println("WiFi Terhubung!");
      display.setCursor(0, 20);
      display.println(WiFi.SSID());
      display.setCursor(0, 36);
      display.println("IP:");
      display.setCursor(0, 48);
      display.println(WiFi.localIP().toString());
      display.display();
      delay(2000);
    }
  } else {
    Serial.println("[WIFI] Timeout! Melanjutkan dalam mode Offline.");
    if (oledOk) {
      showMsg("WIFI OFFLINE", "Mode offline aktif", "KatroKit Siap!", 1500);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(BTN, INPUT_PULLUP);

  Wire.begin(SDA_PIN, SCL_PIN);
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (!oledOk) {
    Serial.println("OLED tidak terdeteksi! Cek kabel SDA/SCL dan alamat I2C (0x3C/0x3D).");
  }

  // Inisialisasi WiFi & ArduinoOTA
  initWiFiAndOTA();

  if (oledOk) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(10, 10);
    display.println("KatroKit");
    display.setTextSize(1);
    display.setCursor(10, 40);
    display.println("Jangan sentuh tali...");
    display.display();
  }

  prefs.begin("katrokit", false);
  calFactor = prefs.getFloat("cal", 1.0f);
  forceSign = prefs.getFloat("sign", -1.0f);
  bebanGram = prefs.getFloat("beban", 240.0f);
  bebanW = (bebanGram / 1000.0f) * G;

  isMultiCal = prefs.getBool("is_multi", false);
  if (isMultiCal) {
    cal1 = prefs.getFloat("cal1", calFactor);
    cal2 = prefs.getFloat("cal2", calFactor);
    midGrams = prefs.getFloat("mid_w", 100.0f);
    midRaw = prefs.getLong("mid_raw", 0);
    isCalibrated = true;
    Serial.printf("[INFO] Mode Kalibrasi: 2-SEGMEN (0 -> %.0fg -> MAX)\n", midGrams);
  } else {
    isCalibrated = (calFactor != 1.0f);
    Serial.printf("[INFO] Mode Kalibrasi: 1-TITIK STANDAR (calFactor = %.2f)\n", calFactor);
  }

  scale.begin(HX_DT, HX_SCK);
  delay(1500);
  if (scale.wait_ready_timeout(2000)) {
    scale.tare(15);
  } else {
    Serial.println("HX711 tidak merespons! Cek DT/SCK, VCC 3.3V dan GND.");
  }

  if (!isCalibrated) {
    showMsg("BELUM KALIBRASI", "Ketik m di Serial", "utk kalibrasi", 3000);
  }

  for (int i = 0; i < 5; i++) {
    float d = readDistanceCm();
    if (d > 2.0f && d < 250.0f) { distCm = d; distValid = true; }
    delay(60);
  }
  if (distValid) distRef = distCm;
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.handle();
  }

  handleSerial();

  // Gaya
  if (scale.is_ready()) {
    lastRaw = scale.get_value(1);
    float fInstant = rawToNewtons((float)lastRaw);
    fBuf[fIdx] = fInstant;
    fIdx++;
    if (fIdx >= F_N) { fIdx = 0; fFull = true; }
    force = avgBuf(fBuf, F_N, fFull, fIdx);
    if (fabs(force) < 0.05f) force = 0.0f;
    if (force > forceMax) forceMax = force;

    float disp_m = fabs(distCm - distRef) / 100.0f;
    usaha = (force > 0 ? force : 0) * disp_m;

    if (isCalibrated && force >= 0.30f) {
      km = bebanW / force;
      if (km > 10.0f) km = 10.0f;
    } else {
      km = 0.0f;
    }
  }

  // Jarak tiap 100 ms
  if (millis() - lastDist >= 100) {
    lastDist = millis();
    float d = readDistanceCm();
    if (d > 2.0f && d < 250.0f) {
      dBuf[dIdx] = d;
      dIdx++;
      if (dIdx >= D_N) { dIdx = 0; dFull = true; }
      distCm = avgBuf(dBuf, D_N, dFull, dIdx);
      distValid = true;
    }
  }

  // Tombol: aksi ditentukan lama tahan, dijalankan saat DILEPAS
  bool pressed = (digitalRead(BTN) == LOW);
  if (pressed && !btnDown) {
    btnDown = true;
    btnDownAt = millis();
  } else if (!pressed && btnDown) {
    btnDown = false;
    unsigned long held = millis() - btnDownAt;
    if (held >= 50) {                      // abaikan bouncing
      if (held < T_WEIGH) {
        tareForce();
        zeroDistance();
        showMsg("TARE OK", "Gaya & jarak = 0", "", 700);
      }
      else if (held < T_PRESET) weighLoad();
      else if (held < T_CAL)    nextPreset();
      else                      calibrate(CAL_REF_G);
    }
    lastDraw = millis();
  }

  // OLED + Serial Plotter tiap 100 ms
  if (millis() - lastDraw >= 100) {
    lastDraw = millis();
    if (btnDown) drawHold(millis() - btnDownAt);
    else draw();
    Serial.print("Gaya_N:");
    Serial.print(force, 2);
    Serial.print(",RAW:");
    Serial.print(lastRaw);
    Serial.print(",KM:");
    Serial.print(km, 2);
    Serial.print(",Jarak_cm:");
    Serial.print(distCm, 1);
    Serial.print(",Pindah_cm:");
    Serial.println(distCm - distRef, 1);
  }
}
