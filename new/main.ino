// ============================================================
//  CubeSat Stratosferik Balon — Teensy 4.0 Ana Kod
//  Üst PCB: BMP280, MPU6050, E22-900T22D, NEO-M8N
//  Alt PCB: MQ-7, MQ-135, MQ-131, SCD-40, SHT-30, 2x NTC, 3x IRF540N
//  Görev 1: Veri Toplama + SD Kart CSV Loglama
//  Görev 2: LoRa ile Telemetri Gönderimi
// ============================================================

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_SHT31.h>
#include <SparkFun_SCD4x_Arduino_Library.h>
#include <TinyGPSPlus.h>

// ── PIN TANIMLARI ─────────────────────────────────────────────
// SD Kart (SPI)
#define SD_CS_PIN       10

// E22 LoRa (UART1)
#define LORA_SERIAL     Serial1
#define LORA_M0         2
#define LORA_M1         3
#define LORA_AUX        4

// GPS (UART2)
#define GPS_SERIAL      Serial2

// MQ Sensörler (Analog)
#define MQ7_PIN         A0   // CO
#define MQ135_PIN       A1   // Hava kalitesi
#define MQ131_PIN       A2   // Ozon

// NTC Termistörler (Analog)
#define NTC1_PIN        A3   // Batarya sıcaklığı
#define NTC2_PIN        A4   // Kart sıcaklığı

// IRF540N MOSFET — Isıtıcı PWM
#define HEATER1_PIN     5    // Batarya ısıtıcısı
#define HEATER2_PIN     6    // Sensör bloğu
#define HEATER3_PIN     7    // Genel gövde

// ── SABİTLER ──────────────────────────────────────────────────
#define NTC_NOMINAL     10000.0   // 25°C'de direnç (10k)
#define NTC_SERIES_R    10000.0   // Seri direnç (10k)
#define NTC_BCOEFF      3950.0    // B katsayısı
#define NTC_NOMINAL_T   298.15    // 25°C (Kelvin)
#define ADC_MAX         1023.0
#define VREF            3.3

#define LOG_INTERVAL_MS    1000   // 1 saniyede bir veri topla
#define LORA_INTERVAL_MS   5000   // 5 saniyede bir LoRa gönder
#define HEATER_INTERVAL_MS 2000   // 2 saniyede bir termal kontrol

// Hedef sıcaklık eşikleri (°C)
#define HEATER_ON_BELOW    -10.0
#define HEATER_OFF_ABOVE     5.0

// ── VERİ YAPISI ───────────────────────────────────────────────
struct SensorData {
  // GPS
  float    gps_lat;
  float    gps_lon;
  float    gps_alt;
  uint8_t  gps_sats;
  bool     gps_valid;

  // BMP280
  float    pressure;     // hPa
  float    bmp_temp;     // °C
  float    altitude_baro;// m

  // MPU6050
  float    accel_x, accel_y, accel_z;  // m/s²
  float    gyro_x, gyro_y, gyro_z;     // °/s

  // SHT30
  float    humidity;     // %RH
  float    sht_temp;     // °C

  // SCD40
  uint16_t co2_ppm;
  float    scd_temp;
  float    scd_humidity;

  // MQ Sensörler (0–1023 ham ADC)
  int      mq7_raw;      // CO
  int      mq135_raw;    // Hava kalitesi
  int      mq131_raw;    // Ozon

  // NTC Sıcaklıklar
  float    ntc1_temp;    // Batarya
  float    ntc2_temp;    // Kart

  // Isıtıcı Duty Cycle (0–255)
  uint8_t  heater1_duty;
  uint8_t  heater2_duty;
  uint8_t  heater3_duty;

  // Zaman
  unsigned long timestamp_ms;
};

// ── GLOBAL NESNELER ───────────────────────────────────────────
Adafruit_BMP280    bmp;
Adafruit_MPU6050   mpu;
Adafruit_SHT31     sht30;
SCD4x              scd40;
TinyGPSPlus        gps;
SensorData         data;
File               logFile;

unsigned long lastLogTime   = 0;
unsigned long lastLoraTime  = 0;
unsigned long lastHeatTime  = 0;
uint32_t      logLineCount  = 0;
bool          sdReady       = false;

// ── YARDIMCI: NTC → Sıcaklık (Steinhart-Hart) ─────────────────
float ntcToTemp(int adcVal) {
  float voltage  = (adcVal / ADC_MAX) * VREF;
  if (voltage <= 0.0 || voltage >= VREF) return -273.0;
  float resistance = NTC_SERIES_R * voltage / (VREF - voltage);
  float steinhart  = resistance / NTC_NOMINAL;
  steinhart = log(steinhart);
  steinhart /= NTC_BCOEFF;
  steinhart += 1.0 / NTC_NOMINAL_T;
  return (1.0 / steinhart) - 273.15;
}

// ── YARDIMCI: MQ Ham → ppm (Yaklaşık, kalibrasyona göre ayarla) ─
float mq7ToCO_ppm(int raw) {
  // Rs/R0 oranına göre yaklaşık CO ppm (datasheet curve)
  float voltage = (raw / ADC_MAX) * VREF;
  float rs = (VREF - voltage) / voltage * 10000.0;
  return 100.0 * pow((rs / 27.0), -1.513); // R0=27k için
}

float mq135ToAQI(int raw) {
  float voltage = (raw / ADC_MAX) * VREF;
  return map(raw, 0, 1023, 0, 500); // Ham 0-500 AQI yaklaşımı
}

float mq131ToO3_ppb(int raw) {
  float voltage = (raw / ADC_MAX) * VREF;
  float rs = (VREF - voltage) / voltage * 10000.0;
  return 10.0 * pow((rs / 15.0), -1.67); // R0=15k için yaklaşık
}

// ── YARDIMCI: Termal Kontrol ───────────────────────────────────
void updateHeaters() {
  // NTC1 → Heater1 (batarya)
  float t1 = data.ntc1_temp;
  if (t1 < HEATER_ON_BELOW) {
    data.heater1_duty = 200;
  } else if (t1 > HEATER_OFF_ABOVE) {
    data.heater1_duty = 0;
  } else {
    // Orantılı kontrol
    float ratio = (HEATER_OFF_ABOVE - t1) / (HEATER_OFF_ABOVE - HEATER_ON_BELOW);
    data.heater1_duty = (uint8_t)(ratio * 200);
  }

  // NTC2 → Heater2 (sensör bloğu)
  float t2 = data.ntc2_temp;
  if (t2 < HEATER_ON_BELOW) {
    data.heater2_duty = 180;
  } else if (t2 > HEATER_OFF_ABOVE) {
    data.heater2_duty = 0;
  } else {
    float ratio = (HEATER_OFF_ABOVE - t2) / (HEATER_OFF_ABOVE - HEATER_ON_BELOW);
    data.heater2_duty = (uint8_t)(ratio * 180);
  }

  // Heater3 → BMP280 sıcaklığını baz al (genel gövde)
  float t3 = data.bmp_temp;
  if (t3 < HEATER_ON_BELOW) {
    data.heater3_duty = 150;
  } else if (t3 > HEATER_OFF_ABOVE) {
    data.heater3_duty = 0;
  } else {
    float ratio = (HEATER_OFF_ABOVE - t3) / (HEATER_OFF_ABOVE - HEATER_ON_BELOW);
    data.heater3_duty = (uint8_t)(ratio * 150);
  }

  analogWrite(HEATER1_PIN, data.heater1_duty);
  analogWrite(HEATER2_PIN, data.heater2_duty);
  analogWrite(HEATER3_PIN, data.heater3_duty);
}

// ── SD KART: Başlık yaz ───────────────────────────────────────
void writeCSVHeader() {
  logFile.println(
    "timestamp_ms,gps_lat,gps_lon,gps_alt_m,gps_sats,gps_valid,"
    "pressure_hPa,bmp_temp_C,baro_alt_m,"
    "accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,"
    "humidity_pct,sht_temp_C,"
    "co2_ppm,scd_temp_C,scd_humidity,"
    "mq7_raw,mq7_co_ppm,mq135_raw,mq135_aqi,mq131_raw,mq131_o3_ppb,"
    "ntc1_temp_C,ntc2_temp_C,"
    "heater1_duty,heater2_duty,heater3_duty"
  );
  logFile.flush();
}

// ── SD KART: Satır yaz ────────────────────────────────────────
void writeCSVRow() {
  char buf[512];
  snprintf(buf, sizeof(buf),
    "%lu,%.6f,%.6f,%.1f,%d,%d,"
    "%.2f,%.2f,%.1f,"
    "%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,"
    "%.1f,%.2f,"
    "%d,%.2f,%.1f,"
    "%d,%.2f,%d,%.0f,%d,%.3f,"
    "%.2f,%.2f,"
    "%d,%d,%d",
    data.timestamp_ms,
    data.gps_lat, data.gps_lon, data.gps_alt, data.gps_sats, (int)data.gps_valid,
    data.pressure, data.bmp_temp, data.altitude_baro,
    data.accel_x, data.accel_y, data.accel_z,
    data.gyro_x, data.gyro_y, data.gyro_z,
    data.humidity, data.sht_temp,
    data.co2_ppm, data.scd_temp, data.scd_humidity,
    data.mq7_raw, mq7ToCO_ppm(data.mq7_raw),
    data.mq135_raw, mq135ToAQI(data.mq135_raw),
    data.mq131_raw, mq131ToO3_ppb(data.mq131_raw),
    data.ntc1_temp, data.ntc2_temp,
    data.heater1_duty, data.heater2_duty, data.heater3_duty
  );
  logFile.println(buf);
  logFile.flush();
  logLineCount++;
}

// ── LoRa: Binary Paket Gönder ────────────────────────────────
void sendLoraPacket() {
  // AUX pini LOW ise meşgul — bekle
  unsigned long t = millis();
  while (digitalRead(LORA_AUX) == LOW && millis() - t < 1000);

  uint8_t pkt[48];
  uint8_t i = 0;

  // Timestamp (4B)
  uint32_t ts = data.timestamp_ms;
  memcpy(pkt + i, &ts, 4); i += 4;

  // GPS (12B)
  memcpy(pkt + i, &data.gps_lat, 4); i += 4;
  memcpy(pkt + i, &data.gps_lon, 4); i += 4;
  memcpy(pkt + i, &data.gps_alt, 4); i += 4;

  // Basınç + sıcaklık (4B)
  int16_t p = (int16_t)(data.pressure * 10);
  int16_t bt = (int16_t)(data.bmp_temp * 100);
  memcpy(pkt + i, &p, 2); i += 2;
  memcpy(pkt + i, &bt, 2); i += 2;

  // Nem + CO2 (4B)
  int16_t hum = (int16_t)(data.humidity * 100);
  memcpy(pkt + i, &hum, 2); i += 2;
  memcpy(pkt + i, &data.co2_ppm, 2); i += 2;

  // MQ sensörler (3B)
  pkt[i++] = map(data.mq7_raw, 0, 1023, 0, 255);
  pkt[i++] = map(data.mq135_raw, 0, 1023, 0, 255);
  pkt[i++] = map(data.mq131_raw, 0, 1023, 0, 255);

  // NTC (4B)
  int16_t n1 = (int16_t)(data.ntc1_temp * 100);
  int16_t n2 = (int16_t)(data.ntc2_temp * 100);
  memcpy(pkt + i, &n1, 2); i += 2;
  memcpy(pkt + i, &n2, 2); i += 2;

  // Isıtıcı duty (3B)
  pkt[i++] = data.heater1_duty;
  pkt[i++] = data.heater2_duty;
  pkt[i++] = data.heater3_duty;

  // IMU ivme (6B)
  int16_t ax = (int16_t)(data.accel_x * 100);
  int16_t ay = (int16_t)(data.accel_y * 100);
  int16_t az = (int16_t)(data.accel_z * 100);
  memcpy(pkt + i, &ax, 2); i += 2;
  memcpy(pkt + i, &ay, 2); i += 2;
  memcpy(pkt + i, &az, 2); i += 2;

  // CRC16 basit (2B)
  uint16_t crc = 0;
  for (uint8_t j = 0; j < i; j++) crc ^= pkt[j];
  memcpy(pkt + i, &crc, 2); i += 2;

  LORA_SERIAL.write(pkt, i);
}

// ── SETUP ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("[BOOT] CubeSat Teensy 4.0 baslatiliyor...");

  // Isıtıcı pinleri
  pinMode(HEATER1_PIN, OUTPUT);
  pinMode(HEATER2_PIN, OUTPUT);
  pinMode(HEATER3_PIN, OUTPUT);
  analogWrite(HEATER1_PIN, 0);
  analogWrite(HEATER2_PIN, 0);
  analogWrite(HEATER3_PIN, 0);

  // LoRa pinleri
  pinMode(LORA_M0, OUTPUT);
  pinMode(LORA_M1, OUTPUT);
  pinMode(LORA_AUX, INPUT);
  digitalWrite(LORA_M0, LOW);   // Normal mod
  digitalWrite(LORA_M1, LOW);
  LORA_SERIAL.begin(9600);
  Serial.println("[OK] LoRa E22 hazir");

  // GPS
  GPS_SERIAL.begin(9600);
  Serial.println("[OK] GPS UART baslatildi");

  // I2C
  Wire.begin();

  // BMP280
  if (!bmp.begin(0x76)) {
    Serial.println("[HATA] BMP280 bulunamadi!");
  } else {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
    Serial.println("[OK] BMP280 hazir");
  }

  // MPU6050
  if (!mpu.begin()) {
    Serial.println("[HATA] MPU6050 bulunamadi!");
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("[OK] MPU6050 hazir");
  }

  // SHT30
  if (!sht30.begin(0x44)) {
    Serial.println("[HATA] SHT30 bulunamadi!");
  } else {
    Serial.println("[OK] SHT30 hazir");
  }

  // SCD40
  scd40.begin(Wire);
  if (scd40.startPeriodicMeasurement() != 0) {
    Serial.println("[HATA] SCD40 baslatma hatasi!");
  } else {
    Serial.println("[OK] SCD40 periyodik olcum baslatildi");
  }

  // SD Kart
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("[HATA] SD kart bulunamadi!");
    sdReady = false;
  } else {
    // Dosya adı: LOG_001.CSV, LOG_002.CSV vb.
    char fname[16];
    for (int n = 1; n <= 999; n++) {
      sprintf(fname, "LOG_%03d.CSV", n);
      if (!SD.exists(fname)) break;
    }
    logFile = SD.open(fname, FILE_WRITE);
    if (logFile) {
      writeCSVHeader();
      sdReady = true;
      Serial.print("[OK] SD kart hazir, dosya: ");
      Serial.println(fname);
    }
  }

  Serial.println("[BOOT] Baslatma tamamlandi. Veri toplama basladi.");
}

// ── LOOP ──────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  // GPS karakterlerini sürekli oku
  while (GPS_SERIAL.available()) {
    gps.encode(GPS_SERIAL.read());
  }

  // ── GÖREV 1: VERİ TOPLAMA + CSV LOGLAMA ──────────────────
  if (now - lastLogTime >= LOG_INTERVAL_MS) {
    lastLogTime = now;
    data.timestamp_ms = now;

    // GPS
    if (gps.location.isValid()) {
      data.gps_lat   = gps.location.lat();
      data.gps_lon   = gps.location.lng();
      data.gps_valid = true;
    } else {
      data.gps_lat   = 0.0;
      data.gps_lon   = 0.0;
      data.gps_valid = false;
    }
    data.gps_alt  = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
    data.gps_sats = gps.satellites.isValid() ? gps.satellites.value() : 0;

    // BMP280
    data.pressure      = bmp.readPressure() / 100.0;
    data.bmp_temp      = bmp.readTemperature();
    data.altitude_baro = bmp.readAltitude(1013.25);

    // MPU6050
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    data.accel_x = a.acceleration.x;
    data.accel_y = a.acceleration.y;
    data.accel_z = a.acceleration.z;
    data.gyro_x  = g.gyro.x;
    data.gyro_y  = g.gyro.y;
    data.gyro_z  = g.gyro.z;

    // SHT30
    data.humidity = sht30.readHumidity();
    data.sht_temp = sht30.readTemperature();

    // SCD40 (her 5s yeni veri hazır)
    uint16_t co2; float scdT, scdH;
    if (scd40.readMeasurement(co2, scdT, scdH) == 0) {
      data.co2_ppm     = co2;
      data.scd_temp    = scdT;
      data.scd_humidity = scdH;
    }

    // MQ Sensörler (Analog)
    data.mq7_raw   = analogRead(MQ7_PIN);
    data.mq135_raw = analogRead(MQ135_PIN);
    data.mq131_raw = analogRead(MQ131_PIN);

    // NTC
    data.ntc1_temp = ntcToTemp(analogRead(NTC1_PIN));
    data.ntc2_temp = ntcToTemp(analogRead(NTC2_PIN));

    // SD Kart yaz
    if (sdReady) {
      writeCSVRow();
    }

    // Seri port debug (her 5 satırda bir)
    if (logLineCount % 5 == 0) {
      Serial.printf("[%lu] GPS:%.4f,%.4f Alt:%.1fm BMP:%.1fhPa/%.1fC CO2:%dppm Hum:%.1f%% NTC1:%.1fC\n",
        now, data.gps_lat, data.gps_lon, data.gps_alt,
        data.pressure, data.bmp_temp, data.co2_ppm,
        data.humidity, data.ntc1_temp);
    }
  }

  // ── GÖREV 2: LoRa TELEMETRİ GÖNDERİMİ ───────────────────
  if (now - lastLoraTime >= LORA_INTERVAL_MS) {
    lastLoraTime = now;
    sendLoraPacket();
    Serial.println("[LoRa] Paket gonderildi");
  }

  // ── TERMAL KONTROL ────────────────────────────────────────
  if (now - lastHeatTime >= HEATER_INTERVAL_MS) {
    lastHeatTime = now;
    updateHeaters();
  }
}
