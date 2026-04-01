# CubeSat Stratosferik Balon — Yazılım Paketi
## Teensy 4.0 Tabanlı (Ege Üst PCB + Akif Alt PCB)

---

## Dosyalar

| Dosya | Açıklama |
|---|---|
| `main.ino` | Teensy 4.0 ana kod (Arduino IDE / Teensyduino) |
| `yer_istasyonu.py` | PC'de çalışan canlı grafik + rapor üretici |
| `requirements.txt` | Python bağımlılıkları |

---

## Teensy 4.0 — Kütüphane Kurulumu

Arduino IDE'de Library Manager'dan kur:

```
Adafruit BMP280 Library
Adafruit MPU6050
Adafruit SHT31 Library
SparkFun SCD4x Arduino Library
TinyGPSPlus
SD (Arduino dahili)
```

Teensyduino kurulu olmalı: https://www.pjrc.com/teensy/teensyduino.html

---

## Python — Kurulum

```bash
pip install matplotlib numpy pyserial
```

---

## Kullanım

### 1. Canlı İzleme (SD karttan kopyalanan CSV)
```bash
python yer_istasyonu.py --csv LOG_001.CSV
```

### 2. Uçuş Raporu PNG Üret
```bash
python yer_istasyonu.py --csv LOG_001.CSV --report
```
→ LOG_001.PNG olarak kaydeder, 12 grafikli özet sayfa.

### 3. Seri Port (LoRa yer istasyonu - geliştirme için)
```bash
python yer_istasyonu.py --port COM3
# Linux: --port /dev/ttyUSB0
```

---

## CSV Kolon Sırası

```
timestamp_ms, gps_lat, gps_lon, gps_alt_m, gps_sats, gps_valid,
pressure_hPa, bmp_temp_C, baro_alt_m,
accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z,
humidity_pct, sht_temp_C,
co2_ppm, scd_temp_C, scd_humidity,
mq7_raw, mq7_co_ppm, mq135_raw, mq135_aqi, mq131_raw, mq131_o3_ppb,
ntc1_temp_C, ntc2_temp_C,
heater1_duty, heater2_duty, heater3_duty
```

---

## Pin Özeti

### Üst PCB (Ege)
| Bileşen | Bağlantı |
|---|---|
| BMP280 | I2C (Wire) — 0x76 |
| MPU6050 | I2C (Wire) — 0x68 |
| NEO-M8N GPS | Serial2 (RX=7, TX=8) |
| E22-900T22D LoRa | Serial1 + M0=2, M1=3, AUX=4 |
| SD Kart | SPI (CS=10) |

### Alt PCB (Akif)
| Bileşen | Bağlantı |
|---|---|
| SCD-40 CO2 | I2C (Wire) — 0x62 |
| SHT-30 Nem | I2C (Wire) — 0x44 |
| MQ-7 CO | A0 (Analog) |
| MQ-135 AQI | A1 (Analog) |
| MQ-131 Ozon | A2 (Analog) |
| NTC1 (Batarya) | A3 (Analog) |
| NTC2 (Kart) | A4 (Analog) |
| IRF540N x3 (Isıtıcı) | PWM: 5, 6, 7 |

---

## Notlar

- MQ sensörler 5V besleme alır, sinyal pini 3.3V'a voltaj bölücü ile düşürülmeli
- IRF540N için gate'e 100Ω seri direnç + 10kΩ pull-down ekle
- NTC için 10kΩ seri direnç ile 3.3V-GND arası bölücü kur
- SCD40 ilk ölçüm için 5 saniye bekler
- GPS cold start ~30-60 saniye sürer
