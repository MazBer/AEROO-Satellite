"""
╔══════════════════════════════════════════════════════════════════╗
║   STRATOSPHERIC CUBESAT — KISIM 1: ONBOARD FLIGHT SYSTEM        ║
║   Hedef Donanım : Adafruit Feather M0 + LoRa RFM9x              ║
║   Dil           : CircuitPython 8.x                              ║
║                                                                  ║
║   Bu script şunları yapar:                                       ║
║     • Tüm sensörleri başlatır ve kalibre eder                    ║
║     • Ana döngüde sensörlerden veri okur                         ║
║     • Hareketli ortalama + aykırı değer filtresi uygular         ║
║     • Sıcaklık kompanzasyonu yapar (gaz sensörleri)              ║
║     • Yüksekliği barometrik formülle hesaplar                    ║
║     • Termal kontrol (ON/OFF ısıtıcı yönetimi)                   ║
║     • Güç yönetimi ve watchdog koruması                          ║
║     • SD karta CSV formatında veri yazar                         ║
║     • LoRa ile paketi yer istasyonuna iletir (TX)                ║
║     • Yer istasyonundan komut/ACK alır (RX)                      ║
╚══════════════════════════════════════════════════════════════════╝
"""

# ──────────────────────────────────────────────────────────────────
# KÜTÜPHANE İMPORTLARI
# ──────────────────────────────────────────────────────────────────
import time
import math
import struct
import busio
import board
import analogio
import digitalio
import sdcardio
import storage
import microcontroller

# Sensör sürücüleri  (lib/ klasörüne kopyalanmış olmalı)
import adafruit_bmp280           # Basınç + Sıcaklık  (I2C)
import adafruit_scd30            # CO2 + RH           (I2C)
import adafruit_pm25             # PM2.5 toz          (UART)
import adafruit_rfm9x            # LoRa RFM9x         (SPI)

# ──────────────────────────────────────────────────────────────────
# 1. DONANIM SABİTLERİ VE KONFİGÜRASYON
# ──────────────────────────────────────────────────────────────────

# --- LoRa RF parametreleri ---
LORA_FREQ           = 868.0          # MHz  (EU bandı)
LORA_TX_POWER       = 13             # dBm  (5–23 arası)
LORA_SPREADING      = 10             # SF10 — uzun menzil
LORA_BANDWIDTH      = 125000         # 125 kHz
LORA_CODING_RATE    = 5              # 4/5
TX_INTERVAL_SEC     = 30             # Her 30 saniyede bir gönder

# --- Örnekleme ---
SAMPLE_INTERVAL_SEC = 10             # SD yazma aralığı (saniye)
FILTER_WINDOW       = 10             # Hareketli ortalama penceresi
OUTLIER_SIGMA       = 3.0            # Aykırı değer eşiği (σ)

# --- Termal kontrol eşikleri ---
HEATER_ON_TEMP      = 0.0            # °C — bu değerin altında ısıtıcı aç
HEATER_OFF_TEMP     = 5.0            # °C — bu değerin üstünde ısıtıcı kapat
HEATER_MAX_WATTS    = 7.0            # W   (sistem tasarımına göre)

# --- Güç / safe-mode eşikleri ---
VBAT_MIN            = 3.3            # V   — bu altında safe-mode
ADC_RESOLUTION      = 65536          # 16-bit ADC
VREF                = 3.3            # Referans gerilimi
VDIV_RATIO          = 2.0            # Voltaj bölücü (örn. 100k/100k)

# --- Kalibrasyon sabitleri (CO2 / CH4 power-law modeli) ---
CO2_A, CO2_B        = 400.0, 1.2     # C = a * (Vout/Vref)^b
CH4_A, CH4_B        = 1.8,   1.5
TEMP_ALPHA          = 0.02           # Sıcaklık kompanzasyon katsayısı
TEMP_REF            = 25.0           # Kalibrasyon referans sıcaklığı (°C)
CO2_OFFSET          = 0.0            # Temiz hava kalibrasyonundan belirlenir
CH4_OFFSET          = 0.0

# --- Barometrik sabit ---
P0                  = 1013.25        # Deniz seviyesi referans basıncı (hPa)

# --- SD dosya adı ---
LOG_FILE            = "/sd/flight_log.csv"
CSV_HEADER          = (
    "time_s,latitude,longitude,pressure_hPa,altitude_m,"
    "temp_internal_C,temp_external_C,co2_ppm,ch4_ppm,"
    "pm25_ugm3,uv_index,humidity_pct,battery_V,heater\n"
)

# ──────────────────────────────────────────────────────────────────
# 2. PIN TANIMLARI
# ──────────────────────────────────────────────────────────────────

# SPI paylaşımlı veri yolu
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

# LoRa CS ve reset pinleri
lora_cs    = digitalio.DigitalInOut(board.D5)
lora_reset = digitalio.DigitalInOut(board.D6)

# SD kart CS
sd_cs      = digitalio.DigitalInOut(board.D10)

# Isıtıcı kontrol pini (MOSFET gate)
heater_pin = digitalio.DigitalInOut(board.D11)
heater_pin.direction = digitalio.Direction.OUTPUT
heater_pin.value = False

# Batarya ADC pini
bat_adc    = analogio.AnalogIn(board.A5)

# Watchdog reset pini (harici WDT varsa)
# wdt_pin  = digitalio.DigitalInOut(board.D12)

# I2C veri yolu
i2c = busio.I2C(board.SCL, board.SDA)

# UART (PM2.5 sensörü için)
uart = busio.UART(board.TX, board.RX, baudrate=9600)

# ──────────────────────────────────────────────────────────────────
# 3. YARDIMCI SINIFLAR
# ──────────────────────────────────────────────────────────────────

class MovingAverageFilter:
    """
    Hareketli ortalama filtresi.
    y[n] = (1/N) * Σ x[n-k]  (k=0..N-1)
    """
    def __init__(self, window=FILTER_WINDOW):
        self.window = window
        self._buf   = []

    def update(self, value):
        if value is None:
            return self.mean()
        self._buf.append(value)
        if len(self._buf) > self.window:
            self._buf.pop(0)
        return self.mean()

    def mean(self):
        if not self._buf:
            return 0.0
        return sum(self._buf) / len(self._buf)

    def std(self):
        if len(self._buf) < 2:
            return 0.0
        m = self.mean()
        variance = sum((x - m) ** 2 for x in self._buf) / len(self._buf)
        return math.sqrt(variance)

    def is_outlier(self, value):
        """3-sigma kuralı ile aykırı değer tespiti."""
        if len(self._buf) < 3:
            return False
        return abs(value - self.mean()) > OUTLIER_SIGMA * self.std()


class ThermalController:
    """
    ON/OFF termal kontrol.
    T < HEATER_ON_TEMP  → ısıtıcı AÇ
    T ≥ HEATER_OFF_TEMP → ısıtıcı KAPAT
    """
    def __init__(self, heater_output_pin):
        self.pin   = heater_output_pin
        self.state = False             # False = kapalı

    def update(self, internal_temp_c):
        if internal_temp_c < HEATER_ON_TEMP:
            self.pin.value = True
            self.state = True
        elif internal_temp_c >= HEATER_OFF_TEMP:
            self.pin.value = False
            self.state = False
        return self.state             # True = açık

    def force_off(self):
        self.pin.value = False
        self.state = False


# ──────────────────────────────────────────────────────────────────
# 4. KALIBRASYON FONKSİYONLARI
# ──────────────────────────────────────────────────────────────────

def calibrate_gas(vout, vref, a, b, offset, temp_c):
    """
    Gaz sensörü kalibrasyon zinciri:
      1. Power-law model    : C = a * (Vout/Vref)^b
      2. Offset düzeltme    : C_corr = C - offset
      3. Sıcaklık kompanze  : C_comp = C_corr * (1 + alpha*(T-Tref))
    """
    if vref == 0 or vout < 0:
        return 0.0
    raw_conc   = a * ((vout / vref) ** b)
    corrected  = raw_conc - offset
    compensated = corrected * (1.0 + TEMP_ALPHA * (temp_c - TEMP_REF))
    return max(0.0, compensated)


def calc_altitude(pressure_hpa):
    """
    Uluslararası barometrik formül:
      h = 44330 * [1 - (P/P0)^0.1903]
    """
    if pressure_hpa <= 0:
        return 0.0
    return 44330.0 * (1.0 - (pressure_hpa / P0) ** 0.1903)


def altitude_uncertainty(pressure_hpa, delta_p=1.0):
    """
    Basınç hatasından kaynaklanan yükseklik belirsizliği:
      Δh = |dh/dP| * ΔP
      dh/dP = -44330 * 0.1903 * (1/P0^0.1903) * P^(-0.8097)
    """
    if pressure_hpa <= 0:
        return 0.0
    dh_dp = (-44330.0 * 0.1903
             * (1.0 / (P0 ** 0.1903))
             * (pressure_hpa ** (-0.8097)))
    return abs(dh_dp) * delta_p


def read_battery_voltage():
    """
    ADC ölçümünden batarya gerilimini hesapla:
      V = (ADC / resolution) * Vref * divider_ratio
    """
    raw = bat_adc.value
    return (raw / ADC_RESOLUTION) * VREF * VDIV_RATIO


# ──────────────────────────────────────────────────────────────────
# 5. LORA PAKET YAPISI
# ──────────────────────────────────────────────────────────────────
#
#  Binary paket formatı (toplam ~28 byte):
#  [Header 2B][Zaman 4B][Yükseklik 4B][Sıcaklık 2B]
#  [CO2 2B][CH4 2B][PM25 2B][Batarya 2B][Bayrak 1B][CRC 2B]
#
#  Header: 0xAB 0xCD (senkronizasyon)
#  Bayrak bit0: ısıtıcı durumu
#  CRC: tüm byte'ların XOR'u

PACKET_HEADER = b'\xAB\xCD'

def build_lora_packet(ts, altitude, temp, co2, ch4, pm25, bat_v, heater_on):
    """
    Ölçüm verilerini binary pakete dönüştür.
    struct format: >HHfhHHHHB
    """
    flag = 0x01 if heater_on else 0x00
    payload = struct.pack(
        ">IfhHHHHB",
        int(ts),           # 4B  zaman damgası (saniye)
        float(altitude),   # 4B  yükseklik (m) — float
        int(temp * 10),    # 2B  sıcaklık * 10 (örn. -52 → -520)
        int(co2),          # 2B  CO2 (ppm)
        int(ch4 * 100),    # 2B  CH4 * 100
        int(pm25 * 10),    # 2B  PM2.5 * 10
        int(bat_v * 100),  # 2B  Batarya * 100
        flag               # 1B  bayraklar
    )
    # CRC: tüm payload byte'larının XOR'u
    crc = 0
    for byte in payload:
        crc ^= byte
    crc_bytes = struct.pack(">H", crc)
    return PACKET_HEADER + payload + crc_bytes


def verify_crc(packet):
    """Gelen paketin CRC doğrulaması."""
    if len(packet) < 4:
        return False
    data    = packet[:-2]
    rx_crc  = struct.unpack(">H", packet[-2:])[0]
    calc_crc = 0
    for b in data:
        calc_crc ^= b
    return rx_crc == calc_crc


def parse_ground_command(packet):
    """
    Yer istasyonundan gelen komut paketini çöz.
    Basit format: [0xAC][CMD 1B]
      CMD 0x01 = ACK
      CMD 0x02 = Safe-mode isteği
      CMD 0x03 = Örnek aralığını değiştir
    """
    if len(packet) >= 2 and packet[0] == 0xAC:
        cmd = packet[1]
        return cmd
    return None


# ──────────────────────────────────────────────────────────────────
# 6. SD KART YARDIMCILARI
# ──────────────────────────────────────────────────────────────────

def sd_write(line: str):
    """
    SD karta güvenli CSV satırı yaz.
    Her yazımda dosya kapatılır → güç kaybına karşı veri bütünlüğü.
    """
    try:
        with open(LOG_FILE, "a") as f:
            f.write(line)
            f.flush()           # Tampon boşalt
    except OSError as e:
        print("[SD] Yazma hatası:", e)
        return False
    return True


def sd_init_header():
    """Dosya yoksa başlık satırını yaz."""
    try:
        with open(LOG_FILE, "r"):
            pass                # Dosya zaten var
    except OSError:
        sd_write(CSV_HEADER)


# ──────────────────────────────────────────────────────────────────
# 7. BAŞLATMA (INITIALIZATION)
# ──────────────────────────────────────────────────────────────────

def init_system():
    """
    Tüm alt sistemleri başlat.
    Hatalı başlayan sensörler için error_flags sözlüğü tutulur.
    """
    error_flags = {
        "bmp280": False,
        "scd30":  False,
        "pm25":   False,
        "sd":     False,
        "lora":   False,
    }

    print("=" * 50)
    print("  CubeSat Onboard System Başlatılıyor...")
    print("=" * 50)

    # --- SD Kart ---
    try:
        sdcard = sdcardio.SDCard(spi, sd_cs)
        vfs    = storage.VfsFat(sdcard)
        storage.mount(vfs, "/sd")
        sd_init_header()
        print("[OK] SD kart bağlandı.")
    except Exception as e:
        print("[HATA] SD kart:", e)
        error_flags["sd"] = True

    # --- LoRa Modülü ---
    try:
        rfm9x = adafruit_rfm9x.RFM9x(
            spi, lora_cs, lora_reset, LORA_FREQ
        )
        rfm9x.tx_power       = LORA_TX_POWER
        rfm9x.spreading_factor = LORA_SPREADING
        rfm9x.signal_bandwidth = LORA_BANDWIDTH
        rfm9x.coding_rate    = LORA_CODING_RATE
        print("[OK] LoRa modülü hazır.")
    except Exception as e:
        print("[HATA] LoRa:", e)
        rfm9x = None
        error_flags["lora"] = True

    # --- BMP280 (Basınç + Sıcaklık) ---
    try:
        bmp = adafruit_bmp280.Adafruit_BMP280_I2C(i2c, address=0x76)
        bmp.sea_level_pressure = P0
        print("[OK] BMP280 hazır.")
    except Exception as e:
        print("[HATA] BMP280:", e)
        bmp = None
        error_flags["bmp280"] = True

    # --- SCD30 (CO2 + Nem) ---
    try:
        scd = adafruit_scd30.SCD30(i2c)
        scd.measurement_interval = 2   # saniye
        print("[OK] SCD30 (CO2) hazır.")
        print("     SCD30 ısınıyor, 30 saniye bekleniyor...")
        time.sleep(30)                 # CO2 sensörü ısınma süresi
    except Exception as e:
        print("[HATA] SCD30:", e)
        scd = None
        error_flags["scd30"] = True

    # --- PM2.5 Sensörü (UART) ---
    try:
        pm25_sensor = adafruit_pm25.PM25_UART(uart)
        print("[OK] PM2.5 sensörü hazır.")
    except Exception as e:
        print("[HATA] PM2.5:", e)
        pm25_sensor = None
        error_flags["pm25"] = True

    # Hata özeti
    active_errors = [k for k, v in error_flags.items() if v]
    if active_errors:
        print("[UYARI] Başlatma hataları:", active_errors)
    else:
        print("[OK] Tüm sistemler hazır.")
    print("=" * 50)

    return bmp, scd, pm25_sensor, rfm9x, error_flags


# ──────────────────────────────────────────────────────────────────
# 8. SAFE-MODE
# ──────────────────────────────────────────────────────────────────

def enter_safe_mode(rfm9x, thermal_ctrl, reason="unknown"):
    """
    Safe-mode:
      • Isıtıcıyı kapat
      • Örnekleme aralığını artır
      • Acil işareti gönder
    """
    print(f"[SAFE-MODE] Neden: {reason}")
    thermal_ctrl.force_off()

    if rfm9x:
        beacon = b'\xFF\xFF' + reason.encode()[:10]
        try:
            rfm9x.send(beacon)
            print("[SAFE-MODE] Acil işareti gönderildi.")
        except Exception:
            pass

    return 60    # Yeni örnekleme aralığı (saniye)


# ──────────────────────────────────────────────────────────────────
# 9. ANA DÖNGÜ
# ──────────────────────────────────────────────────────────────────

def main():
    # ── Başlatma ──────────────────────────────────────────────────
    bmp, scd, pm25_sensor, rfm9x, errors = init_system()
    thermal_ctrl = ThermalController(heater_pin)

    # ── Filtreler (her kanal için ayrı) ───────────────────────────
    f_pressure = MovingAverageFilter()
    f_temp     = MovingAverageFilter()
    f_co2      = MovingAverageFilter()
    f_ch4      = MovingAverageFilter()
    f_pm25     = MovingAverageFilter()

    # ── Zamanlayıcılar ─────────────────────────────────────────────
    t_last_sample  = time.monotonic()
    t_last_tx      = time.monotonic()
    mission_start  = time.monotonic()
    sample_interval = SAMPLE_INTERVAL_SEC
    safe_mode_active = False

    # ── CH4 için simüle ADC pini ───────────────────────────────────
    # Gerçek donanımda: ch4_adc = analogio.AnalogIn(board.A1)
    # Burada placeholder olarak sabit değer kullanılıyor
    CH4_ADC_VALUE = 32000   # Gerçek donanımda: ch4_adc.value

    print("\n[GÖREV] Ana döngü başladı.\n")

    # ══════════════════════════════════════════════════════════════
    while True:
        now = time.monotonic()

        # ── Watchdog besleme (donanım WDT varsa buraya ekle) ──────
        # microcontroller.watchdog.feed()

        # ── Batarya kontrolü ──────────────────────────────────────
        bat_v = read_battery_voltage()
        if bat_v < VBAT_MIN and not safe_mode_active:
            sample_interval  = enter_safe_mode(
                rfm9x, thermal_ctrl, reason="low_battery"
            )
            safe_mode_active = True

        # ─────────────────────────────────────────────────────────
        # ÖRNEKLEME BLOĞU
        # ─────────────────────────────────────────────────────────
        if now - t_last_sample >= sample_interval:
            t_last_sample = now
            elapsed = now - mission_start   # Görev süresi (saniye)

            # ── Ham Sensör Okumaları ───────────────────────────────
            pressure_raw = bmp.pressure     if bmp else 500.0
            temp_raw     = bmp.temperature  if bmp else -30.0
            co2_raw_ppm  = scd.CO2          if (scd and scd.data_available) else 400.0
            rh           = scd.relative_humidity if (scd and scd.data_available) else 50.0
            pm25_raw     = 0.0
            if pm25_sensor:
                try:
                    pm_data  = pm25_sensor.read()
                    pm25_raw = pm_data.get("particles 25um", 0.0)
                except Exception:
                    pm25_raw = 0.0

            # CH4 için analog okuma ve kalibrasyon
            ch4_vout = (CH4_ADC_VALUE / ADC_RESOLUTION) * VREF
            ch4_ppm  = calibrate_gas(
                ch4_vout, VREF, CH4_A, CH4_B, CH4_OFFSET, temp_raw
            )

            # CO2 zaten ppm cinsinden geliyor, sadece sıcaklık kompanze
            co2_ppm = co2_raw_ppm * (1.0 + TEMP_ALPHA * (temp_raw - TEMP_REF))
            co2_ppm = max(0.0, co2_ppm - CO2_OFFSET)

            # ── Aykırı Değer Kontrolü ve Filtreleme ───────────────
            if not f_pressure.is_outlier(pressure_raw):
                pressure = f_pressure.update(pressure_raw)
            else:
                pressure = f_pressure.mean()
                print(f"[FİLTRE] Basınç aykırı değer atlandı: {pressure_raw:.1f}")

            if not f_temp.is_outlier(temp_raw):
                temp = f_temp.update(temp_raw)
            else:
                temp = f_temp.mean()

            if not f_co2.is_outlier(co2_ppm):
                co2 = f_co2.update(co2_ppm)
            else:
                co2 = f_co2.mean()
                print(f"[FİLTRE] CO2 aykırı değer atlandı: {co2_ppm:.1f}")

            if not f_ch4.is_outlier(ch4_ppm):
                ch4 = f_ch4.update(ch4_ppm)
            else:
                ch4 = f_ch4.mean()

            if not f_pm25.is_outlier(pm25_raw):
                pm25 = f_pm25.update(pm25_raw)
            else:
                pm25 = f_pm25.mean()

            # ── Yükseklik Hesabı ──────────────────────────────────
            altitude = calc_altitude(pressure)
            alt_err  = altitude_uncertainty(pressure, delta_p=1.0)

            # ── Termal Kontrol ────────────────────────────────────
            heater_on = thermal_ctrl.update(temp)
            heater_str = "1" if heater_on else "0"

            # ── GPS (placeholder) ─────────────────────────────────
            # Gerçek uygulamada: lat, lon = gps.latitude, gps.longitude
            lat, lon = 0.0, 0.0

            # ── UV (placeholder — analog ADC veya I2C UV sensörü) ─
            uv_index = 0.0

            # ── Durum Çıktısı ─────────────────────────────────────
            print(
                f"[{elapsed:7.1f}s] "
                f"Alt={altitude:7.1f}m(±{alt_err:.1f}m) "
                f"P={pressure:7.2f}hPa "
                f"T={temp:6.1f}°C "
                f"CO2={co2:6.1f}ppm "
                f"CH4={ch4:5.2f}ppm "
                f"PM25={pm25:5.1f}μg "
                f"Bat={bat_v:.2f}V "
                f"Isıtıcı={'ON' if heater_on else 'OFF'}"
            )

            # ── SD'ye CSV Yazma ───────────────────────────────────
            csv_line = (
                f"{elapsed:.1f},{lat:.6f},{lon:.6f},"
                f"{pressure:.2f},{altitude:.1f},"
                f"{temp:.2f},{temp:.2f},"    # internal = external placeholder
                f"{co2:.1f},{ch4:.3f},"
                f"{pm25:.1f},{uv_index:.2f},{rh:.1f},"
                f"{bat_v:.3f},{heater_str}\n"
            )
            if not errors["sd"]:
                sd_write(csv_line)

        # ─────────────────────────────────────────────────────────
        # LORA TX BLOĞU — Her TX_INTERVAL_SEC'de bir gönder
        # ─────────────────────────────────────────────────────────
        if now - t_last_tx >= TX_INTERVAL_SEC:
            t_last_tx = now

            if rfm9x and not errors["lora"]:
                try:
                    # Son filtrelenmiş değerleri paketle
                    packet = build_lora_packet(
                        ts       = now - mission_start,
                        altitude = calc_altitude(f_pressure.mean()),
                        temp     = f_temp.mean(),
                        co2      = f_co2.mean(),
                        ch4      = f_ch4.mean(),
                        pm25     = f_pm25.mean(),
                        bat_v    = bat_v,
                        heater_on = heater_pin.value
                    )
                    rfm9x.send(packet)
                    print(f"[LORA TX] {len(packet)} byte gönderildi.")

                except Exception as e:
                    print("[LORA TX] Gönderim hatası:", e)

        # ─────────────────────────────────────────────────────────
        # LORA RX BLOĞU — Yer istasyonundan komut/ACK dinle
        #   rfm9x.receive(timeout=0.5) → bloke olmayan kısa bekleme
        # ─────────────────────────────────────────────────────────
        if rfm9x and not errors["lora"]:
            try:
                rx_packet = rfm9x.receive(timeout=0.5)   # 500 ms dinle
                if rx_packet is not None:
                    rssi = rfm9x.last_rssi
                    print(f"[LORA RX] Paket alındı, RSSI={rssi} dBm")

                    cmd = parse_ground_command(rx_packet)

                    if cmd == 0x01:
                        print("[LORA RX] ACK alındı — yer istasyonu bağlantısı sağlıklı.")

                    elif cmd == 0x02:
                        print("[LORA RX] Safe-mode komutu alındı!")
                        sample_interval  = enter_safe_mode(
                            rfm9x, thermal_ctrl, reason="ground_command"
                        )
                        safe_mode_active = True

                    elif cmd == 0x03:
                        # Yeni örnekleme aralığı payload'ında kodlanmış
                        if len(rx_packet) >= 3:
                            new_interval = rx_packet[2]
                            if 5 <= new_interval <= 120:
                                sample_interval = new_interval
                                print(f"[LORA RX] Örnekleme aralığı → {new_interval}s")

                    elif cmd is not None:
                        print(f"[LORA RX] Bilinmeyen komut: 0x{cmd:02X}")

            except Exception as e:
                print("[LORA RX] Alım hatası:", e)

        # ─────────────────────────────────────────────────────────
        # CPU uyutma — gereksiz işlem yapmayı önle
        # ─────────────────────────────────────────────────────────
        time.sleep(0.1)

    # ══════════════════════════════════════════════════════════════


# ──────────────────────────────────────────────────────────────────
# GİRİŞ NOKTASI
# ──────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    main()
