"""
╔══════════════════════════════════════════════════════════════════╗
║   STRATOSPHERIC CUBESAT — KISIM 2: YER İSTASYONU SİSTEMİ       ║
║   Hedef Donanım : Raspberry Pi / PC + LoRa RFM9x                ║
║   Dil           : Python 3.9+                                    ║
║                                                                  ║
║   Bu script şunları yapar:                                       ║
║     • LoRa alıcısını başlatır ve paketleri çözümler             ║
║     • Gelen verileri CSV'ye yazar                                ║
║     • ACK / komut paketi gönderir (uplink)                       ║
║     • Görev tamamlandığında CSV'yi yükler                        ║
║     • 3 bilimsel grafik üretir (Matplotlib)                      ║
║         1. Yükseklik vs Zaman                                    ║
║         2. CO2 Konsantrasyonu vs Yükseklik                       ║
║         3. Sıcaklık vs Yükseklik                                 ║
║     • Her grafik için otomatik bilimsel yorum üretir             ║
║     • Tüm istatistikleri konsola ve rapora yazar                 ║
╚══════════════════════════════════════════════════════════════════╝

KURULUM:
  pip install pyserial adafruit-circuitpython-rfm9x matplotlib pandas numpy scipy

ÇALIŞTIRMA:
  python cubesat_part2_ground.py --port /dev/ttyUSB0 --csv flight_log.csv
  python cubesat_part2_ground.py --csv existing_data.csv --no-lora  (sadece analiz)
"""

# ──────────────────────────────────────────────────────────────────
# KÜTÜPHANE İMPORTLARI
# ──────────────────────────────────────────────────────────────────
import argparse
import csv
import math
import os
import struct
import sys
import time
import threading
from datetime import datetime

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.ticker import AutoMinorLocator
from scipy.stats import linregress
from scipy.signal import savgol_filter

# LoRa (Raspberry Pi veya Linux ortamında)
try:
    import board
    import busio
    import digitalio
    import adafruit_rfm9x as rfm9x_lib
    LORA_AVAILABLE = True
except (ImportError, NotImplementedError):
    LORA_AVAILABLE = False
    print("[UYARI] LoRa donanım kütüphaneleri bulunamadı. Sadece CSV analiz modu.")

# ──────────────────────────────────────────────────────────────────
# 1. SABİTLER VE KONFİGÜRASYON
# ──────────────────────────────────────────────────────────────────

LORA_FREQ        = 868.0
LORA_TX_POWER    = 13
LORA_SPREADING   = 10
LORA_BANDWIDTH   = 125000
LORA_CODING_RATE = 5

P0               = 1013.25          # Referans basınç (hPa)
LAPSE_RATE       = -0.0065          # Troposfer sıcaklık düşüşü (°C/m)

PACKET_HEADER    = b'\xAB\xCD'
ACK_PACKET       = b'\xAC\x01'     # ACK komutu

CSV_HEADER = (
    "time_s,latitude,longitude,pressure_hPa,altitude_m,"
    "temp_internal_C,temp_external_C,co2_ppm,ch4_ppm,"
    "pm25_ugm3,uv_index,humidity_pct,battery_V,heater\n"
)

# Grafik stil sabitleri
COLORS = {
    "altitude"   : "#00B4D8",
    "co2"        : "#E63946",
    "temp_meas"  : "#F4A261",
    "temp_theory": "#2A9D8F",
    "fit"        : "#FFFFFF",
    "bg"         : "#0D1B2A",
    "grid"       : "#1E3A5F",
    "text"       : "#E0F0FF",
}

# ──────────────────────────────────────────────────────────────────
# 2. LORA ALICI / VERİCİ
# ──────────────────────────────────────────────────────────────────

class LoRaGroundStation:
    """
    Yer istasyonu LoRa yöneticisi.
    - Sürekli paket dinler (RX thread)
    - ACK / komut gönderir (TX)
    - Gelen paketleri çözümleyip kuyrukta biriktirir
    """
    def __init__(self):
        self.rfm9x       = None
        self.rx_queue    = []
        self._running    = False
        self._lock       = threading.Lock()

    def init_hardware(self):
        if not LORA_AVAILABLE:
            print("[LORA] Donanım yok, simülasyon modunda çalışılıyor.")
            return False
        try:
            spi       = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
            cs        = digitalio.DigitalInOut(board.CE1)
            reset     = digitalio.DigitalInOut(board.D25)
            self.rfm9x = rfm9x_lib.RFM9x(spi, cs, reset, LORA_FREQ)
            self.rfm9x.tx_power        = LORA_TX_POWER
            self.rfm9x.spreading_factor = LORA_SPREADING
            self.rfm9x.signal_bandwidth = LORA_BANDWIDTH
            self.rfm9x.coding_rate     = LORA_CODING_RATE
            print("[LORA] Yer istasyonu LoRa hazır.")
            return True
        except Exception as e:
            print(f"[LORA] Başlatma hatası: {e}")
            return False

    def start_rx_thread(self):
        """Arka planda sürekli dinleyen thread başlat."""
        self._running = True
        t = threading.Thread(target=self._rx_loop, daemon=True)
        t.start()
        print("[LORA] RX thread başlatıldı.")

    def _rx_loop(self):
        """RX döngüsü — ayrı thread'de çalışır."""
        while self._running:
            if self.rfm9x is None:
                time.sleep(1)
                continue
            try:
                pkt = self.rfm9x.receive(timeout=1.0)
                if pkt and len(pkt) >= 2 and pkt[:2] == PACKET_HEADER:
                    parsed = parse_lora_packet(pkt)
                    if parsed:
                        rssi = self.rfm9x.last_rssi
                        parsed["rssi_dBm"] = rssi
                        with self._lock:
                            self.rx_queue.append(parsed)
                        # ACK gönder
                        self.send_command(ACK_PACKET)
                        print(
                            f"[LORA RX] "
                            f"Alt={parsed['altitude_m']:.0f}m "
                            f"T={parsed['temp_C']:.1f}°C "
                            f"CO2={parsed['co2_ppm']:.0f}ppm "
                            f"RSSI={rssi}dBm"
                        )
            except Exception as e:
                print(f"[LORA RX] Hata: {e}")
                time.sleep(0.5)

    def send_command(self, cmd_bytes):
        if self.rfm9x:
            try:
                self.rfm9x.send(cmd_bytes)
            except Exception as e:
                print(f"[LORA TX] Komut gönderilemedi: {e}")

    def get_all(self):
        """Kuyruktan tüm ölçümleri al ve kuyrugu temizle."""
        with self._lock:
            items = list(self.rx_queue)
            self.rx_queue.clear()
        return items

    def stop(self):
        self._running = False


# ──────────────────────────────────────────────────────────────────
# 3. PAKET ÇÖZÜMLEME
# ──────────────────────────────────────────────────────────────────

def verify_crc(packet):
    if len(packet) < 4:
        return False
    # CRC yalnızca payload (header hariç) byte'larının XOR'udur.
    data    = packet[2:-2]
    rx_crc  = struct.unpack(">H", packet[-2:])[0]
    calc_crc = 0
    for b in data:
        calc_crc ^= b
    return rx_crc == calc_crc


def parse_lora_packet(raw_bytes):
    """
    Binary paketi çözümle ve sözlük olarak döndür.
    Header (2B) atlanır, son 2B CRC.
    Payload: IfhHHHHB = 4+4+2+2+2+2+2+1 = 19 byte
    """
    if not verify_crc(raw_bytes):
        print("[PAKET] CRC hatası, paket atlandı.")
        return None

    payload = raw_bytes[2:-2]   # Header ve CRC çıkar
    if len(payload) < 19:
        print(f"[PAKET] Kısa payload: {len(payload)} byte")
        return None

    try:
        (time_s, altitude, temp_x10,
         co2_ppm, ch4_x100, pm25_x10,
         bat_x100, flag) = struct.unpack(">IfhHHHHB", payload[:19])

        return {
            "time_s"     : time_s,
            "altitude_m" : altitude,
            "temp_C"     : temp_x10 / 10.0,
            "co2_ppm"    : float(co2_ppm),
            "ch4_ppm"    : ch4_x100 / 100.0,
            "pm25_ugm3"  : pm25_x10 / 10.0,
            "battery_V"  : bat_x100 / 100.0,
            "heater"     : bool(flag & 0x01),
        }
    except struct.error as e:
        print(f"[PAKET] Çözümleme hatası: {e}")
        return None


# ──────────────────────────────────────────────────────────────────
# 4. CSV YÖNETIMI
# ──────────────────────────────────────────────────────────────────

class CSVWriter:
    """Gelen LoRa verilerini gerçek zamanlı CSV'ye yazar."""

    def __init__(self, filepath):
        self.filepath = filepath
        self._init_file()

    def _init_file(self):
        if not os.path.exists(self.filepath):
            with open(self.filepath, "w", newline="") as f:
                f.write(CSV_HEADER)
            print(f"[CSV] Yeni dosya oluşturuldu: {self.filepath}")
        else:
            print(f"[CSV] Mevcut dosyaya ekleniyor: {self.filepath}")

    def write(self, record: dict):
        """Tek bir ölçüm satırını CSV'ye ekle."""
        row = (
            f"{record.get('time_s', 0):.1f},"
            f"0.000000,0.000000,"                      # lat, lon placeholder
            f"0.00,"                                   # pressure placeholder
            f"{record.get('altitude_m', 0):.1f},"
            f"{record.get('temp_C', 0):.2f},"
            f"{record.get('temp_C', 0):.2f},"
            f"{record.get('co2_ppm', 0):.1f},"
            f"{record.get('ch4_ppm', 0):.3f},"
            f"{record.get('pm25_ugm3', 0):.1f},"
            f"0.00,"                                   # UV placeholder
            f"0.0,"                                    # RH placeholder
            f"{record.get('battery_V', 0):.3f},"
            f"{'1' if record.get('heater') else '0'}\n"
        )
        with open(self.filepath, "a", newline="") as f:
            f.write(row)


# ──────────────────────────────────────────────────────────────────
# 5. VERİ YÜKLEME VE ÖN İŞLEME
# ──────────────────────────────────────────────────────────────────

def load_csv(filepath):
    """
    CSV dosyasını Pandas DataFrame olarak yükle.
    Eksik değerleri temizle, zamanı görev-saniyesine dönüştür.
    """
    print(f"\n[VERİ] CSV yükleniyor: {filepath}")
    df = pd.read_csv(filepath)

    # Sütun adlarını temizle
    df.columns = df.columns.str.strip()

    # Eksik değerleri at
    before = len(df)
    df.dropna(inplace=True)
    after  = len(df)
    if before != after:
        print(f"[VERİ] {before - after} eksik satır silindi.")

    # Zamanı sıfırdan başlat
    df["time_s"] = df["time_s"] - df["time_s"].min()

    # Opsiyonel: Savitzky-Golay düzleştirme (pencere tek sayı olmalı)
    if len(df) >= 11:
        window = min(11, len(df) if len(df) % 2 == 1 else len(df) - 1)
        if window >= 3:
            for col in ["altitude_m", "co2_ppm", "temp_internal_C"]:
                if col in df.columns:
                    df[f"{col}_smooth"] = savgol_filter(
                        df[col], window_length=window, polyorder=2
                    )
    else:
        for col in ["altitude_m", "co2_ppm", "temp_internal_C"]:
            if col in df.columns:
                df[f"{col}_smooth"] = df[col]

    print(f"[VERİ] {len(df)} satır yüklendi.")
    print(df[["time_s", "altitude_m", "co2_ppm", "temp_internal_C"]].describe().round(2))
    return df


def calc_altitude(pressure_hpa):
    if pressure_hpa <= 0:
        return 0.0
    return 44330.0 * (1.0 - (pressure_hpa / P0) ** 0.1903)


# ──────────────────────────────────────────────────────────────────
# 6. İSTATİSTİKSEL ANALİZ
# ──────────────────────────────────────────────────────────────────

def compute_statistics(df):
    """Tüm görev istatistiklerini hesapla ve döndür."""
    stats = {}

    # Yükseklik
    stats["max_altitude_m"]    = df["altitude_m"].max()
    stats["min_altitude_m"]    = df["altitude_m"].min()
    stats["mean_altitude_m"]   = df["altitude_m"].mean()

    # Tırmanma hızı (m/s) — diff / dt
    if len(df) > 1:
        dh = np.diff(df["altitude_m"].values)
        dt = np.diff(df["time_s"].values)
        dt[dt == 0] = 1e-6
        climb_rates = dh / dt
        stats["max_climb_rate_ms"]  = float(np.max(climb_rates))
        stats["mean_climb_rate_ms"] = float(np.mean(climb_rates[climb_rates > 0]))
    else:
        stats["max_climb_rate_ms"]  = 0.0
        stats["mean_climb_rate_ms"] = 0.0

    # Sıcaklık
    stats["min_temp_C"]  = df["temp_internal_C"].min()
    stats["max_temp_C"]  = df["temp_internal_C"].max()
    stats["mean_temp_C"] = df["temp_internal_C"].mean()

    # Sıcaklık düşüş hızı (°C/m) — lineer regresyon
    alt = df["altitude_m"].values
    tmp = df["temp_internal_C"].values
    if len(alt) > 2:
        slope, intercept, r2, *_ = linregress(alt, tmp)
        stats["lapse_rate_Cm"]  = slope
        stats["lapse_rate_R2"]  = r2 ** 2
    else:
        stats["lapse_rate_Cm"] = LAPSE_RATE
        stats["lapse_rate_R2"] = 0.0

    # CO2
    stats["mean_co2_ppm"] = df["co2_ppm"].mean()
    stats["min_co2_ppm"]  = df["co2_ppm"].min()
    stats["max_co2_ppm"]  = df["co2_ppm"].max()

    # Görev süresi
    stats["mission_duration_s"] = df["time_s"].max()
    stats["sample_count"]       = len(df)

    return stats


# ──────────────────────────────────────────────────────────────────
# 7. OTOMATİK YORUM MOTORU
# ──────────────────────────────────────────────────────────────────

def generate_interpretation(stats, graph_name):
    """
    İstatistiksel sonuçlara göre otomatik bilimsel yorum üret.
    Her grafik için ayrı yorum bloğu döner.
    """

    interpretations = {}

    # ── Grafik 1: Yükseklik vs Zaman ────────────────────────────
    max_alt  = stats["max_altitude_m"]
    climb    = stats["max_climb_rate_ms"]
    mean_cl  = stats["mean_climb_rate_ms"]
    dur_min  = stats["mission_duration_s"] / 60

    alt_interp = [
        f"Uçuş Profili Analizi",
        f"─" * 40,
        f"• Maksimum yükseklik      : {max_alt:,.0f} m",
        f"• Ortalama tırmanma hızı  : {mean_cl:.2f} m/s  ({mean_cl*60:.1f} m/dk)",
        f"• Maksimum tırmanma hızı  : {climb:.2f} m/s",
        f"• Görev süresi            : {dur_min:.1f} dakika",
        f"",
        "Yorum:",
    ]

    if max_alt > 15000:
        alt_interp.append(
            f"  ✓ Yük, stratosfer sınırına ({max_alt/1000:.1f} km) ulaştı."
        )
    elif max_alt > 10000:
        alt_interp.append(
            f"  ✓ Yük, üst troposfer katmanına ({max_alt/1000:.1f} km) ulaştı."
        )
    else:
        alt_interp.append(
            f"  ⚠ Yük beklenen irtifanın altında kaldı ({max_alt/1000:.1f} km)."
        )

    if mean_cl > 4.0:
        alt_interp.append("  ✓ Tırmanma hızı nominal değerlerin üzerinde (>4 m/s).")
    elif mean_cl > 2.0:
        alt_interp.append("  ✓ Tırmanma hızı nominal aralıkta (2–4 m/s).")
    else:
        alt_interp.append("  ⚠ Düşük tırmanma hızı — rüzgar veya balon hacmi kontrol edilmeli.")

    interpretations["altitude_time"] = "\n".join(alt_interp)

    # ── Grafik 2: CO2 vs Yükseklik ──────────────────────────────
    co2_mean = stats["mean_co2_ppm"]
    co2_min  = stats["min_co2_ppm"]
    co2_max  = stats["max_co2_ppm"]
    co2_range = co2_max - co2_min

    co2_interp = [
        f"CO₂ Konsantrasyonu Analizi",
        f"─" * 40,
        f"• Ortalama CO₂  : {co2_mean:.1f} ppm",
        f"• Minimum CO₂   : {co2_min:.1f} ppm  (üst atmosfer)",
        f"• Maksimum CO₂  : {co2_max:.1f} ppm  (yüzey yakını)",
        f"• Toplam değişim: {co2_range:.1f} ppm",
        f"",
        "Yorum:",
    ]

    if co2_range > 20:
        co2_interp.append(
            "  ✓ Yükseklikle belirgin CO₂ azalması → beklenen atmosferik davranış."
        )
    elif co2_range > 5:
        co2_interp.append(
            "  ✓ Orta düzey CO₂ gradienti saptandı, troposfer karışımıyla uyumlu."
        )
    else:
        co2_interp.append(
            "  ⚠ CO₂ gradienti çok küçük — sensör kalibrasyonu doğrulanmalı."
        )

    if 380 <= co2_mean <= 430:
        co2_interp.append(
            f"  ✓ Ortalama CO₂ ({co2_mean:.0f} ppm) küresel atmosferik normla uyumlu."
        )
    elif co2_mean > 430:
        co2_interp.append(
            f"  ⚠ Yüksek CO₂ ({co2_mean:.0f} ppm) — kentsel kirlilik veya sensör kayması olabilir."
        )

    interpretations["co2_altitude"] = "\n".join(co2_interp)

    # ── Grafik 3: Sıcaklık vs Yükseklik ─────────────────────────
    lapse   = stats["lapse_rate_Cm"]
    r2      = stats["lapse_rate_R2"]
    t_min   = stats["min_temp_C"]
    t_max   = stats["max_temp_C"]
    theor   = LAPSE_RATE * 1000    # °C / km

    temp_interp = [
        f"Sıcaklık - İrtifa Analizi",
        f"─" * 40,
        f"• Ölçülen düşüş hızı  : {lapse*1000:.2f} °C/km",
        f"• Teorik değer        : {theor:.2f} °C/km (ISA Standardı)",
        f"• Regresyon R²        : {r2:.4f}",
        f"• Min sıcaklık        : {t_min:.1f} °C",
        f"• Maks sıcaklık       : {t_max:.1f} °C",
        f"",
        "Yorum:",
    ]

    deviation = abs(lapse * 1000 - theor)
    if deviation < 1.0:
        temp_interp.append(
            f"  ✓ Ölçülen lapse rate ISA standardıyla mükemmel uyum (±{deviation:.2f} °C/km)."
        )
    elif deviation < 3.0:
        temp_interp.append(
            f"  ✓ Kabul edilebilir sapma (±{deviation:.2f} °C/km) — hava kütlesi kararsızlığı."
        )
    else:
        temp_interp.append(
            f"  ⚠ Büyük sapma (±{deviation:.2f} °C/km) — inversiyon katmanı olabilir."
        )

    if r2 > 0.90:
        temp_interp.append(
            f"  ✓ R²={r2:.3f} — sıcaklık-irtifa ilişkisi güçlü ve lineer."
        )
    elif r2 > 0.70:
        temp_interp.append(
            f"  ✓ R²={r2:.3f} — orta düzey lineer ilişki, atmosferik karışım etkili."
        )
    else:
        temp_interp.append(
            f"  ⚠ R²={r2:.3f} — düşük korelasyon, sensör gürültüsü veya inversiyon."
        )

    if t_min < -40:
        temp_interp.append(
            f"  ✓ Minimum {t_min:.1f}°C troposfer üst sınırıyla uyumlu."
        )

    interpretations["temp_altitude"] = "\n".join(temp_interp)

    return interpretations


# ──────────────────────────────────────────────────────────────────
# 8. GRAFİK ÜRETIMI
# ──────────────────────────────────────────────────────────────────

def setup_plot_style():
    """Koyu uzay temalı genel grafik stili."""
    plt.rcParams.update({
        "figure.facecolor"  : COLORS["bg"],
        "axes.facecolor"    : COLORS["bg"],
        "axes.edgecolor"    : COLORS["grid"],
        "axes.labelcolor"   : COLORS["text"],
        "axes.titlecolor"   : COLORS["text"],
        "xtick.color"       : COLORS["text"],
        "ytick.color"       : COLORS["text"],
        "grid.color"        : COLORS["grid"],
        "grid.linewidth"    : 0.5,
        "text.color"        : COLORS["text"],
        "font.family"       : "monospace",
        "figure.dpi"        : 150,
    })


def add_interpretation_box(ax, text, fontsize=7.5):
    """Grafiğin yanına yorum metin kutusu ekle."""
    ax.text(
        1.02, 1.0, text,
        transform      = ax.transAxes,
        fontsize       = fontsize,
        verticalalignment = "top",
        bbox           = dict(
            boxstyle  = "round,pad=0.5",
            facecolor = "#0A2540",
            edgecolor = "#1E6FA8",
            alpha     = 0.85,
        ),
        color          = COLORS["text"],
        family         = "monospace",
    )


def plot_altitude_vs_time(df, stats, interp, output_dir):
    """
    Grafik 1: Yükseklik vs Zaman
      - Ham + düzleştirilmiş yükseklik
      - Tırmanma fazları renklendirilmiş
      - İstatistik açıklamaları
    """
    fig, ax = plt.subplots(figsize=(13, 6))
    fig.subplots_adjust(right=0.68)

    t   = df["time_s"].values / 60        # dakikaya dönüştür
    alt = df["altitude_m"].values
    alt_s = df.get("altitude_m_smooth", df["altitude_m"]).values

    # Ham veri (soluk)
    ax.plot(t, alt, color=COLORS["altitude"], alpha=0.25, linewidth=0.8,
            label="Ham veri")

    # Düzleştirilmiş
    ax.plot(t, alt_s, color=COLORS["altitude"], linewidth=2.2,
            label="Düzleştirilmiş (S-G)")

    # Max yükseklik işareti
    i_max = np.argmax(alt_s)
    ax.axhline(alt_s[i_max], color="#FFD60A", linewidth=0.8, linestyle="--", alpha=0.6)
    ax.annotate(
        f" {alt_s[i_max]:.0f} m (maks)",
        xy=(t[i_max], alt_s[i_max]),
        color="#FFD60A", fontsize=9,
        arrowprops=dict(arrowstyle="->", color="#FFD60A"),
        xytext=(t[i_max] + 0.5, alt_s[i_max] * 0.95)
    )

    ax.set_xlabel("Görev Süresi (dakika)", fontsize=11)
    ax.set_ylabel("Yükseklik (m)", fontsize=11)
    ax.set_title("Grafik 1 — Yükseklik vs Zaman", fontsize=13, fontweight="bold")
    ax.legend(loc="upper left", fontsize=9, facecolor="#0D1B2A", edgecolor="#1E3A5F")
    ax.grid(True, which="both", linestyle=":")
    ax.xaxis.set_minor_locator(AutoMinorLocator())
    ax.yaxis.set_minor_locator(AutoMinorLocator())

    add_interpretation_box(ax, interp["altitude_time"])

    path = os.path.join(output_dir, "01_altitude_time.png")
    plt.savefig(path, bbox_inches="tight", facecolor=COLORS["bg"])
    plt.close()
    print(f"[GRAFİK] Kaydedildi: {path}")
    return path


def plot_co2_vs_altitude(df, stats, interp, output_dir):
    """
    Grafik 2: CO2 Konsantrasyonu vs Yükseklik
      - Scatter + trend çizgisi
      - Renk haritası: CO2 yoğunluğuna göre
    """
    fig, ax = plt.subplots(figsize=(13, 6))
    fig.subplots_adjust(right=0.68)

    alt  = df["altitude_m"].values
    co2  = df["co2_ppm"].values
    co2_s = df.get("co2_ppm_smooth", df["co2_ppm"]).values

    # Scatter — renk CO2 yoğunluğuna göre
    sc = ax.scatter(alt, co2, c=co2, cmap="plasma", s=8, alpha=0.5, label="Ham ölçüm")
    plt.colorbar(sc, ax=ax, label="CO₂ (ppm)", shrink=0.8)

    # Düzleştirilmiş çizgi
    sorted_idx = np.argsort(alt)
    ax.plot(
        alt[sorted_idx], co2_s[sorted_idx],
        color=COLORS["co2"], linewidth=2.0, label="Düzleştirilmiş"
    )

    # Lineer regresyon
    slope, intercept, *_ = linregress(alt, co2)
    fit_x = np.linspace(alt.min(), alt.max(), 200)
    fit_y = slope * fit_x + intercept
    ax.plot(fit_x, fit_y, color=COLORS["fit"], linewidth=1.2,
            linestyle="--", label=f"Regresyon  (slope={slope*1000:.3f} ppm/km)")

    # Referans çizgisi (global ortalama ~415 ppm)
    ax.axhline(415, color="#4CC9F0", linewidth=0.8, linestyle=":",
               label="Küresel ort. (~415 ppm)")

    ax.set_xlabel("Yükseklik (m)", fontsize=11)
    ax.set_ylabel("CO₂ Konsantrasyonu (ppm)", fontsize=11)
    ax.set_title("Grafik 2 — CO₂ Konsantrasyonu vs Yükseklik", fontsize=13, fontweight="bold")
    ax.legend(loc="upper right", fontsize=8.5, facecolor="#0D1B2A", edgecolor="#1E3A5F")
    ax.grid(True, linestyle=":")

    add_interpretation_box(ax, interp["co2_altitude"])

    path = os.path.join(output_dir, "02_co2_altitude.png")
    plt.savefig(path, bbox_inches="tight", facecolor=COLORS["bg"])
    plt.close()
    print(f"[GRAFİK] Kaydedildi: {path}")
    return path


def plot_temperature_vs_altitude(df, stats, interp, output_dir):
    """
    Grafik 3: Sıcaklık vs Yükseklik
      - Ölçülen sıcaklık
      - ISA teorik lapse rate çizgisi
      - Lineer regresyon + R²
    """
    fig, ax = plt.subplots(figsize=(13, 6))
    fig.subplots_adjust(right=0.68)

    alt  = df["altitude_m"].values
    tmp  = df["temp_internal_C"].values
    tmp_s = df.get("temp_internal_C_smooth", df["temp_internal_C"]).values

    # Ölçülen sıcaklık (scatter)
    ax.scatter(alt, tmp, color=COLORS["temp_meas"], s=6, alpha=0.4, label="Ölçülen (ham)")

    # Düzleştirilmiş
    sorted_idx = np.argsort(alt)
    ax.plot(
        alt[sorted_idx], tmp_s[sorted_idx],
        color=COLORS["temp_meas"], linewidth=2.2, label="Düzleştirilmiş"
    )

    # Teorik ISA sıcaklık profili
    alt_range  = np.linspace(alt.min(), alt.max(), 200)
    t_surface  = tmp[np.argmin(alt)]     # Yüzey sıcaklığını ölçümden al
    tmp_theory = t_surface + LAPSE_RATE * alt_range
    ax.plot(alt_range, tmp_theory, color=COLORS["temp_theory"],
            linewidth=1.8, linestyle="--", label="ISA teorik (-6.5 °C/km)")

    # Lineer regresyon
    slope, intercept, r, *_ = linregress(alt, tmp)
    fit_y = slope * alt_range + intercept
    ax.plot(alt_range, fit_y, color=COLORS["fit"], linewidth=1.2,
            linestyle=":", label=f"Regresyon  R²={r**2:.3f}")

    # 0°C çizgisi
    ax.axhline(0, color="#F72585", linewidth=0.7, linestyle="-.",
               alpha=0.7, label="0 °C (donma noktası)")

    ax.set_xlabel("Yükseklik (m)", fontsize=11)
    ax.set_ylabel("Sıcaklık (°C)", fontsize=11)
    ax.set_title("Grafik 3 — Sıcaklık vs Yükseklik", fontsize=13, fontweight="bold")
    ax.legend(loc="upper right", fontsize=8.5, facecolor="#0D1B2A", edgecolor="#1E3A5F")
    ax.grid(True, linestyle=":")

    add_interpretation_box(ax, interp["temp_altitude"])

    path = os.path.join(output_dir, "03_temp_altitude.png")
    plt.savefig(path, bbox_inches="tight", facecolor=COLORS["bg"])
    plt.close()
    print(f"[GRAFİK] Kaydedildi: {path}")
    return path


def plot_dashboard(df, stats, output_dir):
    """
    Bonus — 4 panelli özet dashboard (tek PNG).
    Panel: Yükseklik | CO2 | Sıcaklık | Batarya
    """
    setup_plot_style()
    fig = plt.figure(figsize=(18, 10))
    fig.suptitle(
        "CubeSat Görev Özeti — Stratosferik Atmosferik Analiz",
        fontsize=15, fontweight="bold", color=COLORS["text"]
    )

    gs = gridspec.GridSpec(2, 2, hspace=0.40, wspace=0.35)

    t = df["time_s"].values / 60

    # Panel A — Yükseklik
    ax_a = fig.add_subplot(gs[0, 0])
    alt_s = df.get("altitude_m_smooth", df["altitude_m"]).values
    ax_a.plot(t, alt_s, color=COLORS["altitude"], linewidth=1.8)
    ax_a.fill_between(t, alt_s, alpha=0.2, color=COLORS["altitude"])
    ax_a.set_title("Yükseklik (m)")
    ax_a.set_xlabel("Süre (dk)")
    ax_a.grid(True, linestyle=":")

    # Panel B — CO2
    ax_b = fig.add_subplot(gs[0, 1])
    co2_s = df.get("co2_ppm_smooth", df["co2_ppm"]).values
    alt   = df["altitude_m"].values
    ax_b.scatter(alt, co2_s, c=co2_s, cmap="plasma", s=5, alpha=0.7)
    ax_b.set_title("CO₂ vs Yükseklik (ppm)")
    ax_b.set_xlabel("Yükseklik (m)")
    ax_b.grid(True, linestyle=":")

    # Panel C — Sıcaklık
    ax_c = fig.add_subplot(gs[1, 0])
    tmp_s = df.get("temp_internal_C_smooth", df["temp_internal_C"]).values
    ax_c.plot(t, tmp_s, color=COLORS["temp_meas"], linewidth=1.8)
    ax_c.axhline(0, color="#F72585", linewidth=0.7, linestyle="--", alpha=0.7)
    ax_c.set_title("Sıcaklık (°C)")
    ax_c.set_xlabel("Süre (dk)")
    ax_c.grid(True, linestyle=":")

    # Panel D — Batarya
    ax_d = fig.add_subplot(gs[1, 1])
    ax_d.plot(t, df["battery_V"].values, color="#70E000", linewidth=1.8)
    ax_d.axhline(3.3, color="#FF4D6D", linewidth=0.8, linestyle="--",
                 label="Safe-mode eşiği (3.3V)")
    ax_d.set_title("Batarya Gerilimi (V)")
    ax_d.set_xlabel("Süre (dk)")
    ax_d.legend(fontsize=8)
    ax_d.grid(True, linestyle=":")

    path = os.path.join(output_dir, "00_dashboard.png")
    plt.savefig(path, bbox_inches="tight", facecolor=COLORS["bg"])
    plt.close()
    print(f"[GRAFİK] Dashboard kaydedildi: {path}")
    return path


# ──────────────────────────────────────────────────────────────────
# 9. RAPOR YAZICI
# ──────────────────────────────────────────────────────────────────

def write_report(stats, interps, output_dir):
    """Tüm istatistikleri ve yorumları düz metin rapor olarak kaydet."""
    path = os.path.join(output_dir, "mission_report.txt")
    now  = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    lines = [
        "=" * 60,
        "  CUBESAT STRATOSFERİK GÖREV RAPORU",
        f"  Oluşturulma: {now}",
        "=" * 60,
        "",
        "── GÖREV İSTATİSTİKLERİ ──────────────────────────────",
        f"  Toplam örnek sayısı   : {stats['sample_count']}",
        f"  Görev süresi          : {stats['mission_duration_s']/60:.1f} dk",
        f"  Maksimum yükseklik    : {stats['max_altitude_m']:,.0f} m",
        f"  Ortalama tırmanma     : {stats['mean_climb_rate_ms']:.2f} m/s",
        f"  Max tırmanma hızı     : {stats['max_climb_rate_ms']:.2f} m/s",
        f"  Minimum sıcaklık      : {stats['min_temp_C']:.1f} °C",
        f"  Maksimum sıcaklık     : {stats['max_temp_C']:.1f} °C",
        f"  Lapse rate (ölçülen)  : {stats['lapse_rate_Cm']*1000:.2f} °C/km",
        f"  Lapse rate R²         : {stats['lapse_rate_R2']:.4f}",
        f"  Ortalama CO₂          : {stats['mean_co2_ppm']:.1f} ppm",
        f"  Min CO₂               : {stats['min_co2_ppm']:.1f} ppm",
        f"  Max CO₂               : {stats['max_co2_ppm']:.1f} ppm",
        "",
    ]

    for key, text in interps.items():
        lines.append("── " + key.upper().replace("_", " ") + " ──")
        lines.append(text)
        lines.append("")

    lines += [
        "=" * 60,
        "  GÖRSEL ÇIKTILAR",
        "  00_dashboard.png     — 4 panelli özet",
        "  01_altitude_time.png — Yükseklik vs Zaman",
        "  02_co2_altitude.png  — CO₂ vs Yükseklik",
        "  03_temp_altitude.png — Sıcaklık vs Yükseklik",
        "=" * 60,
    ]

    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))

    print(f"[RAPOR] Kaydedildi: {path}")
    return path


# ──────────────────────────────────────────────────────────────────
# 10. ANA FONKSİYON
# ──────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="CubeSat Yer İstasyonu — LoRa alıcı ve veri analizi"
    )
    parser.add_argument("--port",    default="/dev/ttyUSB0",
                        help="LoRa SPI cihaz portu (Raspberry Pi)")
    parser.add_argument("--csv",     default="ground_received.csv",
                        help="CSV dosya yolu")
    parser.add_argument("--outdir",  default="output",
                        help="Grafik çıktı klasörü")
    parser.add_argument("--no-lora", action="store_true",
                        help="LoRa donanımı olmadan sadece CSV analiz")
    parser.add_argument("--listen-minutes", type=float, default=0,
                        help="LoRa dinleme süresi (dakika, 0=süresiz)")
    args = parser.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    # ── LoRa Başlatma ve Veri Toplama ─────────────────────────────
    if not args.no_lora and LORA_AVAILABLE:
        station   = LoRaGroundStation()
        hw_ok     = station.init_hardware()
        csv_writer = CSVWriter(args.csv)

        if hw_ok:
            station.start_rx_thread()
            duration_s = args.listen_minutes * 60 if args.listen_minutes > 0 else float("inf")
            t_start    = time.time()

            print(f"\n[YER İSTASYONU] LoRa dinleniyor... (Ctrl+C ile dur)\n")
            try:
                while (time.time() - t_start) < duration_s:
                    new_records = station.get_all()
                    for rec in new_records:
                        csv_writer.write(rec)
                    time.sleep(1)
            except KeyboardInterrupt:
                print("\n[YER İSTASYONU] Kullanıcı tarafından durduruldu.")
            finally:
                station.stop()
        else:
            print("[UYARI] LoRa donanımı başlatılamadı, sadece CSV analiz yapılacak.")
    else:
        print("[BİLGİ] LoRa atlanıyor, mevcut CSV analiz edilecek.")

    # ── Veri Analizi ve Grafik Üretimi ────────────────────────────
    if not os.path.exists(args.csv):
        print(f"[HATA] CSV bulunamadı: {args.csv}")
        print("  Örnek veri üretiliyor (test modu)...")
        _generate_sample_csv(args.csv)

    df    = load_csv(args.csv)
    stats = compute_statistics(df)
    interps = generate_interpretation(stats, "all")

    # İstatistikleri konsola yazdır
    print("\n" + "=" * 50)
    print("  GÖREV İSTATİSTİKLERİ")
    print("=" * 50)
    for k, v in stats.items():
        if isinstance(v, float):
            print(f"  {k:<30}: {v:.4f}")
        else:
            print(f"  {k:<30}: {v}")

    # Grafikleri üret
    print("\n[GRAFİK] Grafikler oluşturuluyor...")
    setup_plot_style()
    plot_dashboard(df, stats, args.outdir)
    plot_altitude_vs_time(df, stats, interps, args.outdir)
    plot_co2_vs_altitude(df, stats, interps, args.outdir)
    plot_temperature_vs_altitude(df, stats, interps, args.outdir)
    write_report(stats, interps, args.outdir)

    print("\n[TAMAMLANDI] Tüm çıktılar:", args.outdir)


# ──────────────────────────────────────────────────────────────────
# ÖRNEK VERİ ÜRETICI (Test İçin)
# ──────────────────────────────────────────────────────────────────

def _generate_sample_csv(filepath):
    """
    Gerçek görev benzeri simüle edilmiş CSV verisi üret.
    Test ve geliştirme amaçlıdır.
    """
    import random
    random.seed(42)

    with open(filepath, "w", newline="") as f:
        f.write(CSV_HEADER)
        t      = 0
        t_surf = 20.0   # Yüzey sıcaklığı
        co2_s  = 415.0  # Yüzey CO2
        bat_v  = 4.1

        for i in range(1440):    # 4 saat × 10 saniyelik örnek
            alt    = calc_altitude(1013.25 * math.exp(-0.0001255 * t))
            pressure = 1013.25 * math.exp(-0.0001255 * t)
            temp   = t_surf + LAPSE_RATE * alt + random.gauss(0, 0.3)
            co2    = co2_s - 0.003 * alt + random.gauss(0, 1.5)
            co2    = max(390, co2)
            ch4    = 1.85 + random.gauss(0, 0.02)
            pm25   = max(0, 12 - 0.0005 * alt + random.gauss(0, 0.5))
            rh     = max(5, 60 - 0.002 * alt + random.gauss(0, 2))
            bat_v -= 0.00005
            heater = "1" if temp < 2 else "0"

            f.write(
                f"{t:.1f},41.0082,28.9784,"
                f"{pressure:.2f},{alt:.1f},"
                f"{temp:.2f},{temp - 5:.2f},"
                f"{co2:.1f},{ch4:.3f},"
                f"{pm25:.1f},0.00,{rh:.1f},"
                f"{bat_v:.3f},{heater}\n"
            )
            t += 10

    print(f"[TEST] Örnek CSV oluşturuldu: {filepath} (1440 satır)")


# ──────────────────────────────────────────────────────────────────
# GİRİŞ NOKTASI
# ──────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    main()
