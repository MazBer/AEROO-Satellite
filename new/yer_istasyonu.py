#!/usr/bin/env python3
"""
CubeSat Yer İstasyonu — Grafik + CSV Rapor
Kullanım:
  python yer_istasyonu.py --port COM3          # Windows
  python yer_istasyonu.py --port /dev/ttyUSB0  # Linux
  python yer_istasyonu.py --csv LOG_001.CSV    # Offline CSV analizi
"""

import argparse
import csv
import os
import struct
import sys
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
import numpy as np

try:
    import serial
    SERIAL_OK = True
except ImportError:
    SERIAL_OK = False
    print("[UYARI] pyserial kurulu degil. Sadece CSV modu calisir.")

# ── RENK PALETİ ───────────────────────────────────────────────
COLORS = {
    "bg":       "#0D1117",
    "panel":    "#161B22",
    "border":   "#30363D",
    "text":     "#E6EDF3",
    "grid":     "#21262D",
    "blue":     "#58A6FF",
    "green":    "#3FB950",
    "yellow":   "#D29922",
    "red":      "#F85149",
    "purple":   "#BC8CFF",
    "orange":   "#FFA657",
    "cyan":     "#39D353",
    "pink":     "#FF7B72",
}

# ── VERİ TAMPONU ─────────────────────────────────────────────
MAX_POINTS = 300

class DataBuffer:
    def __init__(self):
        self.lock = threading.Lock()
        self.t          = deque(maxlen=MAX_POINTS)
        self.pressure   = deque(maxlen=MAX_POINTS)
        self.bmp_temp   = deque(maxlen=MAX_POINTS)
        self.altitude   = deque(maxlen=MAX_POINTS)
        self.gps_alt    = deque(maxlen=MAX_POINTS)
        self.humidity   = deque(maxlen=MAX_POINTS)
        self.sht_temp   = deque(maxlen=MAX_POINTS)
        self.co2        = deque(maxlen=MAX_POINTS)
        self.mq7_ppm    = deque(maxlen=MAX_POINTS)
        self.mq135_aqi  = deque(maxlen=MAX_POINTS)
        self.mq131_ppb  = deque(maxlen=MAX_POINTS)
        self.ntc1       = deque(maxlen=MAX_POINTS)
        self.ntc2       = deque(maxlen=MAX_POINTS)
        self.accel_x    = deque(maxlen=MAX_POINTS)
        self.accel_y    = deque(maxlen=MAX_POINTS)
        self.accel_z    = deque(maxlen=MAX_POINTS)
        self.heater1    = deque(maxlen=MAX_POINTS)
        self.heater2    = deque(maxlen=MAX_POINTS)
        self.heater3    = deque(maxlen=MAX_POINTS)
        self.gps_lat    = []
        self.gps_lon    = []
        self.row_count  = 0

    def push_csv_row(self, row):
        with self.lock:
            try:
                ms = float(row["timestamp_ms"]) / 1000.0
                self.t.append(ms)
                self.pressure.append(float(row["pressure_hPa"]))
                self.bmp_temp.append(float(row["bmp_temp_C"]))
                self.altitude.append(float(row["baro_alt_m"]))
                self.gps_alt.append(float(row["gps_alt_m"]))
                self.humidity.append(float(row["humidity_pct"]))
                self.sht_temp.append(float(row["sht_temp_C"]))
                self.co2.append(float(row["co2_ppm"]))
                self.mq7_ppm.append(float(row["mq7_co_ppm"]))
                self.mq135_aqi.append(float(row["mq135_aqi"]))
                self.mq131_ppb.append(float(row["mq131_o3_ppb"]))
                self.ntc1.append(float(row["ntc1_temp_C"]))
                self.ntc2.append(float(row["ntc2_temp_C"]))
                self.accel_x.append(float(row["accel_x"]))
                self.accel_y.append(float(row["accel_y"]))
                self.accel_z.append(float(row["accel_z"]))
                self.heater1.append(float(row["heater1_duty"]))
                self.heater2.append(float(row["heater2_duty"]))
                self.heater3.append(float(row["heater3_duty"]))
                lat = float(row["gps_lat"])
                lon = float(row["gps_lon"])
                if lat != 0.0 and lon != 0.0:
                    self.gps_lat.append(lat)
                    self.gps_lon.append(lon)
                self.row_count += 1
            except (ValueError, KeyError):
                pass

buf = DataBuffer()

# ── GRAFİK KURULUM ────────────────────────────────────────────
plt.style.use("dark_background")
fig = plt.figure(figsize=(20, 12), facecolor=COLORS["bg"])
fig.canvas.manager.set_window_title("CubeSat Yer İstasyonu — Canlı Gösterge")

gs = gridspec.GridSpec(3, 4, figure=fig,
                       hspace=0.45, wspace=0.35,
                       left=0.05, right=0.97,
                       top=0.93, bottom=0.06)

def make_ax(row, col, title, ylabel, color):
    ax = fig.add_subplot(gs[row, col])
    ax.set_facecolor(COLORS["panel"])
    ax.set_title(title, color=COLORS["text"], fontsize=9, fontweight="bold", pad=4)
    ax.set_ylabel(ylabel, color=color, fontsize=8)
    ax.tick_params(colors=COLORS["text"], labelsize=7)
    ax.grid(True, color=COLORS["grid"], linewidth=0.5, alpha=0.7)
    for spine in ax.spines.values():
        spine.set_edgecolor(COLORS["border"])
    return ax

ax_alt   = make_ax(0, 0, "📍 Yükseklik",        "m",    COLORS["blue"])
ax_temp  = make_ax(0, 1, "🌡 Sıcaklık",          "°C",   COLORS["orange"])
ax_press = make_ax(0, 2, "🔵 Basınç",            "hPa",  COLORS["cyan"])
ax_hum   = make_ax(0, 3, "💧 Nem",               "%RH",  COLORS["blue"])
ax_co2   = make_ax(1, 0, "🟢 CO2",               "ppm",  COLORS["green"])
ax_mq7   = make_ax(1, 1, "⚠ CO (MQ-7)",         "ppm",  COLORS["red"])
ax_mq135 = make_ax(1, 2, "🌫 Hava Kalitesi AQI", "AQI",  COLORS["yellow"])
ax_mq131 = make_ax(1, 3, "🟣 Ozon O3 (MQ-131)",  "ppb",  COLORS["purple"])
ax_imu   = make_ax(2, 0, "📐 İvme (IMU)",        "m/s²", COLORS["pink"])
ax_ntc   = make_ax(2, 1, "🔴 NTC Sıcaklıklar",   "°C",   COLORS["red"])
ax_heat  = make_ax(2, 2, "🔥 Isıtıcı Duty",      "0-255",COLORS["orange"])
ax_gps   = make_ax(2, 3, "🗺 GPS İz",            "Enlem",COLORS["green"])

# Başlık
title_text = fig.text(0.5, 0.97, "🛰  CubeSat Stratosferik Balon — Canlı Veri Göstergesi",
                      ha="center", va="top", fontsize=13, fontweight="bold",
                      color=COLORS["text"])
status_text = fig.text(0.5, 0.005, "Bekleniyor...",
                       ha="center", va="bottom", fontsize=8,
                       color=COLORS["green"])

def update_plot(frame):
    with buf.lock:
        t_arr    = list(buf.t)
        if len(t_arr) < 2:
            return

        def clr(ax, color, *ys, labels=None):
            ax.cla()
            ax.set_facecolor(COLORS["panel"])
            ax.grid(True, color=COLORS["grid"], linewidth=0.5, alpha=0.7)
            for spine in ax.spines.values():
                spine.set_edgecolor(COLORS["border"])
            ax.tick_params(colors=COLORS["text"], labelsize=7)
            colors = [color] if not isinstance(color, list) else color
            for i, y in enumerate(ys):
                c = colors[i % len(colors)]
                lbl = labels[i] if labels else None
                ax.plot(t_arr[-len(y):], list(y)[-len(t_arr):],
                        color=c, linewidth=1.2, label=lbl)
            if labels:
                ax.legend(fontsize=7, loc="upper left",
                          facecolor=COLORS["panel"], edgecolor=COLORS["border"],
                          labelcolor=COLORS["text"])

        clr(ax_alt,   [COLORS["blue"], COLORS["cyan"]],
            buf.altitude, buf.gps_alt,
            labels=["Baro Alt", "GPS Alt"])
        ax_alt.set_ylabel("m", color=COLORS["blue"], fontsize=8)
        ax_alt.set_title("📍 Yükseklik", color=COLORS["text"], fontsize=9, fontweight="bold")

        clr(ax_temp,  [COLORS["orange"], COLORS["red"], COLORS["yellow"]],
            buf.bmp_temp, buf.ntc1, buf.sht_temp,
            labels=["BMP280", "NTC1-Bat", "SHT30"])
        ax_temp.set_title("🌡 Sıcaklık", color=COLORS["text"], fontsize=9, fontweight="bold")

        clr(ax_press, COLORS["cyan"], buf.pressure)
        ax_press.set_title("🔵 Basınç", color=COLORS["text"], fontsize=9, fontweight="bold")

        clr(ax_hum,   COLORS["blue"], buf.humidity)
        ax_hum.set_title("💧 Nem", color=COLORS["text"], fontsize=9, fontweight="bold")

        clr(ax_co2,   COLORS["green"], buf.co2)
        ax_co2.set_title("🟢 CO2", color=COLORS["text"], fontsize=9, fontweight="bold")
        if buf.co2:
            last = list(buf.co2)[-1]
            clr_flag = COLORS["red"] if last > 1000 else COLORS["green"]
            ax_co2.set_ylabel(f"ppm  ({last:.0f})", color=clr_flag, fontsize=8)

        clr(ax_mq7,   COLORS["red"], buf.mq7_ppm)
        ax_mq7.set_title("⚠ CO (MQ-7)", color=COLORS["text"], fontsize=9, fontweight="bold")

        clr(ax_mq135, COLORS["yellow"], buf.mq135_aqi)
        ax_mq135.set_title("🌫 Hava Kalitesi AQI", color=COLORS["text"], fontsize=9, fontweight="bold")

        clr(ax_mq131, COLORS["purple"], buf.mq131_ppb)
        ax_mq131.set_title("🟣 Ozon O3", color=COLORS["text"], fontsize=9, fontweight="bold")

        clr(ax_imu,   [COLORS["pink"], COLORS["blue"], COLORS["green"]],
            buf.accel_x, buf.accel_y, buf.accel_z,
            labels=["X", "Y", "Z"])
        ax_imu.set_title("📐 İvme (IMU)", color=COLORS["text"], fontsize=9, fontweight="bold")

        clr(ax_ntc,   [COLORS["red"], COLORS["orange"]],
            buf.ntc1, buf.ntc2,
            labels=["NTC1-Bat", "NTC2-Kart"])
        ax_ntc.set_title("🔴 NTC Sıcaklıklar", color=COLORS["text"], fontsize=9, fontweight="bold")

        clr(ax_heat,  [COLORS["orange"], COLORS["yellow"], COLORS["red"]],
            buf.heater1, buf.heater2, buf.heater3,
            labels=["H1", "H2", "H3"])
        ax_heat.set_title("🔥 Isıtıcı Duty", color=COLORS["text"], fontsize=9, fontweight="bold")

        # GPS iz
        ax_gps.cla()
        ax_gps.set_facecolor(COLORS["panel"])
        ax_gps.grid(True, color=COLORS["grid"], linewidth=0.5, alpha=0.7)
        for spine in ax_gps.spines.values():
            spine.set_edgecolor(COLORS["border"])
        ax_gps.tick_params(colors=COLORS["text"], labelsize=7)
        if len(buf.gps_lat) > 1:
            ax_gps.plot(buf.gps_lon, buf.gps_lat,
                        color=COLORS["green"], linewidth=1.0, alpha=0.7)
            ax_gps.plot(buf.gps_lon[-1], buf.gps_lat[-1],
                        "o", color=COLORS["yellow"], markersize=6)
        ax_gps.set_title("🗺 GPS İz", color=COLORS["text"], fontsize=9, fontweight="bold")
        ax_gps.set_xlabel("Boylam", color=COLORS["text"], fontsize=7)
        ax_gps.set_ylabel("Enlem", color=COLORS["green"], fontsize=7)

        # Durum çubuğu
        if t_arr:
            elapsed = t_arr[-1] - t_arr[0]
            status_text.set_text(
                f"Satır: {buf.row_count}  |  Süre: {elapsed:.0f}s  |"
                f"  Son GPS: {list(buf.gps_alt)[-1] if buf.gps_alt else 0:.0f}m  |"
                f"  CO2: {list(buf.co2)[-1] if buf.co2 else 0:.0f}ppm  |"
                f"  NTC1: {list(buf.ntc1)[-1] if buf.ntc1 else 0:.1f}°C"
            )

# ── CSV OKUYUCU (Offline veya Canlı) ─────────────────────────
def csv_reader_thread(csv_path, live=False):
    """CSV dosyasını okur. live=True ise sonuna kadar okuyup bekler."""
    path = Path(csv_path)
    processed_lines = 0

    while True:
        if not path.exists():
            time.sleep(0.5)
            continue
        with open(path, newline="") as f:
            reader = csv.DictReader(f)
            rows = list(reader)

        for row in rows[processed_lines:]:
            buf.push_csv_row(row)
            processed_lines += 1
            if live:
                time.sleep(0.01)  # Canlı modda biraz yavaşlat

        if not live:
            break
        time.sleep(1.0)  # Yeni satır var mı diye bekle

# ── RAPOR ÜRETICI ─────────────────────────────────────────────
def generate_report(csv_path):
    """CSV'den özet rapor PNG üret."""
    path = Path(csv_path)
    if not path.exists():
        print(f"[HATA] {csv_path} bulunamadi.")
        return

    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    if not rows:
        print("[HATA] CSV bos.")
        return

    # Veriyi yükle
    for row in rows:
        buf.push_csv_row(row)

    def safe_list(d):
        return list(d) if d else []

    report_fig, axes = plt.subplots(4, 3, figsize=(18, 14),
                                     facecolor=COLORS["bg"])
    report_fig.suptitle(f"CubeSat Uçuş Raporu — {path.name}",
                        color=COLORS["text"], fontsize=14, fontweight="bold", y=0.98)

    plot_defs = [
        (0,0, "Yükseklik (m)",        "m",    [safe_list(buf.altitude), safe_list(buf.gps_alt)],
         [COLORS["blue"], COLORS["cyan"]], ["Baro", "GPS"]),
        (0,1, "Basınç (hPa)",         "hPa",  [safe_list(buf.pressure)],
         [COLORS["cyan"]], None),
        (0,2, "CO2 (ppm)",            "ppm",  [safe_list(buf.co2)],
         [COLORS["green"]], None),
        (1,0, "Sıcaklık (°C)",        "°C",   [safe_list(buf.bmp_temp), safe_list(buf.ntc1), safe_list(buf.sht_temp)],
         [COLORS["orange"], COLORS["red"], COLORS["yellow"]], ["BMP", "NTC1", "SHT"]),
        (1,1, "Nem (%RH)",            "%",    [safe_list(buf.humidity)],
         [COLORS["blue"]], None),
        (1,2, "CO ppm (MQ-7)",        "ppm",  [safe_list(buf.mq7_ppm)],
         [COLORS["red"]], None),
        (2,0, "Hava Kalitesi (MQ-135)","AQI", [safe_list(buf.mq135_aqi)],
         [COLORS["yellow"]], None),
        (2,1, "Ozon ppb (MQ-131)",    "ppb",  [safe_list(buf.mq131_ppb)],
         [COLORS["purple"]], None),
        (2,2, "İvme m/s² (IMU)",      "m/s²", [safe_list(buf.accel_x), safe_list(buf.accel_y), safe_list(buf.accel_z)],
         [COLORS["pink"], COLORS["blue"], COLORS["green"]], ["X","Y","Z"]),
        (3,0, "NTC Sıcaklıklar (°C)", "°C",   [safe_list(buf.ntc1), safe_list(buf.ntc2)],
         [COLORS["red"], COLORS["orange"]], ["NTC1","NTC2"]),
        (3,1, "Isıtıcı Duty (0-255)", "",     [safe_list(buf.heater1), safe_list(buf.heater2), safe_list(buf.heater3)],
         [COLORS["orange"], COLORS["yellow"], COLORS["red"]], ["H1","H2","H3"]),
    ]

    t_arr = safe_list(buf.t)

    for (r, c, title, ylabel, ys, colors, labels) in plot_defs:
        ax = axes[r][c]
        ax.set_facecolor(COLORS["panel"])
        ax.set_title(title, color=COLORS["text"], fontsize=9, fontweight="bold")
        ax.set_ylabel(ylabel, color=COLORS["text"], fontsize=8)
        ax.tick_params(colors=COLORS["text"], labelsize=7)
        ax.grid(True, color=COLORS["grid"], linewidth=0.5, alpha=0.6)
        for spine in ax.spines.values():
            spine.set_edgecolor(COLORS["border"])
        for i, y in enumerate(ys):
            x = t_arr[-len(y):]
            lbl = labels[i] if labels else None
            ax.plot(x, y, color=colors[i % len(colors)], linewidth=1.0, label=lbl)
        if labels:
            ax.legend(fontsize=7, facecolor=COLORS["panel"],
                      edgecolor=COLORS["border"], labelcolor=COLORS["text"])

    # GPS iz
    ax_g = axes[3][2]
    ax_g.set_facecolor(COLORS["panel"])
    ax_g.set_title("GPS İz", color=COLORS["text"], fontsize=9, fontweight="bold")
    ax_g.tick_params(colors=COLORS["text"], labelsize=7)
    ax_g.grid(True, color=COLORS["grid"], linewidth=0.5, alpha=0.6)
    for spine in ax_g.spines.values():
        spine.set_edgecolor(COLORS["border"])
    if len(buf.gps_lat) > 1:
        ax_g.plot(buf.gps_lon, buf.gps_lat,
                  color=COLORS["green"], linewidth=1.2)
        ax_g.plot(buf.gps_lon[0], buf.gps_lat[0],
                  "^", color=COLORS["green"], markersize=8, label="Kalkış")
        ax_g.plot(buf.gps_lon[-1], buf.gps_lat[-1],
                  "v", color=COLORS["red"], markersize=8, label="İniş")
        ax_g.legend(fontsize=7, facecolor=COLORS["panel"],
                    edgecolor=COLORS["border"], labelcolor=COLORS["text"])

    plt.tight_layout(rect=[0, 0, 1, 0.97])

    out_png = path.with_suffix(".png")
    report_fig.savefig(out_png, dpi=150, facecolor=COLORS["bg"], bbox_inches="tight")
    print(f"[OK] Rapor kaydedildi: {out_png}")
    plt.show()

# ── ANA PROGRAM ───────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="CubeSat Yer İstasyonu")
    parser.add_argument("--csv",    type=str, help="Offline CSV dosyası")
    parser.add_argument("--port",   type=str, help="Seri port (LoRa yer istasyonu)")
    parser.add_argument("--baud",   type=int, default=9600)
    parser.add_argument("--report", action="store_true",
                        help="CSV'den statik rapor PNG üret ve çık")
    args = parser.parse_args()

    if args.report and args.csv:
        generate_report(args.csv)
        return

    if args.csv:
        print(f"[CSV] {args.csv} okunuyor...")
        t = threading.Thread(target=csv_reader_thread,
                             args=(args.csv, True), daemon=True)
        t.start()
    elif args.port:
        if not SERIAL_OK:
            print("[HATA] pyserial gerekli: pip install pyserial")
            sys.exit(1)
        print(f"[SERIAL] {args.port}:{args.baud} bekleniyor...")
        # Seri port LoRa binary alımı — gelecek geliştirme
        # Şimdilik SD karttan kopyalanan CSV kullan
    else:
        print("Kullanım:")
        print("  python yer_istasyonu.py --csv LOG_001.CSV")
        print("  python yer_istasyonu.py --csv LOG_001.CSV --report")
        print("  python yer_istasyonu.py --port COM3")
        sys.exit(0)

    ani = animation.FuncAnimation(fig, update_plot, interval=1000, cache_frame_data=False)
    plt.show()

if __name__ == "__main__":
    main()
