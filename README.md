# AEROO-Satellite

# KISIM 1 — ONBOARD FLIGHT SYSTEM: Adım Adım Detaylı Yapım Rehberi

---

## 🔩 AŞAMA 0 — DONANIM BAĞLANTILARI

**Güç Hattı**
- 3.7V Li-ion batarya → Feather M0'ın BAT pinine bağlanır
- USB şarj devresi Feather üzerinde dahilidir
- Voltaj bölücü devre (2× 100kΩ direnç) batarya hattına seri bağlanır, orta nokta A5 analog pinine gider → batarya voltajı ölçümü için

**I2C Hattı (SDA / SCL)**
- BMP280 (basınç + sıcaklık) → adres 0x76
- SCD30 (CO2 + nem) → adres 0x61
- Her iki sensör aynı I2C hattına paralel bağlanır
- Hat üzerine 4.7kΩ pull-up dirençler eklenir (3.3V'a)

**SPI Hattı (SCK / MOSI / MISO)**
- LoRa RFM9x modülü → CS pini D5, RESET pini D6
- MicroSD kart modülü → CS pini D10
- İki cihaz aynı SPI hattını paylaşır, CS pinleri ayrıdır

**UART Hattı (TX / RX)**
- PM2.5 toz sensörü (PMS5003 veya benzeri) → 9600 baud
- Feather'ın TX→Sensör RX, Feather RX→Sensör TX çapraz bağlanır

**Analog Hat**
- CH4 sensörü çıkış pini → A1 analog girişi
- Sensörün ısınma direnci için ayrı 5V hat veya güçlü 3.3V hattı kullanılır

**Isıtıcı Devresi**
- 7W ısıtıcı eleman doğrudan MCU'ya bağlanamaz
- N-kanal MOSFET (örn. IRLZ44N) gate pini → D11
- MOSFET drain → ısıtıcı negatif ucu
- MOSFET source → GND
- Isıtıcı pozitif ucu → batarya pozitif hattı (güç katmanından beslenir)
- Gate direnci 100Ω, flyback diyot ısıtıcıya paralel eklenir

---

## 🔧 AŞAMA 1 — SİSTEM BAŞLATMA (Initialization)

**MCU Saati ve GPIO**
- Feather M0, 48 MHz ARM Cortex-M0+ işlemci ile başlar
- Tüm kullanılacak digital pinler önce OUTPUT veya INPUT olarak tanımlanır
- CS pinleri başlangıçta HIGH (pasif) konumuna alınır → SPI çakışmasını önler

**SPI, I2C, UART, ADC Başlatma**
- Önce SPI başlatılır çünkü hem LoRa hem SD bu hattı kullanır
- I2C başlatılır, BMP280 ve SCD30 adresleri taranır (scan)
- UART başlatılır, PM2.5 sensörü baud rate 9600 ile açılır
- ADC çözünürlüğü 16-bit olarak ayarlanır

**SD Kart Mount**
- SPI üzerinden SD kart init edilir
- FAT32 dosya sistemi mount edilir
- Başarısız olursa error_flag["sd"] = True atanır, görev logsuz devam edebilir
- Başarılıysa flight_log.csv yoksa oluşturulur ve başlık satırı yazılır

**LoRa Modülü Başlatma**
- SPI üzerinden RFM9x init edilir
- Frekans: 868 MHz (EU) veya 915 MHz (US) bölgeye göre seçilir
- Spreading Factor, Bandwidth, Coding Rate, TX Power ayarlanır
- Başarısız olursa error_flag["lora"] = True atanır

**Sensör Bağlantı Testi**
- Her sensöre tek okuma komutu gönderilir
- Yanıt alınamazsa ilgili error_flag True yapılır ve durum loglanır
- SCD30 CO2 sensörü için 30 saniye ısınma beklenir (kimyasal reaksiyon stabilitesi)
- BMP280'den P0 referans basınç değeri alınır (deniz seviyesi referansı)

---

## 📡 AŞAMA 2 — COMPONENT DATA (Sensörlerden Ham Veri Alma)

**BMP280 — Basınç ve Sıcaklık**
- I2C üzerinden oversampling ayarı yapılır (×8 veya ×16 hassasiyet için)
- pressure ve temperature register'ları okunur
- Ham değerler doğrudan Pascal ve °C cinsinden döner (sürücü dahili dönüşüm yapar)
- Okuma başarısız olursa son geçerli değer korunur (stale value stratejisi)

**SCD30 — CO2 ve Nem**
- data_available bayrağı True olana kadar beklenir (sensör kendi ölçüm döngüsündedir)
- CO2 ppm, sıcaklık ve bağıl nem üç ayrı register'dan okunur
- Sensör 0–10.000 ppm aralığında ölçüm yapar
- Ölçüm aralığı 2 saniyeye ayarlanmıştır

**CH4 — Analog ADC**
- Sensörün çıkış voltajı A1 pini üzerinden 16-bit ADC ile okunur
- Ham sayısal değer (0–65535) volt cinsine dönüştürülür:
  - V_out = (ADC_ham / 65535) × 3.3

**PM2.5 — UART**
- Sensör her 1 saniyede otomatik paket gönderir
- 32 byte'lık sabit formatlı paketten PM1.0, PM2.5, PM10 değerleri çekilir
- Paket başlangıç byte'ları (0x42, 0x4D) kontrol edilerek geçersiz paketler atlanır

**Batarya Voltajı**
- A5 pininden ADC okuması yapılır
- Voltaj bölücü oranı uygulanarak gerçek batarya voltajı hesaplanır:
  - V_bat = (ADC_ham / 65535) × 3.3 × 2.0

---

## 🧮 AŞAMA 3 — MCU COMPUTE (Hesaplama ve Sinyal İşleme)

**Hareketli Ortalama Filtresi**
- Her sensör kanalı için ayrı 10 elemanlı tampon (buffer) tutulur
- Her yeni okuma tampona eklenir, en eski değer çıkarılır
- Çıkış değeri tampondaki tüm elemanların ortalamasıdır:
  - y = (x₁ + x₂ + ... + x₁₀) / 10
- Bu işlem yüksek frekanslı elektriksel gürültüyü bastırır

**Aykırı Değer (Outlier) Tespiti**
- Tamponun ortalaması (μ) ve standart sapması (σ) hesaplanır
- Yeni gelen değer |x − μ| > 3σ koşulunu sağlıyorsa atılır
- Atılan değer yerine mevcut ortalama kullanılır
- Atılma olayı SD'ye hata notu olarak kaydedilir

**CO2 ve CH4 Kalibrasyon Zinciri**
- Adım 1 — Power-law model uygulanır:
  - C = a × (V_out / V_ref)^b
- Adım 2 — Temiz hava kalibrasyonundan belirlenen offset çıkarılır:
  - C_düz = C − offset
- Adım 3 — Sıcaklık kompanzasyonu uygulanır (soğukta duyarlılık düşer):
  - C_son = C_düz × (1 + 0.02 × (T_ölçülen − 25))

**Yükseklik Hesabı**
- Filtrelenmiş basınç değeri barometrik formüle sokulur:
  - h = 44330 × [1 − (P / 1013.25)^0.1903]
- Her örnekleme adımında yükseklik farkı ve zaman farkından tırmanma hızı hesaplanır:
  - v = (h₂ − h₁) / (t₂ − t₁)

**Termal Kontrol Kararı**
- İç sıcaklık 0°C altına düşerse MOSFET gate pini HIGH → ısıtıcı AÇIK
- İç sıcaklık 5°C üzerine çıkarsa MOSFET gate pini LOW → ısıtıcı KAPALI
- Isıtıcı durumu (0 veya 1) CSV kaydına eklenir

**Safe-Mode Kontrolü**
- Batarya voltajı < 3.3V → ısıtıcı zorla kapatılır, örnekleme aralığı 60 saniyeye çıkar
- İç sıcaklık < −5°C → sensörler devre dışı bırakılır, sadece konum ve voltaj loglanır
- SD yazma hatası üst üste 3 kez gelirse SD yeniden mount edilmeye çalışılır

---

## 💾 AŞAMA 4 — SD WRITE (SD Karta Veri Yazma)

**Dosya Yazma Stratejisi**
- Dosya her yazımda append (ekleme) modunda açılır
- Tek CSV satırı yazılır
- Buffer flush edilir (tampon belleği diske aktarılır)
- Dosya kapatılır → ani güç kesilmesine karşı veri bütünlüğü sağlanır

**CSV Satır Formatı**
Sırasıyla şu alanlar yazılır:
```
Zaman(s), Enlem, Boylam, Basınç(hPa), Yükseklik(m),
İç_Sıcaklık(°C), Dış_Sıcaklık(°C), CO2(ppm), CH4(ppm),
PM2.5(μg/m³), UV_İndeks, Nem(%), Batarya(V), Isıtıcı(0/1)
```

**Hata Yönetimi**
- Yazma başarısız olursa 3 kez yeniden denenir
- 3 denemede de başarısız olursa SD yeniden mount edilir
- SD tamamen erişilemezse görev logsuz devam eder, sadece LoRa iletimi sürer

---

## 📤 AŞAMA 5 — LoRa TX (Paket Oluşturma ve Gönderme)

**Paket Yapısı**
Her 30 saniyede bir aşağıdaki binary paket oluşturulur:
```
[0xAB 0xCD]  → Senkronizasyon header (2 byte)
[Zaman]      → 4 byte unsigned int (saniye)
[Yükseklik]  → 4 byte float (metre)
[Sıcaklık]   → 2 byte signed int (°C × 10)
[CO2]        → 2 byte unsigned int (ppm)
[CH4]        → 2 byte unsigned int (ppm × 100)
[PM2.5]      → 2 byte unsigned int (μg × 10)
[Batarya]    → 2 byte unsigned int (V × 100)
[Bayrak]     → 1 byte (bit0: ısıtıcı durumu)
[CRC]        → 2 byte (tüm payload byte'larının XOR'u)
              Toplam: ~28 byte
```

**Neden Binary?**
- ASCII CSV formatı aynı veri için ~120 byte yer tutar
- Binary 28 byte ile TX süresi 4× kısalır → enerji tasarrufu
- 28 byte @ SF10 → yaklaşık 60ms havada kalma süresi
- Her iletim: 2W × 0.06s = 0.12 Joule

**Gönderim Enerjisi (4 Saatlik Görev)**
- 14400s / 30s = 480 iletim
- 480 × 0.12J = 57.6J → 0.016 Wh (batarya kapasitesinin %0.04'ü)

---

## 📶 AŞAMA 6 — RF (Radyo Frekans İletim Ortamı)

**LoRa Modülasyon Parametreleri**
- Spreading Factor 10 → her sembol 1024 chip'e yayılır → gürültü bağışıklığı artar
- Bandwidth 125 kHz → dar bant → daha uzun menzil, daha yavaş veri hızı
- Coding Rate 4/5 → her 4 veri bitine 1 hata düzeltme biti eklenir
- TX Gücü 13 dBm (~20mW) → stratosfer için yeterli

**Beklenen Menzil**
- Stratosfer yüksekliğinde görüş hattı (LOS) açık olduğundan
- SF10, 125kHz ile teorik menzil > 50 km
- Yere yakın ortamda bina engeli olmadığı için pratik menzil 30–50 km

**Zayıflama Faktörleri**
- Anten yönelimi (polarizasyon uyumsuzluğu) → 3 dB kayıp
- Atmosferik nem ve yoğunluk farkı → 1–2 dB kayıp
- Paket kayıp olasılığı SF10'da çok düşük (~%1 altı)

---

## 📥 AŞAMA 7 — LoRa RX (Yer İstasyonundan Komut Alma)

**Dinleme Stratejisi**
- TX gönderdikten hemen sonra 500ms dinleme penceresi açılır
- rfm9x.receive(timeout=0.5) komutu ile bloke olmayan dinleme yapılır
- 500ms içinde paket gelmezse ana döngü devam eder → görev geciktirilmez

**Gelen Paket Doğrulama**
- İlk byte 0xAC değilse paket tanınmaz, atılır
- İkinci byte komut tipidir

**Komut Tipleri**
| Komut Byte | Anlamı | MCU'nun Tepkisi |
|---|---|---|
| 0x01 | ACK — iletim başarılı | Kayıt tutulur, işlem yok |
| 0x02 | Safe-mode komutu | Isıtıcı kapanır, aralık 60s'ye çıkar |
| 0x03 | Örnekleme aralığı değiştir | 3. byte yeni aralık (saniye) olarak uygulanır |

**RSSI Takibi**
- Her alınan pakette rfm9x.last_rssi değeri okunur
- Bu değer sinyal gücünü dBm cinsinden verir
- −90 dBm altına düşerse anten bağlantısı veya menzil sorunu olduğu loglanır
