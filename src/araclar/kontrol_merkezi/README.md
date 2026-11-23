# 🖥️ kontrol_merkezi — Robotaksi Kontrol Merkezi

Araca oturan kişinin terminal açmadan LiDAR sürücüsünü, engel algılamayı, kayıt almayı ve oynatmayı
tek pencereden yönetmesi için. Türkçe ayar paneli ve kayıt/oynatma betikleri de bu pakette.

Robotaksi çalışma alanının `araclar/` katmanındaki paket ([çalışma alanı rehberi](../../../README.md)).
Yönettiği algılama düğümleri [`lidar_algilama`](../../algilama/lidar_algilama/README.md) paketindedir.

---

## 🖥️ Kontrol Merkezi

Araca oturan kişinin terminal açıp komut yazmadan her şeyi tek pencereden yönetmesi için.

```bash
ros2 run kontrol_merkezi kontrol_merkezi
```

Masaüstü simgesi için programda **Ayarlar → Masaüstü kısayolunu oluştur**'a basın; masaüstüne ve
uygulama menüsüne **Kontrol Merkezi** simgesi gelir (terminal penceresi açmadan çalışır).

**Ana ekran** LiDAR'dan kablo üzerinden paket gelip gelmediğini (sürücüyü açmadan), diskte kaç saatlik
kayda yer kaldığını, son kullanılan ayar profilini ve kayıtları gösterir. Dört bölümden birine tıklayın:

| Bölüm | Arkada açılanlar | Neler yapılır |
|---|---|---|
| 🚧 **Engel Algılama** | LiDAR sürücüsü, `engel_node`, `nesne_node`, RViz | Canlı karar, yukarıdan bakış çizimi (nesne kutuları, gövde kutusu), canlı nesne tablosu, çalışırken değişen **tüm** ayarlar, profiller, sayfadan çıkmadan kayıt |
| 📡 **LiDAR'ı İzle** | LiDAR sürücüsü, RViz (sadece ham bulut) | Saniyedeki tur, tur başına nokta, kopan turlar, sürücü ayarları (sadece gösterilir), kayıt |
| ⏺ **Kayıt Al** | LiDAR sürücüsü (+ istenirse RViz) | İsim, süre, sıkıştırma; geçen süre / boyut / tur; kayıt listesi: oynat, adını değiştir, çöp kutusuna at |
| ▶ **Kayıt Oynat** | Kayıt oynatıcı, `engel_node`, `nesne_node`, RViz | Duraklat / devam, tek tur ilerlet, ilerleme çubuğuyla atla, hız 0.1×–2×, döngü; Engel Algılama'daki ayar paneli |

- **Sadece fark açılır/kapanır.** LiDAR'ı İzle'den Engel Algılama'ya geçerken sürücü açık kalır. Ayarı
  değişen program (RViz aç/kapat, nesne algılama aç/kapat, oynatıcının döngü ayarı, başka kayıt seçmek)
  kendiliğinden yeniden başlatılır; pencere hiç kapanmaz.
- **⌂ Ana ekran** o bölümün açtığı her şeyi kapatır. Pencere kapatılınca programlar düzgünce (Ctrl+C ile)
  kapatılır, kayıt dosyası yarım kalmaz. Arayüz çökse bile açtığı programlar da kapanır.
- Kayıt sürerken sürücünün açık kaldığı bölümler (İzle ↔ Engel Algılama ↔ Kayıt Al) arasında geçilebilir.
- Her programın terminal çıktısı alttaki **Program çıktıları** bölümündedir. Bir program çökerse sayfanın
  üstünde kırmızı bant ve **Yeniden başlat** düğmesi çıkar.
- Eski kısayollardan ya da bir terminalden açılmış sürücü / node / oynatıcı varsa uyarır ve kapatmayı önerir
  (iki sürücü aynı LiDAR portunu kullanamaz, kayıt ile canlı veri karışır).
- LiDAR sürücüsünün ayarları (`src/surucular/rslidar_sdk/config/config.yaml`) **değiştirilmez**, sadece gösterilir.
- **⚙ Ayarlar:** kayıt klasörü, bölüme girince RViz açılsın mı, RViz bu pencerenin yanına yerleşsin mi
  (ekranı ikiye böler), nesne algılama varsayılanı, koyu / açık tema, varsayılan sıkıştırma, masaüstü
  kısayolu. Dosyası: `~/.ros/kontrol_merkezi/arayuz.yaml`.

> Simgeden açılınca programın kendi çıktısı `~/.ros/kontrol_merkezi/kontrol_merkezi.log` dosyasına yazılır;
> açılırken hata verirse son satırlar bir pencerede gösterilir.

---

## 🚀 Çalıştırma

Her şey tek komutla (LiDAR sürücüsü + RViz + `engel_node` + `nesne_node` + ayar paneli):

```bash
ros2 launch kontrol_merkezi lidar_sistem.launch.py
```

| Argüman | Varsayılan | Açıklama |
|---|---|---|
| `surucu` | `true` | LiDAR sürücüsünü aç. Rosbag oynatırken `false` |
| `panel` | `true` | Ayar panelini aç |
| `nesne` | `true` | `nesne_node`'u (kutu / mesafe / takip) aç |

```bash
ros2 launch kontrol_merkezi lidar_sistem.launch.py surucu:=false
```

Tek başına ayar paneli (düğümler başka yerde açıkken):

```bash
ros2 run kontrol_merkezi ayar_paneli
```

Sadece algılama düğümleri için: [`lidar_algilama`](../../algilama/lidar_algilama/README.md#-çalıştırma).

---

## 🎞️ Kayıt alma ve oynatma (rosbag)

LiDAR'ı her seferinde kurmadan geliştirme yapmak için gerçek veriyi kaydedip tekrar oynatabilirsiniz.

**Kayıt** (önce sistem açık ve RViz'de noktalar görünür olmalı):

```bash
~/robotaksi_ws/install/kontrol_merkezi/lib/kontrol_merkezi/lidar_kayit.sh oda_yurume 60
```

- İsim verilmezse sorar, süre verilmezse `Ctrl+C`'ye kadar kaydeder.
- Kayıt `~/robotaksi_kayitlar/TARIH_SAAT_isim/` klasörüne, MCAP biçiminde, `zstd` sıkıştırmalı gider.
- Dakikada ~0.5–0.9 GB yer kaplar. Bilgisayar kasarsa: `SIKISTIRMA=yok lidar_kayit.sh ...`

**Oynatma** (LiDAR takılı olmasına gerek yok; canlı sürücü **kapalı** olmalı):

```bash
~/robotaksi_ws/install/kontrol_merkezi/lib/kontrol_merkezi/lidar_oynat.sh ~/robotaksi_kayitlar/2026-11-20_15-30-00_oda_yurume
```

- Klasör verilmezse seçme penceresi açılır. Üçüncü argüman hızdır: `... KAYIT 0.5` → yarı hız.
- RViz, node'lar ve panel açılır; kayıt sonsuz döngüde oynar. Durdurmak için `Ctrl+C`.

---

## 🎛️ Ayar paneli

Panel `rqt_reconfigure` yerine, ne değiştirdiğinizi görerek ayar yapmanız için yazıldı.

<p align="center">
  <img src="../../../docs/images/ayar_paneli.png" alt="Ayar paneli: canlı karar, yukarıdan bakış çizimi ve ayarlar" width="100%">
</p>

- **Sol taraf:** büyük renkli karar kutusu (DUR / DİKKAT / GÜVENLİ + mesafe), kararı veren noktanın
  açısı/mesafesi/yüksekliği ve yukarıdan bakış çizimi. Dış çember ölçeği altta yazar.
- **Çizim:** uyarı yelpazesi (sarı / kırmızı), kutulama alanı (mavi kesik çizgi), gövde kutusu (taralı)
  ve `nesne_node`'un bulduğu nesne kutuları. Fare tekerleğiyle yakınlaştırılır, çift tıkla eski ölçeğe döner.
- **Nesne tablosu:** takip numarası, etiket, mesafe, yön, hız, yaklaşma hızı, çarpışma süresi.
- **Sağ taraf:** iki node'un çalışırken değişebilen **tüm** ayarları 7 sekmede — *Uyarı alanı*, *Filtreler*,
  *Kutulama alanı*, *Zemin*, *Kümeleme*, *Etiket*, *Takip*. Her ayarda kaydırma çubuğu, klavyeyle yazılabilen
  değer kutusu, açıklama ve varsayılana döndürme düğmesi var. Birbiriyle çelişen değerler (ör. DUR ≥ DİKKAT)
  kırmızı gösterilir ve düzeltilene kadar gönderilmez.
- **Kayıtlı ayarlar:**
  - Uyguladığınız her değişiklik `~/.ros/kontrol_merkezi/son_ayarlar.yaml` dosyasına **otomatik**
    kaydedilir ve launch dosyası açılışta bunu varsayılanların üstüne yükler.
  - **Kaydet…** ile isimli profil oluşturun (`~/.ros/kontrol_merkezi/profiller/`), **Yükle** ile geri getirin.
  - **Dışa aktar / İçe aktar** ile ayarları başka bilgisayara taşıyın.
  - Kayıt dosyaları ROS parametre dosyası biçimindedir (ayarların anlamı: [`lidar_algilama` → Parametreler](../../algilama/lidar_algilama/README.md#️-parametreler)), `--params-file` ile doğrudan da kullanılabilir.
- Bir node kapalıysa onun sekmeleri kilitli kalır ve son kaydedilen değerleri gösterir; o sırada yüklenen
  profilin değerleri node açılınca uygulanır.
- Aynı panel Kontrol Merkezi'nin Engel Algılama ve Kayıt Oynat bölümlerinde de var.

---

## 🖱️ Masaüstü kısayolları (isteğe bağlı)

> Önerilen yol tek simgeli [Kontrol Merkezi](#️-kontrol-merkezi)'dir. Aşağıdaki betikler
> terminalden kullanmak isteyenler için duruyor.

`scripts/` altındaki betikler masaüstü simgesinden çift tıkla açılmak için hazırlandı
(`~/.bashrc`'yi okumadıkları için ROS ortamını ortak [`ros_ortami.sh`](../../ortak/robotaksi_baslat/scripts/ros_ortami.sh) üzerinden kurarlar; çalışma alanının yerini kendileri bulur):

| Betik | Ne yapar |
|---|---|
| `engel_sistemi_baslat.sh` | Tüm sistemi açar. Sürücü zaten açıksa ikinci kez açmaz |
| `lidar_kayit.sh` | LiDAR verisini `~/robotaksi_kayitlar/` altına kaydeder |
| `lidar_oynat.sh` | Bir kaydı RViz + node'larla birlikte, sonsuz döngüde oynatır |

Örnek kısayol (`~/Desktop/engel-algilama.desktop`):

```ini
[Desktop Entry]
Type=Application
Name=Engel Algılama
Comment=LiDAR sürücüsü + RViz + engel algılama ayar paneli
Exec=/home/KULLANICI/robotaksi_ws/install/kontrol_merkezi/lib/kontrol_merkezi/engel_sistemi_baslat.sh
Icon=/home/KULLANICI/robotaksi_ws/install/kontrol_merkezi/share/kontrol_merkezi/resource/kontrol_merkezi.svg
Terminal=true
Categories=Science;
```

Dosyayı oluşturduktan sonra sağ tık → **Başlatmaya izin ver**.

---

## 🩺 Sorun giderme

| Belirti | Olası neden / çözüm |
|---|---|
| Kayıt oynatınca veri karışık | Canlı sürücü açık kalmış — ikisi aynı topic'e yayın yapar. Sürücüyü kapatın |
| Kontrol Merkezi simgeden açılmıyor | `~/.ros/kontrol_merkezi/kontrol_merkezi.log` dosyasına bakın; paket derlenmemiş olabilir |
| Kontrol Merkezi "başka yerden açılmış programlar" diyor | Eski kısayolla ya da terminalden açılmış sürücü / node açık. Uyarıdaki **Hepsini kapat**'a basın |

Algılama ile ilgili sorunlar: [`lidar_algilama` → Sorun giderme](../../algilama/lidar_algilama/README.md#-sorun-giderme).

---

## 🗂️ Paket yapısı

```
kontrol_merkezi/
├── launch/lidar_sistem.launch.py    # sürücü + RViz + lidar_algilama düğümleri + panel
├── rviz/lidar.rviz                  # hazır RViz ekranı
├── resource/kontrol_merkezi.svg     # uygulama simgesi
└── scripts/
    ├── kontrol_merkezi.py           # Kontrol Merkezi (tek pencere)
    ├── kontrol_merkezi_baslat.sh    # masaüstü simgesi bunu açar
    ├── ayar_paneli.py               # tek başına ayar paneli
    ├── arayuz/                      # iki arayüzün ortak Python paketi (sayfalar, süreç yönetimi, ayarlar)
    ├── engel_sistemi_baslat.sh
    ├── lidar_kayit.sh
    └── lidar_oynat.sh
```

Kullanıcı verisi: `~/.ros/kontrol_merkezi/` (arayüz ayarları, son ayarlar, profiller, günlük),
kayıtlar `~/robotaksi_kayitlar/`.
