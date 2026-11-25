# Nasıl çalışır: LiDAR ile sürüş (düz git, engel / yaya görünce dur)

1. [Kısaca](#1-kısaca)
2. [Algoritma adım adım](#2-algoritma-adım-adım)
3. [LiDAR aracın önünü nereden biliyor?](#3-lidar-aracın-önünü-nereden-biliyor)
4. [ÖNCE: Teensy'deki firmware](#4-önce-teensydeki-firmware)
5. [Kurulum](#5-kurulum)
6. [Araç ölçüleri](#6-araç-ölçüleri)
7. [Araçta çalıştırma, adım adım](#7-araçta-çalıştırma-adım-adım)
8. [Araç başında ayar yapma (deneme-yanılma)](#8-araç-başında-ayar-yapma-deneme-yanılma)
9. [Düz gitme (yön tutma)](#9-düz-gitme-yön-tutma)
10. [LiDAR ve araç olmadan, masada deneme](#10-lidar-ve-araç-olmadan-masada-deneme)
11. [Sorun giderme](#11-sorun-giderme)
12. [Deneme sırası](#12-deneme-sırası-atlamadan)

---

## 1. Kısaca

Araç **düz gider. Önündeki yolda engel ya da yaya görünce fren yapar. Yol 2 saniye boş kalınca
yeniden kalkar.** Bunu sen `surus_aktif false` diyene kadar tekrarlar.

- Harita, SLAM ya da GPS **kullanmaz**. Nerede olduğunu bilmez, bilmesine de gerek yok.
  O an gördüğüne bakıp karar verir.
- Şerit takip etmez, engelin etrafından dolanmaz, geri gitmez.
- Hız ayarı yok: gaz ya açık ya kapalı. Hızı Teensy'nin verdiği sabit gaz belirler.
- Direksiyon ya sabittir (84) ya da yön tutma açıksa sapmayı düzeltir (Bölüm 9).

```
RoboSense Helios 32 ──Ethernet──► rslidar_sdk ──/rslidar_points──┬─► engel_node ──/engel/durum──┐
                                                                 └─► nesne_node ──/nesne/liste──┤
                                                                     (+ /nesne/odometri)        ▼
Logitech F710 ──► gamepad_node ──/gamepad/state──┐                                         lidar_surus
                                                 ▼                                              │
                       Teensy ◄──USB seri, 50 Hz── serial_comm_node ◄────────/arac/komut────────┘
```

| Düğüm | Paket | Görevi |
|---|---|---|
| `rslidar_sdk_node` | `rslidar_sdk` | LiDAR sürücüsü: saniyede 10 kez 360° nokta bulutu |
| `engel_node` | `lidar_algilama` | Önündeki yelpazede engel var mı: GÜVENLİ / DİKKAT / DUR |
| `nesne_node` | `lidar_algilama` | Nesneleri tek tek bulur, takip eder; yol koridorundakileri ve hareket edenleri yayınlar |
| `lidar_surus` | `lidar_surus` | Karar: gaz mı fren mi, direksiyon kaç |
| `serial_comm_node` | `arac_haberlesme` | Teensy'ye yazan **tek** düğüm. Kumandayı ve `/arac/komut`'u birleştirir |
| `gamepad_node` | `arac_haberlesme` | Kumandayı okur (LB = manuel ↔ otonom) |

---

## 2. Algoritma adım adım

### 2.1 LiDAR ne görüyor?

LiDAR saniyede 10 kez etrafını 360° tarar ve ~40 bin noktalık bir bulut verir. Her noktanın
konumu **LiDAR'ın kendi eksenlerinde** yazılır:

- **x = LiDAR'ın önü**, **y = solu**, **z = yukarısı**, birim metre, LiDAR'ın merkezi (0, 0, 0).
- Açılar x ekseninden ölçülür: 0° = ön, +90° = sol, −90° = sağ, 180° = arka.

Bütün algılama bu eksenlerle çalışır. Bu yüzden LiDAR'ın x ekseni aracın önüne bakmalı
(Bölüm 3).

Yukarıdan bakınca iki ayrı kontrol alanı var:

```
                       ▲ x (aracın önü)
          DİKKAT  .    |    .                  ┌───────┐
                .  \   |   /  .                │koridor│  nesne_node: yol koridoru
        DUR    .    \  |  /    .               │ (dik- │  (dikdörtgen: araç genişliği
              .  ___ \ | / ___  .              │dörtgen│   + iki yanda pay, ~8 m ileri)
                    yelpaze                    │       │
                (engel_node, pasta dilimi)     │       │
                       [L] ◄── LiDAR           └──[L]──┘
                    ┌───────┐                   ┌───────┐
                    │ araç  │                   │ araç  │
                    └───────┘                   └───────┘
```

### 2.2 `engel_node`: önümde bir şey var mı? (her taramada, 10 Hz)

1. **Süzgeç.** Şu noktalar atılır:
   - LiDAR'a `min_mesafe`'den yakın olanlar (aracın kendi kaputu),
   - yükseklik bandının dışında kalanlar: `z_min`'in altı (zemin) ve `z_max`'ın üstü (dal, tavan),
   - yelpazenin dışında kalanlar (`merkez_aci` yönünde, `gorus_genisligi` genişliğinde),
   - `dikkat_mesafesi`'nden uzak olanlar.
2. **Gürültüye karşı.** Kalan nokta sayısı `min_nokta_sayisi`'ndan (10) azsa engel yok sayılır:
   toz, yağmur damlası ya da tek tük yansıma aracı durdurmasın. Mesafe olarak en yakın tek nokta
   değil, **en yakın 10. nokta** alınır.
3. **Karar.** Bu mesafe `dur_mesafesi`'nden kısaysa **DUR**, `dikkat_mesafesi`'nden kısaysa
   **DİKKAT**, değilse **GÜVENLİ**. Sınırda titremesin diye 10 cm pay (histerezis) var. Örneğin
   DUR 3,2 m ise, DUR'dan çıkmak için engelin 3,3 m'ye uzaklaşması gerekir.
4. Sonuç `/engel/durum`'a yazılır: karar, mesafe ve kararı veren nokta.

Bu hızlı ve basit bir kontrol. Nesnenin ne olduğuna bakmaz: yelpazede yeterince nokta varsa durur.

### 2.3 `nesne_node`: önümdeki şeyler ne, hareket ediyorlar mı? (10 Hz)

1. **Gövde kutusu:** aracın kendi üstüne düşen noktalar silinir. Ardından bulut seyreltilir
   (`voxel_boyu`).
2. **Zemin:** yer düzlemi otomatik bulunur (RANSAC). Zemindeki noktalar atılır. Zemin
   ayrıldığı için alçak nesneler de görülür.
3. **Aracın hareketi (LiDAR odometrisi):** art arda gelen taramalar üst üste oturtularak aracın
   ne kadar gittiği ve döndüğü bulunur (`/nesne/odometri`). Böylece araç giderken duran
   nesneler "geriye kayıyor" sanılmaz.
4. **Hareket testi:** bir noktanın şu an bulunduğu yer, 0,3–1 saniye önceki taramalarda boş
   görülmüşse (ışın oradan geçip arkaya çarpmışsa), o nokta hareketlidir.
5. **Kümeleme:** birbirine yakın noktalar tek nesne sayılır. 5 noktadan küçük kümeler gürültü
   kabul edilir.
6. **Kutu ve etiket:** her nesneye bir kutu çizilir ve şekline göre kaba bir etiket verilir:
   - `yaya_adayi`: tepesi yerden 1,0–2,1 m ve tabanı en fazla 1,2 × 1,0 m,
   - `yapi`: uzun ve ince (duvar),
   - `engel`: geri kalan her şey.
7. **Takip:** nesneler turdan tura eşleştirilir. Her birine numara verilir, hızı ve **çarpışma
   süresi** hesaplanır.
8. **Yayın** (`/nesne/liste`): **yol koridorundaki her şey** (duruyor olsa bile) ve **hareket
   eden her şey**. Duvar, park etmiş araç gibi yolun dışında duran şeyler yayınlanmaz.

### 2.4 `lidar_surus`: gaz mı, fren mi? (saniyede 20 kez)

Yukarıdan aşağı ilk tutan kural kazanır:

| Sıra | Durum | Ne zaman | Araca |
|---|---|---|---|
| 1 | `BEKLIYOR` | `surus_aktif` false (açılışta böyle) | fren |
| 2 | `SURE_DOLDU` | `azami_surus_suresi` doldu | fren |
| 3 | `VERI_YOK` | LiDAR / algılama / yön verisi yarım saniyedir gelmiyor | fren |
| 4 | `DUR` | `engel_node` DUR diyor **ya da** yol koridorunda `nesne_dur_mesafesi`'nden yakın nesne var **ya da** koridordaki bir nesneye çarpmaya `carpisma_suresi_dur`'dan (2,5 sn) az kaldı | fren |
| 5 | `KALKIS_BEKLE` | son DUR / VERI_YOK'tan ya da başlatmadan beri 2 sn geçmedi | fren |
| 6 | `DIKKAT` | `engel_node` DİKKAT diyor ya da koridorda `nesne_dikkat_mesafesi`'nden yakın nesne var | gaz (istersen kesilir: `dikkatte_gaz_kes`) |
| 7 | `GIDIYOR` | yol boş | gaz |

Önemli noktalar:

- **Etiket karar vermez.** Yaya, kutu ya da koni fark etmez. Koridorda yeterince yakınsa
  durur. "Yaya mı engel mi" sorusu sadece ekranda bilgi için var.
- **Yaya önünde durduğu sürece araç kalkmaz.** Her turda yeniden DUR denir. Yaya çekilince
  2 sn bekler, sonra kalkar.
- **Çarpışma süresi:** sana doğru yürüyen bir yaya daha uzaktayken de durdurur. Duran bir
  engele 1 m/s ile yaklaşırken çoğunlukla önce mesafe kuralı tutar.
- Yolun kenarındaki bir yaya koridorda değilse ve yelpazede DUR mesafesinden yakın değilse
  araç durmaz.

### 2.5 `serial_comm_node`: Teensy'ye ne gidiyor? (saniyede 50 kez)

| Durum | Teensy'ye giden |
|---|---|
| Kumanda bağlı, **LB = MANUEL** | Her şey kumandadan |
| Kumanda bağlı, **LB = OTONOM** ya da kumanda hiç bağlanmadı | `lidar_surus`'un komutu |
| Kumanda OTONOM'dayken koptu | `lidar_surus`'un komutu (yarış turu kumanda parazitinden ölmesin) |
| Kumanda MANUEL'deyken koptu | **fren** |
| `lidar_surus` 0,3 sn komut göndermedi (çöktü / dondu) | **fren** |
| `serial_comm_node` kapandı / USB koptu | Teensy 300 ms sonra kendisi frenler (firmware'e bağlı, Bölüm 4) |

Paket 5 byte: `0xAA | direksiyon (25–165) | bayraklar (gaz, fren, geri) | sağlama | 0x55`.
Otonom direksiyon 5'lik bloklara yuvarlanır (84 → 82).

---

## 3. LiDAR aracın önünü nereden biliyor?

**Otomatik bilmiyor.** LiDAR'ın bir "önü" var: gövdesinin x ekseni. Bu, cihazın nasıl
takıldığına bağlı. Sistem "aracın önü = LiDAR'ın x ekseni" diye kabul eder:

- `engel_node`'un yelpazesi `merkez_aci: 0` ile LiDAR'ın x yönüne bakar,
- `nesne_node`'un yol koridoru **her zaman** LiDAR'ın x yönündedir. Bunun ayarı yok.

O yüzden **LiDAR'ı x ekseni aracın önüne bakacak şekilde tak.** Sadece `merkez_aci`'yı
çevirmek yetmez: yelpaze döner ama yol koridoru dönmez. Atölyedeki eski ayarlarda
`merkez_aci: -137` vardı, çünkü LiDAR masada yan duruyordu. Araçta bu ayar uygulanmaz,
araç ölçüleri dosyası yönü 0'a çeker.

**Nasıl kontrol edersin** (sistem açıkken, araç dururken):

1. Biri aracın **tam önünde**, 3–4 m ileride dursun.
2. RViz'de (Bölüm 7.4) o kişinin noktaları LiDAR'ın **kırmızı ekseni / beyaz ok** yönünde
   görünmeli. Panelin yukarıdan bakış çiziminde de aracın önünde görünmeli.
3. Ya da terminalden: `ros2 topic echo /engel/durum`. Kişi yelpazedeyken `engel_noktasi`'nda
   **x artı** ve **y sıfıra yakın** olmalı. y artıysa kişi solda, eksiyse sağda görünüyor demektir.
4. Kişi aracın sağında ya da arkasında görünüyorsa: LiDAR'ı yuvasında, kişi önde görünene kadar
   çevir.

---

## 4. ÖNCE: Teensy'deki firmware

`serial_comm_node` 5 byte'lık paket gönderiyor. Teensy'de **bu protokolü anlayan firmware**
yüklü olmalı.

⚠️ `~/Downloads/ovatson_icerik/29.06/teensy_gomulu_duzeltilmis.ino` **bu firmware DEĞİL**:

| | Eski `.ino` | `serial_comm_node`'un beklediği |
|---|---|---|
| Komut | Tek byte: `201` gaz, `202` fren, `204` vites, `205` boşta, `25–165` direksiyon | 5 byte'lık çerçeve |
| Manuel / otonom seçimi | RC alıcının 7. kanalı | Gamepad LB, bilgisayar üzerinden |
| Paket gelmezse | Hiçbir şey yapmaz, **son komut sürer** | 300 ms sonra fren |

Eski firmware'e bizim paketlerimiz giderse her byte ayrı bir komut sanılır: direksiyon
titrer, gaz ve fren hiç gitmez. Araçtaki güncel firmware'i ekipten iste (kodda adı
`robotaxi_gamepad.ino` diye geçiyor). Doğrulanana kadar araçla sadece masada ve teker havadayken
dene (Bölüm 12, 1. ve 2. adım).

---

## 5. Kurulum

### 5.1 Bu bilgisayarda (bir kez)

```bash
sudo usermod -aG dialout $USER     # Teensy portuna yazma izni; sonra OTURUMU KAPAT-AÇ
sudo apt install python3-pygame    # kumanda için
sudo ufw allow in on enp109s0      # güvenlik duvarı LiDAR'ın UDP paketlerini (6699/7788) kesmesin
cd ~/robotaksi_ws
colcon build --symlink-install
```

ROS ortam satırları `~/.bashrc`'de zaten var. Kod değişince `colcon build --symlink-install`
yeterli. `.yaml` ayar dosyalarını değiştirdiğinde derlemeye gerek yok, launch'ı yeniden
başlatman yeter.

### 5.2 Başka bir bilgisayarda

Gereken: **Ubuntu 24.04** ve **ROS 2 Jazzy (desktop)**.

```bash
cd ~
git clone https://github.com/ayberkdoruk/robotaksi-autonomy.git robotaksi_ws
cd ~/robotaksi_ws
sudo apt install python3-vcstool ros-jazzy-pcl-conversions libpcl-dev ros-jazzy-python-qt-binding python3-yaml zenity python3-pygame
scripts/dis_depolari_cek.sh                       # LiDAR sürücüsü + Helios yaması
source /opt/ros/jazzy/setup.bash
sudo rosdep init 2>/dev/null; rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
sudo usermod -aG dialout $USER                    # sonra oturumu kapat-aç
```

`~/.bashrc` dosyasının sonuna şunları ekle:

```bash
source /opt/ros/jazzy/setup.bash
source ~/robotaksi_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export FASTRTPS_DEFAULT_PROFILES_FILE=~/robotaksi_ws/install/robotaksi_baslat/share/robotaksi_baslat/config/fastdds_buyuk_mesaj.xml
```

Kablolu ağa LiDAR'ın veri gönderdiği sabit IP'yi ver (genelde LiDAR `192.168.1.200`,
bilgisayar `192.168.1.102`). `ufw` açıksa kablolu arayüze izin ver:
`sudo ufw allow in on <arayüz>` (arayüz adı: `ip -br link`). Kontrol:
`ros2 launch rslidar_sdk start.py` ve ardından `ros2 topic hz /rslidar_points` (~10 Hz).

Notlar:
- GitHub'da sadece **push edilmiş** kod var. Bu bilgisayardaki son değişiklikler commit + push
  edilmeden diğer bilgisayara gitmez. İnternetsiz aktarmak için `robotaksi_ws`'i USB'ye kopyala.
  `build/`, `install/` ve `log/` klasörlerini alma.
- Pistte başka bir laptoptan izlemek için iki bilgisayarda da
  `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` yap ve aynı ağa bağlanın.

---

## 6. Araç ölçüleri

Dosya: [`src/ortak/robotaksi_baslat/config/arac_olculeri.yaml`](src/ortak/robotaksi_baslat/config/arac_olculeri.yaml)

Metreyle ölçüp yazarsın, hepsi **LiDAR'ın merkezinden**:

| Ölçü | Aracın raporuna göre |
|---|---|
| `lidar_yukseklik`: yerden LiDAR'ın merkezine | araç 1,50 m; LiDAR tavandaysa 1,50 + ayağı |
| `lidar_on_tampon`: LiDAR → en ön nokta | ön + arka = 2,15 |
| `lidar_arka_tampon`: LiDAR → en arka nokta | |
| `lidar_sol_kenar` / `lidar_sag_kenar` | sol + sağ = 1,25 (ortadaysa 0,63 / 0,63) |

**Şu an dosyada olanlar (SolidWorks çiziminden, 2026-11-25):** yükseklik 1,00 · ön 0,10 ·
arka 2,05 · sol 0,625 · sağ 0,625. LiDAR ön çamurluğun hemen arkasında, bir direğin üstünde.
Yanlar için LiDAR'ın ortada olduğu **varsayıldı**. Araçta metreyle doğrula. Bu ölçülerle hesaplanan:
DUR 2,1 m (LiDAR'dan; tampondan 2,0), DİKKAT 4,1 m, yelpaze 50°, zemin −1,00, alt sınır −0,85,
koridor 1,95 m × 8 m.

Aynı dosyada **tercihler** de var. Bunlar ölçü değil, istediğin gibi ayarlarsın:

| Tercih | Varsayılan | Anlamı |
|---|---|---|
| `durma_boslugu` | 2.0 | Engel **ön tampona** bu kadar yaklaşınca DUR |
| `dikkat_boslugu` | 4.0 | Ön tampona bu kadar yaklaşınca DİKKAT |
| `yan_pay` | 0.35 | Yol koridoru aracın iki yanından bu kadar taşar |
| `zemin_payi` | 0.15 | Yerden bu yüksekliğin altı engel sayılmaz (zemin, tümsek) |
| `govde_payi` | 0.10 | Aracın kendi gövdesinin çevresinde yok sayılan pay |

Launch bu sayılardan şunları hesaplar ve açılışta `Arac olculeri uygulandi` başlığının altına
yazar: bakış yönü (0 = öne), yelpaze açısı, yakın sınır, zemin yüksekliği, alt sınır, gövde
kutusu, koridor genişliği ve uzunluğu, DUR ve DİKKAT mesafeleri. Örnek: LiDAR ön tamponun
1,2 m gerisindeyse `durma_boslugu: 2.0` → `dur_mesafesi = 3.2` (LiDAR'dan).

- Ölçüler boşsa (hepsi 0) araçta launch **açılmaz**. Masada (`kuru:=true`) uyarı verip devam eder.
- Kontrol Merkezi'nin kümeleme, hareket ve takip ayarlarına dokunulmaz.

---

## 7. Araçta çalıştırma, adım adım

### 7.1 Bağlantılar

1. LiDAR: güç + Ethernet (sabit IP).
2. Teensy: USB. Kontrol: `ls -l /dev/serial/by-id/` içinde `...Teensyduino...` görünmeli.
3. Kumanda: USB alıcı takılı, kumanda açık, ön anahtar **X** konumunda.

### 7.2 Başlat (Terminal 1)

```bash
ros2 launch lidar_surus lidar_surus.launch.py
```

Bu tek komut sürücü, algılama, Teensy, kumanda ve `lidar_surus` düğümlerinin hepsini açar.
**Araç frende bekler.** Terminalde `Arac olculeri uygulandi` listesine ve
`Teensy baglandi` satırına bak.

| Seçenek | Ne zaman |
|---|---|
| `kuru:=true` | Teensy'ye **hiç yazmadan** dene |
| `kumanda:=false` | Kumanda yoksa |
| `surucu:=false` | Kayıt (rosbag) oynatırken |

### 7.3 Panel (Terminal 2, isteğe bağlı ama önerilir)

```bash
ros2 run kontrol_merkezi ayar_paneli
```

Solda canlı karar ve yukarıdan bakış çizimi, sağda kaydırıcılar (Bölüm 8).

### 7.4 RViz (Terminal 3, isteğe bağlı)

```bash
rviz2 -d ~/robotaksi_ws/install/kontrol_merkezi/share/kontrol_merkezi/rviz/lidar.rviz
```

Bulut, yelpaze (sarı = DİKKAT, kırmızı = DUR sınırı), kararı veren noktadaki top (karar renginde), nesne kutuları ve yol
koridoru görünür. **İlk iş LiDAR'ın önünü kontrol et (Bölüm 3).**

### 7.5 Sür (Terminal 4)

```bash
ros2 param set /lidar_surus surus_aktif true     # KALK: 2 sn bekler, yol boşsa gaz
ros2 param set /lidar_surus surus_aktif false    # DUR
```

Kumandayla: **LB** manuel ↔ otonom. Manuelde sağ stick direksiyon, sol stick ileri gaz,
**RT** fren, **RB** ileri/geri vites. **Bir sorun olursa LB'ye bas**, araç anında manuele geçer.
İlk denemelerde süre sınırı koy: `ros2 param set /lidar_surus azami_surus_suresi 10.0`.

### 7.6 İzle

| Komut | Ne gösterir |
|---|---|
| `ros2 topic echo /lidar_surus/durum` | Karar + sebep: `GIDIYOR`, `DUR: engel 1.8 m`, `DUR: nesne #12 yaya_adayi 2.4 m`, `VERI_YOK: ...` |
| `ros2 topic echo /arac/gonderilen` | Teensy'ye **gerçekten** giden direksiyon / gaz / fren |
| `ros2 topic echo /arac/surus_modu` | `1` manuel, `2` otonom, `3` güvenlik freni |
| `ros2 topic echo --qos-durability transient_local /arac/baglanti` | Teensy portu açık mı |
| `ros2 topic echo /engel/durum` | Engel kararı, mesafe, kararı veren nokta |
| `ros2 topic echo /nesne/liste` | Koridordaki ve hareketli nesneler |
| `ros2 topic hz /rslidar_points` | LiDAR verisi geliyor mu (~10 Hz) |

Terminal 1'de her karar değişikliği bir satır olarak yazılır. `serial_comm_node` da yarım
saniyede bir `[TX] OTONOM gaz=1 fren=0 geri=0 steer= 82 | yazma=OK` yazar.

---

## 8. Araç başında ayar yapma (deneme-yanılma)

Evet, hepsi araç başında **çalışırken** değişir. Yeniden başlatmana gerek yok.

### 8.1 Üç araç

| Ne değişecek | Nasıl | Kalıcı mı? |
|---|---|---|
| `engel_node` ve `nesne_node`: yelpaze, DUR / DİKKAT mesafesi, yükseklik, koridor, kümeleme, yaya etiketi, takip... | **Panel** (`ros2 run kontrol_merkezi ayar_paneli`), kaydırıcıyla | Panel `son_ayarlar.yaml`'a kaydeder. Ama ölçülerden hesaplanan alanlar her açılışta yeniden hesaplanır (8.3) |
| `lidar_surus`: kalkış bekleme, çarpışma süresi, yön tutma, süre sınırı... | `ros2 param set /lidar_surus <ad> <değer>` | Hayır. Kalıcı olsun diye [`lidar_surus_params.yaml`](src/surus/lidar_surus/config/lidar_surus_params.yaml)'a yaz |
| Araç ölçüleri ve tercihler | [`arac_olculeri.yaml`](src/ortak/robotaksi_baslat/config/arac_olculeri.yaml)'ı düzenle | Evet, launch yeniden başlayınca uygulanır |

### 8.2 Belirtiye göre ne değiştirilir?

| Belirti | Değiştir |
|---|---|
| Çok **geç** duruyor | Panel: DUR mesafesi ↑ · ya da `lidar_surus`: `carpisma_suresi_dur` ↑ |
| Çok **erken / uzaktan** duruyor | Panel: DUR mesafesi ↓ |
| **Yandaki** şeyler (çöp tenekesi, duvar) için duruyor | Panel → Uyarı alanı: Görüş genişliği ↓ · Kutulama: koridor genişliği ↓ |
| Önündeki şeyi **görmüyor** | Görüş genişliği ↑ · Filtreler: Alt sınır (`z_min`) aşağı · En az nokta sayısı ↓ |
| **Yeri** engel sanıyor (boş yolda DUR) | Filtreler: Alt sınır (`z_min`) yukarı (sıfıra doğru) |
| Aracın **kendisini** görüyor (hep çok yakın mesafede DUR) | Filtreler: Yakın sınır ↑ · Kutulama: gövde kutusu büyüt |
| Toz / yağmurda boşuna duruyor | Filtreler: En az nokta sayısı ↑ |
| Yaya çekildikten sonra çok bekliyor / hemen kalkıyor | `ros2 param set /lidar_surus kalkis_bekleme 1.0` (ya da 3.0) |
| DİKKAT'te yavaşlasın | `ros2 param set /lidar_surus dikkatte_gaz_kes true` |
| Koridordaki nesne için mesafe | `ros2 param set /lidar_surus nesne_dur_mesafesi 4.0` (LiDAR'dan, metre) |
| Yaya `engel` diye etiketleniyor | Panel → Etiket: yaya tepe yüksekliği aralığı (sadece ekranı etkiler, durmayı değil) |

Panelin yukarıdan bakış çizimi ve RViz değişikliği anında gösterir. Biri aracın önünde farklı
mesafelerde dururken kararın ne zaman DUR'a geçtiğine bak.

### 8.3 Bulduğun değeri kalıcı yapmak

Şu alanlar araçta her açılışta **ölçülerden yeniden hesaplanır**:
`merkez_aci`, `gorus_genisligi`, `min_mesafe`, `z_min`, `dur_mesafesi`, `dikkat_mesafesi`,
`zemin_z`, `govde_*`, `koridor_*`, `nesne_merkez_aci`, `nesne_gorus_genisligi`, `menzil`,
`voxel_boyu` ve `lidar_surus`'un `nesne_dur_mesafesi` / `nesne_dikkat_mesafesi`'si.

Panelde bunlardan birini değiştirip beğenirsen karşılığını `arac_olculeri.yaml`'a yaz:

| Panelde bulduğun (LiDAR'dan) | Dosyaya yaz |
|---|---|
| DUR mesafesi = 3.6 | `durma_boslugu = 3.6 − lidar_on_tampon` |
| DİKKAT mesafesi = 5.5 | `dikkat_boslugu = 5.5 − lidar_on_tampon` |
| Koridor genişliği = 1.8 | `yan_pay = (1.8 − araç genişliği) / 2` |
| Alt sınır (`z_min`) = −1.30 | `zemin_payi = lidar_yukseklik − 1.30` |

Görüş genişliği DUR mesafesinden ve koridordan hesaplanıyor. Ayrı bir tercihi yok.

---

## 9. Düz gitme (yön tutma)

Direksiyonu başta düz ayarlamak yetmez: direksiyonun ±3 birimlik toleransı, yol eğimi ve lastikler
aracı yavaş yavaş bir yana kaydırır. Yön tutma açıkken **sürüş başladığı andaki yön hedef
alınır**. Araç saparsa direksiyon ters yöne düzeltilir:

```
direksiyon = 84 − yon_kazanc × (hedef yön − şu anki yön)     (derece; 25 = tam sol, 165 = tam sağ)
```

| `yon_tutma` | Yön nereden | Gereken |
|---|---|---|
| `kapali` (varsayılan) | Okunmaz, direksiyon hep 84 | — |
| `imu` (**önerilen**) | ZED 2'nin içindeki IMU (jiroskop) | ZED sürücüsü açık ve IMU yayını açık: `~/ros2_ws`'teki ZED ayarında `publish_imu: false` → `true`. Topic adını `ros2 topic list \| grep imu` ile kontrol et, farklıysa `imu_topic` parametresini değiştir |
| `lidar` | `/nesne/odometri` (LiDAR taramaları) | Ek donanım yok. Çevrede duvar, direk gibi şeyler olmalı. Araç hareket ederken henüz doğrulanmadı |

Seçilen kaynaktan veri yarım saniye gelmezse araç **fren** yapar.

Sahada açmak:
1. Yön tutma kapalıyken kumandayla düz bir çizgide sür. Kaynağı izle (IMU topic'i ya da
   `ros2 topic echo /nesne/odometri`): düz giderken yön sabit kalıyor, dönünce değişiyor mu?
2. `ros2 param set /lidar_surus yon_tutma imu`
3. Aracı düz yerleştir, `surus_aktif true`. `/lidar_surus/durum` satırının sonunda
   `(yon hatasi +1.2 derece, direksiyon 83)` görünür.
4. **İlk denemede düzeltmenin yönünü kontrol et:** araç sağa saparken direksiyon değeri
   **küçülmeli** (sola). Büyüyorsa hemen `surus_aktif false` de ve
   `ros2 param set /lidar_surus yon_kazanc -1.0` yap.
5. Zigzag yapıyorsa `yon_kazanc` ↓ (0.5). Düzeltme geç kalıyorsa ↑ (1.5). En fazla düzeltme
   `yon_siniri` (84 ± 15).

Not: otonom direksiyon 5'lik bloklara yuvarlandığı için 1–2 derecelik sapmalar düzeltilmez,
yaklaşık 3 dereceden itibaren düzeltilir. Araç düz değil de hep aynı yana çekiyorsa önce
`duz_direksiyon`'u ayarla (`79`, `89`...).

---

## 10. LiDAR ve araç olmadan, masada deneme

Sahada alınmış kayıtlar (`~/robotaksi_kayitlar/`) gerçek LiDAR'ın yerine oynatılır. `kuru:=true`
olduğu için Teensy'ye hiçbir şey yazılmaz.

```bash
# Terminal 1
ros2 launch lidar_surus lidar_surus.launch.py surucu:=false kuru:=true kumanda:=false
# Terminal 2
ros2 bag play ~/robotaksi_kayitlar/2026-11-23_18-16-27_mta_atolye --loop
# Terminal 3
ros2 param set /lidar_surus surus_aktif true
ros2 topic echo /lidar_surus/durum
```

Atölye kaydında Terminal 1'de şuna benzer satırlar çıkar:

```
GIDIYOR  [gaz=1 fren=0]
DUR: nesne #40 yaya_adayi 1.3 m  [gaz=0 fren=1]
KALKIS_BEKLE: 1.9 sn  [gaz=0 fren=1]
GIDIYOR  [gaz=1 fren=0]
DUR: engel 1.0 m  [gaz=0 fren=1]
```

Kaydı durdurursan yarım saniye içinde `VERI_YOK` gelir ve fren basılır. Panel ve RViz de
masada açılabilir (Bölüm 7.3, 7.4).

Sınırlar:
- Ölçüler boşken masada atölye (kayıtlı) ayarları kullanılır.
- Kayıtlar LiDAR sabitken alındı. Masada sadece "önüne biri gelince duruyor mu" kısmı
  denenebilir. Giderken yaklaşma ve yön tutma sahada ya da araç hareket ederken alınan kayıtla
  görülür.

---

## 11. Sorun giderme

| Belirti | Sebep / çözüm |
|---|---|
| `ARAC OLCULERI GIRILMEMIS` | `arac_olculeri.yaml`'ı doldur (Bölüm 6). Masada denemek için `kuru:=true` |
| `Seri port acilamadi ... dialout` | `sudo usermod -aG dialout $USER`, oturumu kapat-aç. Teensy takılı mı: `ls /dev/serial/by-id/` |
| `KUMANDA YOK!` | Alıcı takılı mı, kumanda açık mı, anahtar X'te mi. Kumandasız: `kumanda:=false` |
| `pygame yok` | `sudo apt install python3-pygame` |
| Sürekli `VERI_YOK: /engel/durum gelmiyor` | LiDAR verisi yok: `ros2 topic hz /rslidar_points`. Kablo, IP, sürücü |
| `ros2 topic hz /rslidar_points` boş, ama `sudo tcpdump -i enp109s0 -c 5 udp port 6699` paket görüyor | Güvenlik duvarı kesiyor: `sudo ufw allow in on enp109s0` |
| Ctrl+C'de kırmızı `process has died ... exit code -2` / rslidar `std::system_error` | Normal, kapanış mesajı. Açılışta çıkarsa sorun |
| Sürekli `VERI_YOK: yon (IMU) gelmiyor` | ZED sürücüsü kapalı ya da IMU yayını kapalı / topic adı farklı (Bölüm 9). Ya da `yon_tutma kapali` |
| Önünde kimse yokken `DUR` | RViz'de kararı veren noktayı gösteren topa bak (LiDAR'dan ona çizgi uzanır): zemin mi, aracın kendisi mi, yandaki bir şey mi (Bölüm 8.2) |
| Önündeki kişiyi görmüyor | LiDAR'ın önü yanlış olabilir (Bölüm 3) |
| `[TX] ... gaz=1` ama araç gitmiyor | Teensy'de yanlış firmware olabilir (Bölüm 4) |
| Araç bir yana çekiyor | Yön tutma (Bölüm 9) ya da `duz_direksiyon` |
| Topic'ler görünmüyor | İki terminalde de `ROS_DOMAIN_ID` aynı mı? `source ~/.bashrc` |
| Kod değişikliği etkisiz | `colcon build --symlink-install` ve yeni terminal |

---

## 12. Deneme sırası (atlamadan)

Her denemeden önce kayıt al (Kontrol Merkezi → Kayıt Al). Sorunları sonra masada aynı kayıtla
tekrar oynatıp ayarlarız.

1. **Masada, Teensy'siz** (Bölüm 10). Kararların mantıklı değiştiğini gör.
2. **Araçta, araç dururken, `kuru:=true`:** ölçüler girildi mi? LiDAR'ın önü doğru mu
   (Bölüm 3)? Biri önde farklı mesafelerde dursun: DUR doğru mesafede mi? Panelle ayarla.
3. **Teker havada (araç krikoda):** Teensy takılı, kumanda elde, LB ile otonom.
   - Tekerler dönmeli.
   - Önüne biri ya da bir kutu gelince fren basmalı.
   - `serial_comm_node`'u Ctrl+C ile kapatınca fren basmalı.
   - Firmware de böylece doğrulanır.
4. **Yerde, kısa ve boş pist:** `azami_surus_suresi 10.0`, yön tutma kapalı. Bir kişi
   kumandayla başında dursun. Önce duran bir kutuyla, sonra yürüyen biriyle dene. Aracın ne kadar
   saptığını gör.
5. **Yön tutma:** Bölüm 9'daki adımlarla aç, düzeltme yönünü kontrol et.
