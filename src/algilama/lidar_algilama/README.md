<div align="center">

# 🚧 lidar_algilama — LiDAR ile Engel ve Nesne Algılama

**RoboSense Helios 32** nokta bulutundan gerçek zamanlı **DUR / DİKKAT / GÜVENLİ** kararı,
3B nesne kutuları, mesafe, kaba etiket (yaya adayı / yapı / engel) ve nesne takibi.

<img src="../../../docs/images/rviz_nesneler.png" alt="RViz: nesne kutuları, mesafeler, yelpaze ve karar" width="100%">

<sub>Gerçek LiDAR kaydı, RViz. <b>Turuncu</b>: engel kutuları · <b>mavi</b>: yaya adayı ·
<b>sarı yelpaze</b>: uyarı alanı (kırmızı yay = DUR sınırı) · <b>açık mavi</b>: kutulama alanı.
Her kutunun üstünde takip numarası, etiket ve mesafe yazar.</sub>

</div>

Robotaksi çalışma alanının `algilama/` katmanındaki paket ([çalışma alanı rehberi](../../../README.md)).
Araca komut vermez; sonuçlarını topic olarak yayınlar, sürüş yöntemleri istediğini okur.
Arayüz, ayar paneli ve kayıt/oynatma [`kontrol_merkezi`](../../araclar/kontrol_merkezi/README.md) paketindedir.

---

## ✨ Ne yapar?

**`engel_node` — sektör tabanlı engel durumu**
- LiDAR'ın etrafında istediğiniz yöne, istediğiniz genişlikte bir **yelpaze** (sektör) çizer.
- Yelpazenin içindeki, yükseklik ve mesafe filtresinden geçen noktalara bakar.
- En yakın engelin hangi bölgede olduğunu **DUR** (kırmızı), **DİKKAT** (sarı) veya **GÜVENLİ** (yeşil)
  olarak, mesafesi ve konumuyla birlikte `/engel/durum`'da yayınlar. Bu bir sürüş komutu değildir:
  bu bilgiyle ne yapılacağına sürüş yöntemi karar verir.
- Tek tük gürültü noktaları (toz, yağmur, yansıma) karar vermesin diye **en az N nokta** şartı ve
  mesafeyi en yakın N'inci noktaya göre ölçer.
- Engel tam sınırdayken kararın titrememesi için **histerezis** (10 cm) uygular.

**`nesne_node` — nesne algılama katmanı**
- Zemini her turda **RANSAC** ile ölçüp ayırır (sabit bir zemin yüksekliğine güvenmez).
  Açılışta zemini **kendisi bulur** (LiDAR'ın altındaki en alçak geniş yatay yüzey) ve ayarlardaki
  beklenen yükseklikle **karşılaştırır**: tutmazsa uyarır (LiDAR oynamış / yanlış profil).
- Kalan noktaları **kümeleyip** nesnelere ayırır (uzaktaki nesneler için mesafeye göre büyüyen tolerans).
- Her nesneyi **yönlü 3B kutu** içine alır, LiDAR'a en yakın noktasının **yatay mesafesini** ölçer.
- Kutunun şekline göre kaba etiket verir: **yaya adayı**, **yapı** (duvar vb.), **engel**.
- **Sadece hareket eden nesneleri** (yaya, araç) ve aracın önündeki **yol koridorundaki** her şeyi
  (duran engel dahil) gösterir; duvar, park etmiş araç gibi duranları gizler.
- Araç giderken kendi hareketini **LiDAR taramalarından** bulur (odometri/IMU gerekmez) ve çıkarır:
  araç 1 m/s giderken duran nesneler hareketli sanılmaz, kutular kaymaz.
- Nesneleri kareden kareye **yere göre takip eder**: numara, hız, araca göre yaklaşma hızı ve
  çarpışma süresi.
- Sonuçları `/nesne/liste` topic'inde yayınlar — başka node'lar (ör. kamera füzyonu) doğrudan okuyabilir.

> **Not:** Hiçbir node LiDAR'ın kendisine veya sürücüsüne ayar göndermez. Sadece sürücünün
> yayınladığı `/rslidar_points` bulutunu okur ve kendi içinde filtreler.

---

## 📸 Ekran görüntüleri

Aynı ayarlarla, engel yaklaştıkça kararın değişimi (DUR sınırı 1.9 m, DİKKAT sınırı 3.0 m):

<table>
  <tr>
    <th width="33%">🟢 GÜVENLİ</th>
    <th width="33%">🟡 DİKKAT</th>
    <th width="33%">🔴 DUR</th>
  </tr>
  <tr>
    <td><img src="../../../docs/images/panel_guvenli.png" alt="Ayar paneli: GÜVENLİ"></td>
    <td><img src="../../../docs/images/panel_dikkat.png" alt="Ayar paneli: DİKKAT"></td>
    <td><img src="../../../docs/images/panel_dur.png" alt="Ayar paneli: DUR"></td>
  </tr>
  <tr>
    <td align="center"><sub>Görüş alanında engel yok</sub></td>
    <td align="center"><sub>Engel önde, 2.70 m'de</sub></td>
    <td align="center"><sub>Engel solda 24°, 1.00 m'de</sub></td>
  </tr>
</table>

Yukarıdan bakış çiziminde **kırmızı** bölge DUR, **sarı** bölge DİKKAT alanıdır, **mavi kesik çizgi**
nesne kutulama alanını gösterir. Kararı veren nokta çizimde işaretlenir; üstte hangi açıda, kaç metrede
ve hangi yükseklikte olduğu yazar.

---

## 🧠 Nasıl çalışır?

```mermaid
flowchart LR
    L[RoboSense Helios 32] -->|Ethernet UDP| S[rslidar_sdk]
    S -->|/rslidar_points<br/>PointCloud2, 10 Hz| E[engel_node]
    S -->|/rslidar_points| N[nesne_node]

    E -->|/engel/durum<br/>EngelDurumu| P[ayar_paneli<br/>kontrol_merkezi]
    E -->|/engel/gorsel<br/>/engel/filtrelenmis_noktalar| R[RViz]
    N -->|/nesne/gorsel<br/>/nesne/kumeler<br/>/nesne/dinamik_noktalar| R
    N -->|/nesne/liste<br/>NesneDizisi<br/>/nesne/odometri| F[Başka node'lar<br/>ör. kamera füzyonu]

    P -.->|parametre ayarı| E
    P -.->|kutulama alanı| N
    E -.->|yelpaze ayarları okunur| N
```

### `engel_node` her turda (10 Hz)

1. **Yelpaze testi** — noktanın yatay yönü, bakış yönünden `gorus_genisligi / 2`'den fazla sapıyorsa atılır.
2. **Mesafe filtresi** — `min_mesafe`'den yakın (aracın kendisi, masa) ve `dikkat_mesafesi`'nden uzak noktalar atılır.
3. **Yükseklik filtresi** — `z_min` altındaki (zemin) ve `z_max` üstündeki (tavan, ağaç dalı) noktalar atılır.
4. **Gürültü filtresi** — kalan nokta sayısı `min_nokta_sayisi`'ndan azsa engel yok sayılır. Engel mesafesi
   olarak en yakın noktanın değil, **en yakın N'inci noktanın** mesafesi kullanılır (tek bir yansıma DUR verdiremez).
5. **Durum** — mesafe `dur_mesafesi`'nden küçükse DUR, `dikkat_mesafesi`'nden küçükse DİKKAT, değilse GÜVENLİ.
   Bir durumdan **çıkmak** için engelin sınırdan 10 cm daha uzaklaşması gerekir (histerezis).

### `nesne_node` her turda

```
nokta bulutu → kutulama alanı + gövde kutusu + voxel seyreltme
             → zemin ayırma (RANSAC; açılışta otomatik arama, sonra önceki düzlemin ± zemin_bant'ı)
             → zemin kontrolü (bulunan zemin zemin_z / egim_uyari ile tutuyor mu → /nesne/zemin_durumu)
             → aracın hareketi: LiDAR odometri (2B ICP, her yöndeki duvar/direk/ağaç) → /nesne/odometri
             → hareketli noktalar: şu anki yeri 0.3 / 0.7 / 1.0 sn önce BOŞ görülmüş mü? (görünürlük testi)
             → kümeleme (yatay + mesafeye göre büyüyen dikey tolerans); duvara yapışık yaya ayrılır
             → yönlü kutu + en yakın nokta mesafesi + tepe yüksekliği + yol koridorunda mı
             → kaba etiket (yaya_adayi / yapi / engel)
             → takip YERE GÖRE (numara, hız, kutu yumuşatma, son 3 turun 2'sinde kanıt → hareketli)
             → (sadece_dinamik açıksa) sadece hareketliler + yol koridorundakiler kalır
             → araca göre yaklaşma hızı ve çarpışma süresi
             → /nesne/liste + RViz görselleri
```

Algoritmalar ROS'tan bağımsız olarak [`src/nesne/`](src/nesne/) altındadır
(`zemin`, `odometri`, `dinamik`, `kumeleme`, `kutu`, `takip`); ROS tarafı sadece `nesne_node.cpp`'dedir.

#### Hareketli nesne nasıl ayırt edilir?

LiDAR ışını bir yere çarpıp döner; **ışının geçtiği yol boştur**. Bir noktanın şu anki yeri az önceki
taramada boş görülmüşse (o yöne giden ışın noktanın çok arkasından dönmüşse), oradaki cisim oraya
sonradan gelmiştir:

```text
1 sn önce:   LiDAR ----------------------------> duvar    (ışın boşluktan geçti)
şimdi:       LiDAR ----------> yaya                        (aynı yerde artık yaya var → hareketli)
```

- Duran cisimler bu testi geçemez: o yöne giden ışın ya cismin kendisine ya da önündeki bir şeye
  çarpmıştır. Eski yöntem küme merkezinin kaymasına bakıyordu; duran nesnelerin kümeleri birleşip
  ayrılınca merkez 0.6–0.8 m sıçradığı için duvarlar/masalar "hareketli" görünüyor, kutular titriyordu.
- Her nokta 3 geçmiş taramayla karşılaştırılır (`dinamik_bakis_suresi`'nin 1/3'ü, 2/3'ü ve kendisi kadar
  önce). Kümenin en az `dinamik_min_nokta` noktası ve `dinamik_oran` oranı hareketliyse o tur için
  kanıt var; **son 3 turun 2'sinde** kanıt olunca nesne hareketli olur. `dinamik_tutma` (3 sn) boyunca
  hiç kanıt gelmezse (yaya durdu) tekrar gizlenir.
- Yaya duvarın / park etmiş aracın dibinden geçip onunla tek kümede birleşirse, hareketli noktaları
  ayrı kümelenip kendi kutusunu alır.
- **Sınır:** LiDAR'dan doğrudan **uzaklaşan** bir nesne bu testle yakalanmaz (yeni yeri, eski halinin
  arkasında kalır). Aracın önünde giden biri yine de yol koridorunda olduğu için gösterilir.

#### Aracın hareketi (LiDAR odometri)

Araç 1 m/s giderken LiDAR'a göre duran her şey geriye kayıyormuş gibi görünür. Araçta tekerlek
sayacı / IMU olmadığı için hareket taramaların kendisinden bulunur: zeminden yüksek noktalar (duvar,
direk, ağaç, araç) bir "anahtar tarama" ile üst üste oturtulur (2B ICP; düz yüzeylerde noktadan
çizgiye, köşelerde noktadan noktaya; hareket eden noktalar sağlam ağırlıkla bastırılır). Sonuç
`/nesne/odometri`'de (`nav_msgs/Odometry`, frame `lidar_odom`) yayınlanır. Eşleştirme başarısızsa o tur
sabit hız varsayılır ve hareketli nokta testi **yapılmaz** (yanlış alarm vermesin); terminalde
`(TAHMIN)` yazar. Masada test ederken `lidar_odometri: false` ile kapatılabilir.

Takip de **yere göre** yapılır: duran bir direğin hızı ~0, kutusu araç dönerken bile dönmez.
Yaklaşma hızı ve çarpışma süresi ise **araca göre**dir: araç 1 m/s giderken önündeki duran engel
1 m/s yaklaşır.

#### Yol koridoru

LiDAR'ın önünde, `koridor_genislik` genişliğinde ve `koridor_uzunluk` uzunluğunda (gövde kutusunun
önünden başlayan) bir şerit. İçinde en az 2 noktası olan **her** nesne, dursa da gösterilir
(`yol_uzerinde: true`, RViz'de kalın kutu + dolgu, yeşil dikdörtgen koridorun kendisi). Şimdilik düz
ileri; direksiyon açısı gelince eğilebilir.

#### Zemin nasıl bulunur ve kontrol edilir?

1. **İlk arama (açılışta):** `zemin_otomatik: true` iken ipucu kullanılmaz. LiDAR'ın en az 30 cm altındaki
   yatay yüzeyler sırayla bulunur (en fazla 5), yeterince geniş olanların (en genişin ≥ %25'i) **en alçağı**
   zemin seçilir. Masa, tezgâh, kaldırım her zaman zeminin üstünde olduğu için onlara kapılmaz.
   `zemin_otomatik: false` iken eskisi gibi sadece `zemin_z ± zemin_bant` içinde aranır.
2. **Takip (sonraki turlar):** sadece bir önceki düzlemin `± zemin_bant` yakınına bakılır. Zemin 2 sn
   üst üste bulunamazsa ilk arama tekrarlanır; o arada son düzlem kullanılır.
3. **Kontrol:** bulunan zemin `zemin_z`'den `zemin_tolerans`'tan fazla farklıysa ya da `egim_uyari`'dan
   eğikse **uyarı** verilir: terminalde, Kontrol Merkezi'nin nesne kartında (kırmızı satır) ve
   `/nesne/zemin_durumu`'nda (`uyumlu: false`). İşleme yine bulunan zeminle devam edilir; uyarı,
   ayarların (ör. `engel_node.z_min`) ve profilin gözden geçirilmesi içindir.

```text
[INFO] Zemin otomatik bulundu: z -0.98 m, egim 0.5 derece (5 yatay yuzeyin en alcagi; beklenen z -0.81)
[WARN] Zemin -0.98 m'de bulundu ama beklenen -0.81 m (fark 17 cm > 10 cm). LiDAR yerinden oynamis
       ya da yanlis profil yuklu olabilir. Olcum dogruysa zemin_z'yi -0.98 yapin.
```

Yani montajdan sonra her seferinde ölçüm yapmak gerekmez: `zemin_z` bir kez (araca takınca) yazılır,
her açılışta kendiliğinden doğrulanır.

> ⚠️ Etiketler **kural tabanlıdır ve kabadır**. "Yaya adayı" kesin bir yaya kararı değildir;
> kutunun boyutuna ve tepe yüksekliğine bakılarak verilen bir tahmindir.

---

## 🚀 Çalıştırma

İki düğümü birlikte (LiDAR sürücüsü başka yerde açıksa ya da kayıt oynatılıyorsa):

```bash
ros2 launch lidar_algilama lidar_algilama.launch.py
```

| Argüman | Varsayılan | Açıklama |
|---|---|---|
| `nesne` | `true` | `nesne_node`'u (kutu / mesafe / takip) aç |

Parametreler önce `config/*.yaml`'dan, sonra varsa ayar panelinin kaydettiği
`~/.ros/kontrol_merkezi/son_ayarlar.yaml`'dan yüklenir.

Sürücü + RViz + düğümler + ayar paneli hepsi birden:
[`ros2 launch kontrol_merkezi lidar_sistem.launch.py`](../../araclar/kontrol_merkezi/README.md#-çalıştırma)

Düğümleri tek tek çalıştırmak:

```bash
ros2 run lidar_algilama engel_node --ros-args --params-file src/algilama/lidar_algilama/config/engel_params.yaml
```

```bash
ros2 run lidar_algilama nesne_node --ros-args --params-file src/algilama/lidar_algilama/config/nesne_params.yaml
```

---

## ⚙️ Parametreler

Tüm parametreler **çalışırken** değiştirilebilir (panelden veya `ros2 param set` ile).
Ondalıklı değerleri YAML'da mutlaka `90.0` gibi noktalı yazın, yoksa ROS tip hatası verir.

Eksenler LiDAR'a göredir: **x = ileri, y = sol, z = yukarı**, birim metre.
Açılar derece: **0 = ön, +90 = sol, −90 = sağ, 180 = arka**.

### `engel_node` — [`config/engel_params.yaml`](config/engel_params.yaml)

| Parametre | Varsayılan | Açıklama |
|---|---|---|
| `merkez_aci` | `0.0` | Yelpazenin baktığı yön (°) |
| `gorus_genisligi` | `90.0` | Yelpazenin toplam açısı (°). `360` = her yön |
| `min_mesafe` | `0.3` | Bundan yakın noktalar yok sayılır (aracın kendisi, masa) |
| `z_min` | `-1.2` | LiDAR merkezinden bu kadar aşağısı yok sayılır — **zeminin biraz üstünde olmalı** |
| `z_max` | `1.0` | LiDAR merkezinden bu kadar yukarısı yok sayılır (tavan, dal, köprü) |
| `dur_mesafesi` | `2.0` | Engel bundan yakınsa **DUR** |
| `dikkat_mesafesi` | `5.0` | Engel bundan yakınsa **DİKKAT**. Daha uzağı işlenmez |
| `min_nokta_sayisi` | `10` | Engel sayılması için en az nokta; mesafe en yakın N'inci noktaya göre ölçülür |
| `canli_izleme` | `true` | Her turda karar + mesafe terminale yazılır |

### `nesne_node` — [`config/nesne_params.yaml`](config/nesne_params.yaml)

Varsayılanlar **masa** düzenine göredir (LiDAR masada, zemin ~0.81 m aşağıda; atölye kaydında ~0.98 m).

<details>
<summary><b>Tüm parametreleri göster</b></summary>

| Grup | Parametre | Varsayılan | Açıklama |
|---|---|---|---|
| Alan | `nesne_merkez_aci` | `0.0` | Kutulama alanının yönü (°) — uyarı yelpazesinden **bağımsız** |
| | `nesne_gorus_genisligi` | `120.0` | Kutulama alanının açısı (°) |
| | `menzil` | `20.0` | Bundan uzak noktalar işlenmez (yatay, m) |
| | `govde_x_min` … `govde_y_max` | masa ölçüleri | Gövde kutusu: içindeki noktalar yok sayılır. Hepsi `0.0` = kapalı |
| | `voxel_boyu` | `0.08` | Nokta seyreltme küp boyu (m). `0.0` = kapalı |
| Hareket | `sadece_dinamik` | `true` | Sadece hareketliler + yol koridorundakiler yayınlanır. Panelde nesne kartındaki **Sadece hareketliler** düğmesi |
| | `koridor_genislik` | `2.0` | Yol koridorunun toplam genişliği (araç + pay, m) |
| | `koridor_uzunluk` | `8.0` | Yol koridoru LiDAR'dan bu kadar ileri (m) |
| | `lidar_odometri` | `true` | Aracın hareketini taramalardan bul. `false` = araç duruyor varsay |
| | `dinamik_bakis_suresi` | `1.0` | Bu kadar önceye kadarki taramalarla karşılaştır (s) |
| | `dinamik_pay` | `0.3` | Geçmiş ışın noktanın en az bu kadar arkasına çarpmış olmalı (m) |
| | `dinamik_min_nokta` / `dinamik_oran` | `4` / `0.3` | Kümenin en az bu kadar / bu oranda noktası hareketli olmalı |
| | `dinamik_tutma` | `3.0` | Bu kadar süre hareket etmeyen nesne tekrar gizlenir (s) |
| Zemin | `zemin_otomatik` | `true` | Açılışta zemini ipucusuz bul (en alçak geniş yatay yüzey) |
| | `zemin_z` | `-0.81` | Beklenen zemin yüksekliği (`-4.0` … `-0.3`). Araçta: `-montaj_yüksekliği`. Otomatikte sadece kontrol için |
| | `zemin_tolerans` | `0.10` | Kontrol: bulunan zemin `zemin_z`'den en fazla bu kadar farklı olabilir (m) |
| | `egim_uyari` | `3.0` | Kontrol: bulunan zemin en fazla bu kadar eğik olabilir (°) |
| | `zemin_bant` | `0.30` | Takipte önceki düzlemin `± bant`'ı aranır (otomatik kapalıysa ilk aramada `zemin_z ± bant`) |
| | `zemin_esik` | `0.10` | Zemin düzlemine bundan yakın nokta zemin sayılır |
| | `zemin_max_egim` | `10.0` | Zemin en fazla bu kadar eğik olabilir (°) |
| | `zemin_min_nokta` | `30` | Zemin düzleminin kabulü için en az nokta |
| | `max_yukseklik` | `2.5` | Zeminden bu kadar yukarısı yok sayılır |
| Kümeleme | `kume_yatay_tol` | `0.35` | Aynı nesne için en fazla yatay aralık (m) |
| | `kume_dikey_taban` / `kume_dikey_artis` | `0.25` / `0.04` | Dikey eşik = taban + artış × mesafe |
| | `kume_min_nokta` / `kume_max_nokta` | `5` / `20000` | Küme boyut sınırları |
| Etiket | `yaya_min_tepe` / `yaya_max_tepe` | `1.0` / `2.1` | Yaya adayı: tepe noktası zeminden bu aralıkta |
| | `yaya_max_uzunluk` / `yaya_max_genislik` | `1.2` / `1.0` | …ve taban en fazla bu boyutta |
| | `yapi_min_uzunluk` / `yapi_max_genislik` | `2.5` / `0.5` | Yapı: uzun ve ince |
| | `yapi_kesin_uzunluk` | `6.0` | …ya da bundan uzun her şey |
| Takip | `takip_kapi` | `1.0` | Bir turda en fazla bu kadar yer değiştirebilir (m, yere göre) |
| | `takip_dogrulama` | `3` | Yeni nesne bu kadar tur üst üste görülünce numara alır |
| | `takip_kayip` | `5` | Bu kadar tur görülmeyen nesne silinir |
| | `kutu_yumusatma` | `0.7` | Kutu titremesini azaltır: merkez Kalman'dan, boyut/yön yavaşça güncellenir. `0` = ham kutu. Mesafe yumuşatılmaz |
| | `canli_izleme` | `true` | Saniyede bir özet satırı |

</details>

Çalışırken değiştirme örneği:

```bash
ros2 param set /engel_node dur_mesafesi 1.5
```

```bash
ros2 param set /nesne_node zemin_z -1.80
```

---

## 📡 Topic'ler ve mesajlar

### Giriş

| Topic | Tip | Açıklama |
|---|---|---|
| `/rslidar_points` | `sensor_msgs/PointCloud2` | LiDAR nokta bulutu (frame `rslidar`, ~10 Hz, SensorData QoS) |

### `engel_node` çıkışları

| Topic | Tip | Açıklama |
|---|---|---|
| `/engel/durum` | `robotaksi_msgs/EngelDurumu` | Bölge durumu (`GUVENLI` / `DIKKAT` / `DUR`), mesafe, durumu belirleyen nokta |
| `/engel/filtrelenmis_noktalar` | `sensor_msgs/PointCloud2` | Tüm filtrelerden geçen noktalar |
| `/engel/gorsel` | `visualization_msgs/MarkerArray` | Yelpaze, DUR/DİKKAT yayları, karar yazısı |

### `nesne_node` çıkışları

| Topic | Tip | Açıklama |
|---|---|---|
| `/nesne/liste` | `robotaksi_msgs/NesneDizisi` | Hareketli ve yol koridorundaki nesneler (`sadece_dinamik: false` ise hepsi) |
| `/nesne/odometri` | `nav_msgs/Odometry` | Aracın (LiDAR'ın) yere göre konumu ve hızı; frame `lidar_odom` |
| `/nesne/dinamik_noktalar` | `sensor_msgs/PointCloud2` | Hareketli bulunan noktalar (ayar yaparken RViz'de "Hareketli noktalar") |
| `/nesne/gorsel` | `visualization_msgs/MarkerArray` | Kutular, numara/etiket/mesafe yazıları, hız okları |
| `/nesne/kumeler` | `sensor_msgs/PointCloud2` | Her nesne ayrı renkte noktalar |
| `/nesne/zemin` | `sensor_msgs/PointCloud2` | Zemin olarak ayrılan noktalar |
| `/nesne/zemin_disi` | `sensor_msgs/PointCloud2` | Zemin dışında kalan noktalar |
| `/nesne/zemin_durumu` | `robotaksi_msgs/ZeminDurumu` | Kullanılan zemin düzlemi (yükseklik, eğim, katsayılar) ve beklenenle uyumu |

### `robotaksi_msgs/EngelDurumu`

```
uint8 GUVENLI = 0
uint8 DIKKAT = 1
uint8 DUR = 2

std_msgs/Header header             # gelen nokta bulutunun başlığı (zaman + frame)
uint8 durum                        # GUVENLI | DIKKAT | DUR (histerezisli)
bool engel_var                     # yelpazede en az min_nokta_sayisi nokta var mı
float32 mesafe                     # en yakın N'inci noktanın mesafesi (m); engel yoksa +inf
geometry_msgs/Point engel_noktasi  # durumu belirleyen nokta; engel yoksa (0, 0, 0)
```

Durumu metinle değil sabitle karşılaştırın; yazım hatası derleme / çalışma anında yakalanır:

```cpp
if (msg.durum == robotaksi_msgs::msg::EngelDurumu::DUR) { ... }
```

```python
if msg.durum == EngelDurumu.DUR: ...
```

Engel yoksa `mesafe` `+inf`'tir, böylece `mesafe < esik` gibi bir kontrol yanlışlıkla doğru çıkmaz.

### `robotaksi_msgs/Nesne`

```
uint32 id                          # takip numarası
string etiket                      # "yaya_adayi" | "yapi" | "engel"
float32 guven                      # 0..1, kural tabanlı güven puanı

geometry_msgs/Point merkez         # kutunun merkezi
geometry_msgs/Vector3 boyut        # x = uzunluk, y = genişlik, z = yükseklik
float32 yon                        # kutunun z ekseni etrafındaki açısı (radyan)

geometry_msgs/Point en_yakin_nokta # LiDAR'a en yakın nokta
float32 mesafe                     # en yakın noktanın yatay mesafesi (m)
float32 tepe_yuksekligi            # en üst noktanın zemin üstü yüksekliği (m)

geometry_msgs/Vector3 hiz          # YERE göre hız (m/s); hareketli olmayanlarda 0
bool hareketli                     # hareket ediyor (ya da son dinamik_tutma sn içinde etti)
float32 yaklasma_hizi              # m/s, ARACA göre, pozitif = yaklaşıyor
float32 carpisma_suresi            # saniye; yaklaşmıyorsa -1

bool yol_uzerinde                  # aracın önündeki yol koridorunda mı
bool uyari_alaninda                # engel_node'un uyarı yelpazesinin içinde mi
uint32 nokta_sayisi
uint32 yas                         # kaç turdur takipte
```

Hızlıca izlemek için:

```bash
ros2 topic echo /engel/durum
```

```bash
ros2 topic echo /nesne/liste --field nesneler
```

---

## 🚗 Masadan araca geçerken

Varsayılan değerler LiDAR masadayken ölçüldü. Araca takarken en az şunları güncelleyin:

| Ayar | Masada | Araçta (örnek: LiDAR yerden 1.8 m) |
|---|---|---|
| `engel_node.z_min` | `-0.1` … `-0.6` | `-1.6` civarı (zeminin ~20 cm üstü) |
| `engel_node.min_mesafe` | `~1.0` | Aracın gövdesi kadar |
| `nesne_node.zemin_z` | `-0.81` | `-1.80` (zemin otomatik bulunur; bu değer kontrol içindir, tutmazsa uyarı çıkar) |
| `nesne_node.govde_*` | masa ölçüleri | Aracın LiDAR'a göre kutusu |
| `nesne_node.koridor_genislik` | `2.0` | Araç genişliği + iki yanda ~0.3 m pay |
| `nesne_node.nesne_gorus_genisligi` | `120` | Yandan karşıya geçen yayalar için geniş tutun (180–360) |

> 💡 `z_min` çok aşağıda kalırsa **zemin engel sanılır** ve sürekli DUR alırsınız. Panelde *Alt sınır (zemin)*
> çubuğunu, RViz'de zemin noktaları kaybolana kadar yukarı alın.

Panelde **Kaydet…** ile `masa` ve `arac` diye iki profil oluşturup ortam değiştikçe tek tıkla geçebilirsiniz.

---

## 🩺 Sorun giderme

| Belirti | Olası neden / çözüm |
|---|---|
| RViz'de nokta yok | Ethernet IP'si yanlış veya `lidar_type` hatalı. `ros2 topic hz /rslidar_points` ile kontrol edin |
| Node'lar veri almıyor / takılıyor | Fast DDS büyük mesaj ayarı eksik. `FASTRTPS_DEFAULT_PROFILES_FILE` değişkenini verin (kök [README → Kurulum](../../../README.md#-kurulum)) |
| Sürekli DUR | `z_min` zemine çok yakın, ya da `min_mesafe` aracın/masanın kendisini içine alıyor |
| Duran nesneler "hareketli" görünüyor | RViz'de "Hareketli noktalar"ı açın. Araç giderken çoksa terminalde `(TAHMIN)` var mı bakın (odometri tutmuyor). Az ise `dinamik_pay`'ı büyütün (0.5) ya da `dinamik_oran`'ı artırın |
| Yayalar geç görünüyor / hiç görünmüyor | `dinamik_oran`'ı (0.2) ya da `dinamik_min_nokta`'yı azaltın; LiDAR'dan doğrudan uzaklaşan yaya yakalanmaz (bkz. "Hareketli nesne nasıl ayırt edilir?") |
| Duvarlar YAYA?, insanlar ENGEL | Zemin yanlış yerde (ör. masa yüzeyi). Nesne kartındaki zemin satırına / `ros2 topic echo /nesne/zemin_durumu` çıktısına bakın |
| Küçük cisimler kaçıyor | `min_nokta_sayisi`'nı azaltın |
| Toz/yağmurda yanlış DUR | `min_nokta_sayisi`'nı artırın (20–30) |
| Ondalıklı parametrede "tip uyuşmuyor" | YAML'da `90` yerine `90.0` yazın |
| Başka bilgisayarın ROS verisi karışıyor | `ROS_DOMAIN_ID` ve `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` |
| PCL derlenirken `CMP0144 / FLANN_ROOT` uyarısı | PCL'in kendi CMake dosyasından gelir, zararsızdır |

---

## 🗂️ Paket yapısı

```
lidar_algilama/
├── config/
│   ├── engel_params.yaml        # engel_node varsayılanları
│   └── nesne_params.yaml        # nesne_node varsayılanları
├── launch/
│   └── lidar_algilama.launch.py # iki düğüm
└── src/
    ├── engel_node.cpp           # sektör tabanlı engel durumu (DUR / DİKKAT / GÜVENLİ)
    ├── nesne_node.cpp           # nesne algılamanın ROS tarafı
    └── nesne/                   # ROS'tan bağımsız algoritmalar
        ├── zemin.*              #   RANSAC zemin ayırma
        ├── odometri.*           #   aracın hareketi (2B ICP)
        ├── dinamik.*            #   hareketli nokta testi (görünürlük)
        ├── kumeleme.*           #   nokta kümeleme
        ├── kutu.*               #   yönlü kutu + etiket
        └── takip.*              #   nesne takibi
```

Mesajlar (`EngelDurumu`, `Nesne`, `NesneDizisi`, `ZeminDurumu`) ortak [`robotaksi_msgs`](../../ortak/robotaksi_msgs/) paketindedir.
