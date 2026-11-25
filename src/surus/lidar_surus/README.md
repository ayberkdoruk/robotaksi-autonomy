# lidar_surus

LiDAR ile sürüş yöntemi: **düz git, önüne engel ya da yaya gelirse dur**, yol boşalınca devam et.

```
/engel/durum (engel_node) ──┐
                            ├─► lidar_surus ──/arac/komut──► serial_comm_node ──► Teensy
/nesne/liste (nesne_node) ──┘        └──/lidar_surus/durum (karar + sebep)
```

## Karar

Her 50 ms'de bir, ilk tutan kazanır:

| Durum | Ne zaman | Araca |
|---|---|---|
| `BEKLIYOR` | `surus_aktif: false` (açılış) | fren |
| `SURE_DOLDU` | `azami_surus_suresi` doldu | fren |
| `VERI_YOK` | `/engel/durum`, `/nesne/liste` ya da (yön tutma açıksa) yön verisi 0.5 sn'dir gelmiyor | fren |
| `DUR` | engel_node **DUR** (yelpazede `dur_mesafesi`'nden yakın) ya da yol koridorunda `nesne_dur_mesafesi`'nden yakın / `carpisma_suresi_dur`'dan kısa sürede çarpacak nesne | fren |
| `KALKIS_BEKLE` | son DUR / VERI_YOK'tan (ya da aktif edildiğinden) beri `kalkis_bekleme` geçmedi | fren |
| `DIKKAT` | engel_node **DİKKAT** ya da koridorda `nesne_dikkat_mesafesi`'nden yakın nesne | gaz (`dikkatte_gaz_kes: true` ise gaz kesilir) |
| `GIDIYOR` | yol boş | gaz |

Direksiyon: `yon_tutma: kapali` ise hep `duz_direksiyon` (84). `lidar` / `imu` ise sürüş
başladığı andaki yön hedef alınır ve sapma düzeltilir:
`direksiyon = duz_direksiyon − yon_kazanc × (hedef_yon − yon)` (en fazla ± `yon_siniri`).
Yön `/nesne/odometri`'den (LiDAR) ya da `imu_topic`'ten (ZED 2 IMU) okunur. Seçilen kaynak
0.5 sn gelmezse `VERI_YOK` (fren). Parametreler:
[`config/lidar_surus_params.yaml`](config/lidar_surus_params.yaml); hepsi çalışırken
`ros2 param set /lidar_surus <ad> <değer>` ile değişir.

## Çalıştırma

```bash
ros2 launch lidar_surus lidar_surus.launch.py                  # araçta: sürücü + algılama + araç + lidar_surus
ros2 launch lidar_surus lidar_surus.launch.py kuru:=true       # Teensy'ye yazmadan (masada)
ros2 launch lidar_surus lidar_surus.launch.py algilama:=false  # algılama Kontrol Merkezi'nden açıksa
ros2 launch lidar_surus lidar_surus.launch.py surucu:=false    # kayıt oynatırken
```

```bash
ros2 param set /lidar_surus surus_aktif true    # KALK
ros2 param set /lidar_surus surus_aktif false   # DUR
ros2 topic echo /lidar_surus/durum              # karar + sebep
ros2 topic echo /arac/gonderilen                # Teensy'ye giden
```
