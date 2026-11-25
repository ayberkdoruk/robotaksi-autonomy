# arac_haberlesme

Bilgisayar ile araç (Teensy) arasındaki köprü. Teensy'ye yazan **tek** düğüm buradadır; sürüş
yöntemleri araca doğrudan değil, `/arac/komut` topic'i üzerinden konuşur.

| Düğüm | Ne yapar |
|---|---|
| `serial_comm_node` | `/arac/komut` (otonom) ve `/gamepad/state` (kumanda) birleştirir, 50 Hz'te Teensy'ye 5 byte'lık paket yollar |
| `gamepad_node` | Logitech F710'u okur, `/gamepad/state` yayınlar. Seri porta yazmaz |

Ekibin geçen seneki `serial_comm_node`'undan (2026-08) alındı. Değişenler: otonom girdi tek topic
(`/arac/komut`; eski `/steering_angle_topic` ve `speed_on` / `brake_on` / `reverse_on` / `gear_toggle`
servisleri kaldırıldı), komut zaman aşımında fren, açılışta gaz kapalı, manuelde kumanda kopunca
fren, Teensy yokken masada deneme (`kuru_calisma`).

## Kim sürüyor?

| Durum | Teensy'ye giden |
|---|---|
| Kumanda bağlı, **LB = MANUEL** | Her şey kumandadan (sağ stick direksiyon, sol stick ileri gaz, RT fren, RB vites) |
| Kumanda bağlı, **LB = OTONOM** | `/arac/komut` |
| Kumanda hiç bağlanmadı | `/arac/komut` |
| Kumanda OTONOM'dayken koptu | `/arac/komut` (`otonomda_kumanda_zorunlu: true` ise fren) |
| Kumanda MANUEL'deyken koptu | **Fren** |
| Otonomda `/arac/komut` 0.3 sn'dir gelmiyor | **Fren** |
| `serial_comm_node` kapandı / USB koptu | Teensy 300 ms paket alamayınca kendisi frenler |

Paket: `0xAA | direksiyon(25–165) | bayraklar | (direksiyon ^ bayraklar) | 0x55`, bayraklar
bit0 = gaz, bit1 = fren, bit2 = geri. Otonom direksiyon 5'lik bloklara yuvarlanır (84 → 82).

## Topic'ler

| Topic | Tip | Yön |
|---|---|---|
| `/arac/komut` | `robotaksi_msgs/AracKomutu` | girdi: sürüş yönteminin komutu (≥ 10 Hz) |
| `/gamepad/state` | `std_msgs/Int32MultiArray` | girdi: `[direksiyon, gaz, fren, geri, otonom, acil]` |
| `/arac/gonderilen` | `robotaksi_msgs/AracKomutu` | çıktı: Teensy'ye gerçekten giden (50 Hz), `kaynak` = `manuel` / `otonom` / `fren:<sebep>` |
| `/arac/surus_modu` | `std_msgs/Int32` | çıktı: 0 paket yok, 1 manuel, 2 otonom, 3 güvenlik freni |
| `/arac/baglanti` | `std_msgs/Bool` (latched) | çıktı: Teensy portu açık mı |

## Çalıştırma

```bash
ros2 launch arac_haberlesme arac.launch.py                 # Teensy + kumanda
ros2 launch arac_haberlesme arac.launch.py kumanda:=false  # kumandasız
ros2 launch arac_haberlesme arac.launch.py kuru:=true      # Teensy'ye yazmaz (masada deneme)
```

Parametreler: [`config/arac_params.yaml`](config/arac_params.yaml).

## İlk kurulum (araçtaki bilgisayarda bir kez)

```bash
sudo usermod -aG dialout $USER     # Teensy portuna yazma izni; sonra oturumu kapatıp açın
sudo apt install python3-pygame    # gamepad_node için
```

Teensy takılıyken `ls -l /dev/serial/by-id/` içinde `...Teensyduino...` görünmeli. Kumandanın ön
anahtarı **X** konumunda olmalı.
