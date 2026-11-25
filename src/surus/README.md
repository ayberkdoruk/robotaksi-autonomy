# surus/

Sürüş yöntemleri. Ekipte herkes kendi yöntemini **ayrı bir pakette** geliştirir (ör. `lidar_surus`,
`gps_surus`); en iyisi yarışmada kullanılır.

Kurallar:
- Yönteminiz sadece kendi paketinin içinde yaşar; `ortak/`, `surucular/`, `arac/` paketlerini
  değiştirmeniz gerekiyorsa ekiple konuşun.
- Sensörlerden ve algılama paketlerinden (`algilama/`) istediğiniz topic'i okuyun.
- Araca komutu `/arac/komut` topic'ine ([`robotaksi_msgs/AracKomutu`](../ortak/robotaksi_msgs/msg/AracKomutu.msg))
  **sürekli** (≥ 10 Hz) yazın; 0.3 sn yazmazsanız araç fren yapar.
- Paket adı yöntemi anlatsın, kişiyi değil: `lidar_surus`, `ayberk_surus` değil.

| Paket | Yöntem |
|---|---|
| [`lidar_surus`](lidar_surus/README.md) | Düz git, yelpazede engel ya da yol koridorunda nesne (yaya) görünce dur |
