# surus/

Sürüş yöntemleri. Ekipte herkes kendi yöntemini **ayrı bir pakette** geliştirir (ör. `lidar_surus`,
`gps_surus`); en iyisi yarışmada kullanılır.

Kurallar:
- Yönteminiz sadece kendi paketinin içinde yaşar; `ortak/`, `surucular/`, `arac/` paketlerini
  değiştirmeniz gerekiyorsa ekiple konuşun.
- Sensörlerden ve algılama paketlerinden (`algilama/`) istediğiniz topic'i okuyun.
- Araca komutu, `arac/` katmanının dinlediği ortak komut topic'ine yazın (tanımlanınca).
- Paket adı yöntemi anlatsın, kişiyi değil: `lidar_surus`, `ayberk_surus` değil.

Henüz paket yok.
