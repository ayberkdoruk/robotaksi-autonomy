# surucular/

Sensör sürücüleri: donanımdan veriyi alıp ROS topic'i olarak yayınlar. Ekipteki herkes aynı
sürücüleri kullanır.

| Klasör | İçerik |
|---|---|
| `rslidar_sdk`, `rslidar_msg` | RoboSense LiDAR sürücüsü (dış depo, bu depoda takip edilmez; `robotaksi.repos` ile çekilir) |
| `yamalar/` | Dış sürücülere uyguladığımız değişiklikler. `rslidar_sdk_helios.patch`: Helios 32 için `lidar_type: RSHELIOS` ve `POINT_TYPE XYZIRT` |

İleride: ZED kamera, GPS, encoder sürücüleri. Dış depodan gelen bir sürücüyü `robotaksi.repos`'a
ekleyin; değişiklik gerekirse dosyayı elle değiştirip bırakmak yerine `yamalar/` altına yama koyun.
