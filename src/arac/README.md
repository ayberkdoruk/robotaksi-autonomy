# arac/

Araca komut gönderen katman: bilgisayar ile araç arasındaki köprü (geçen sene: `serial_comm_node`
ile Teensy'ye seri port). Sürüş yöntemleri araca doğrudan değil, bu katmanın dinlediği **ortak komut
topic'i** üzerinden konuşur; böylece hangi yöntemin sürdüğü araç tarafında fark etmez.

| Paket | İçerik |
|---|---|
| [`arac_haberlesme`](arac_haberlesme/README.md) | `serial_comm_node` (Teensy'ye yazan tek düğüm) + `gamepad_node` (kumanda) |

Ortak komut: `/arac/komut` ([`robotaksi_msgs/AracKomutu`](../ortak/robotaksi_msgs/msg/AracKomutu.msg)).
Sürekli yayınlanmalı (≥ 10 Hz); 0.3 sn gelmezse araç fren yapar.
