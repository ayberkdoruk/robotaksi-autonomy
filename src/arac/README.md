# arac/

Araca komut gönderen katman: bilgisayar ile araç arasındaki köprü (geçen sene: `serial_comm_node`
ile Teensy'ye seri port). Sürüş yöntemleri araca doğrudan değil, bu katmanın dinlediği **ortak komut
topic'i** üzerinden konuşur; böylece hangi yöntemin sürdüğü araç tarafında fark etmez.

Henüz paket yok. Ortak komut mesajı tanımlandığında `ortak/robotaksi_msgs` içine eklenecek.
