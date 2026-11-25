# Bir sürüş yönteminin araca verdiği komut (/arac/komut). arac_haberlesme'deki serial_comm_node
# bunu Teensy'nin 5 byte'lık paketine çevirir; araç hangi yöntemin sürdüğünü bilmez.
#
# SÜREKLİ YAYINLAYIN (en az 10 Hz, önerilen 20 Hz): serial_comm_node son komut
# "komut_zaman_asimi" (varsayılan 0.3 sn) içinde gelmezse gazı keser ve FREN basar. Yöntem
# çökerse ya da donarsa araç böylece kendiliğinden durur.
#
# Aynı serial_comm_node, Teensy'ye gerçekten gönderdiği değeri de bu tiple /arac/gonderilen
# topic'ine yazar (kaynak = "manuel" | "otonom" | "fren:<sebep>").

int16 DIREKSIYON_SOL = 25          # tam sol (Teensy servo birimi)
int16 DIREKSIYON_SAG = 165         # tam sağ. Fiziksel düz ~84 (araca göre kalibre edilir)

std_msgs/Header header
string kaynak                      # komutu veren yöntem, ör. "lidar_surus" (izleme için)
int16 direksiyon                   # DIREKSIYON_SOL..DIREKSIYON_SAG; dışındakiler kırpılır
bool gaz                           # Teensy gazı aç/kapa (sabit hız, ara değer yok)
bool fren                          # fren > gaz: ikisi birden true ise sadece fren uygulanır
bool geri                          # geri vites; firmware kendi güvenli vites sırasını uygular
