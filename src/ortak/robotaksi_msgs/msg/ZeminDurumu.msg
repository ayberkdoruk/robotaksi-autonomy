# nesne_node'un kullandigi zemin duzlemi ve beklenenle karsilastirmasi (/nesne/zemin_durumu).
# Her nokta bulutunda bir kez yayinlanir. "uyumlu" false ise ya zemin bulunamamistir ya da
# bulunan zemin ayarlardaki beklenen yukseklik / egimle tutmuyordur (LiDAR yerinden oynamis
# veya yanlis profil yuklu olabilir).
# Duzlem LiDAR frame'inde (rslidar): a*x + b*y + c*z + d = 0; (a, b, c) birim ve yukari bakar,
# bu yuzden bir noktayi denkleme koyunca cikan sayi o noktanin zeminden yuksekligidir (metre).

uint8 OLCULDU = 0       # bu turda olculdu
uint8 ONCEKI = 1        # bu turda bulunamadi, son olculen duzlem kullaniliyor
uint8 VARSAYILAN = 2    # hic olculemedi, beklenen yukseklikte duz zemin varsayiliyor

std_msgs/Header header        # gelen nokta bulutunun basligi
uint8 kaynak                  # OLCULDU | ONCEKI | VARSAYILAN
bool otomatik                 # true: zemin ipucusuz arandi (zemin_otomatik parametresi)
float32 yukseklik             # zeminin LiDAR'in tam altindaki z'si (m), ornek -0.81
float32 egim                  # zemin duzleminin yataydan sapmasi (derece)
float32[4] duzlem             # a, b, c, d
float32 beklenen_yukseklik    # zemin_z parametresi (m)
float32 yukseklik_toleransi   # zemin_tolerans parametresi (m)
float32 egim_siniri           # egim_uyari parametresi (derece)
bool uyumlu                   # kaynak != VARSAYILAN ve yukseklik/egim sinirlar icinde
