# engel_node'un secili yelpazedeki en yakin engele gore verdigi bolge durumu (/engel/durum).
# Bu bir surus komutu DEGILDIR: "engel hangi bolgede" der; ne yapilacagina surus yontemi karar verir.
# Konumlar LiDAR frame'inde (rslidar): x = ileri, y = sol, z = yukari, metre.

uint8 GUVENLI = 0                  # yelpazede engel yok ya da dikkat_mesafesi'nden uzak
uint8 DIKKAT = 1                   # engel dikkat_mesafesi'nden yakin
uint8 DUR = 2                      # engel dur_mesafesi'nden yakin

std_msgs/Header header             # gelen nokta bulutunun basligi (zaman + frame)
uint8 durum                        # GUVENLI | DIKKAT | DUR (histerezisli)
bool engel_var                     # yelpazede en az min_nokta_sayisi nokta var mi
float32 mesafe                     # en yakin N'inci noktanin mesafesi (m); engel yoksa +inf
geometry_msgs/Point engel_noktasi  # durumu belirleyen nokta; engel yoksa (0, 0, 0)
