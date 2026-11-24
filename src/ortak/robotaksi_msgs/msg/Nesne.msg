# LiDAR'dan bulunan tek bir nesne (takip edilen, dogrulanmis).
# Tum konumlar LiDAR frame'inde (rslidar): x = ileri, y = sol, z = yukari, metre.

uint32 id                          # takip numarasi (nesne kayboldukca yeniden kullanilmaz)
string etiket                      # "yaya_adayi" | "yapi" | "engel"  (kaba, kural tabanli)
float32 guven                      # 0..1, kural tabanli guven puani

# merkez / boyut / yon takipte yumusatilir (nesne_node kutu_yumusatma); en_yakin_nokta ve
# mesafe ise her zaman o turun ham olcumudur (guvenlik icin gecikmez).
geometry_msgs/Point merkez         # kutunun merkezi
geometry_msgs/Vector3 boyut        # x = uzunluk, y = genislik (uzunluk >= genislik), z = yukseklik
float32 yon                        # kutunun z ekseni etrafindaki acisi (radyan, uzunluk yonu)

geometry_msgs/Point en_yakin_nokta # nesnenin LiDAR'a en yakin noktasi
float32 mesafe                     # en yakin noktanin yatay mesafesi (metre)
float32 tepe_yuksekligi            # en ust noktanin zemin ustu yuksekligi (metre)

geometry_msgs/Vector3 hiz          # YERE gore hiz (aracin hareketi cikarilmis), LiDAR eksenlerinde,
                                   # m/s (z = 0). Hareketli olmayan nesnelerde 0.
bool hareketli                     # hareket ediyor (gorunurluk testi: bulundugu yer az once bostu)
                                   # ya da son dinamik_tutma saniyesi icinde hareket etti (nesne_node)
float32 yaklasma_hizi              # m/s, ARACA gore, pozitif = yaklasiyor. Arac 1 m/s giderken
                                   # onundeki duran engel icin ~1.0
float32 carpisma_suresi            # saniye; yaklasmiyorsa -1

bool yol_uzerinde                  # nesnenin noktalari aracin onundeki yol koridorunda mi
                                   # (koridor_genislik x koridor_uzunluk). Koridordaki nesneler
                                   # duruyor olsalar bile yayinlanir.
bool uyari_alaninda                # en yakin noktasi engel_node'un uyari yelpazesinin icinde mi
uint32 nokta_sayisi                # kumedeki nokta sayisi (voxel sonrasi)
uint32 yas                         # kac turdur takipte
