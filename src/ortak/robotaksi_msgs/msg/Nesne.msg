# LiDAR'dan bulunan tek bir nesne (takip edilen, dogrulanmis).
# Tum konumlar LiDAR frame'inde (rslidar): x = ileri, y = sol, z = yukari, metre.

uint32 id                          # takip numarasi (nesne kayboldukca yeniden kullanilmaz)
string etiket                      # "yaya_adayi" | "yapi" | "engel"  (kaba, kural tabanli)
float32 guven                      # 0..1, kural tabanli guven puani

geometry_msgs/Point merkez         # kutunun merkezi
geometry_msgs/Vector3 boyut        # x = uzunluk, y = genislik (uzunluk >= genislik), z = yukseklik
float32 yon                        # kutunun z ekseni etrafindaki acisi (radyan, uzunluk yonu)

geometry_msgs/Point en_yakin_nokta # nesnenin LiDAR'a en yakin noktasi
float32 mesafe                     # en yakin noktanin yatay mesafesi (metre)
float32 tepe_yuksekligi            # en ust noktanin zemin ustu yuksekligi (metre)

geometry_msgs/Vector3 hiz          # m/s (z = 0)
float32 yaklasma_hizi              # m/s, pozitif = LiDAR'a yaklasiyor
float32 carpisma_suresi            # saniye; yaklasmiyorsa -1

bool uyari_alaninda                # en yakin noktasi engel_node'un uyari yelpazesinin icinde mi
uint32 nokta_sayisi                # kumedeki nokta sayisi (voxel sonrasi)
uint32 yas                         # kac turdur takipte
