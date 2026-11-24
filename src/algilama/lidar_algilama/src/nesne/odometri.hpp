// =============================================================================
// odometri.hpp — Aracın kendi hareketini LiDAR taramalarından bulur
//
// Neden gerekli? Araç 1 m/s giderken LiDAR'a göre DURAN her şey 1 m/s ile geriye
// kayıyormuş gibi görünür. "Bu nesne hareket ediyor mu?" sorusunu cevaplamak için
// önce aracın kendi hareketini çıkarmak gerekir. Araçta tekerlek sayacı / IMU
// olmadığı için hareketi taramaların kendisinden buluyoruz.
//
// Yöntem (2 boyutlu ICP, "en yakın nokta ile hizalama"):
//   - Zeminden yüksek, seyreltilmiş noktalar kullanılır (duvar, direk, ağaç, araç).
//   - Bir "anahtar tarama" saklanır (dünya çerçevesinde). Yeni tarama, sabit hız
//     tahminiyle kaydırılıp anahtar taramaya yaklaştırılır; her noktanın en yakın
//     komşusu bulunur, aradaki farkı en küçük yapan kaydırma + dönüş hesaplanır;
//     birkaç tur tekrarlanır.
//   - Duvar gibi düz yüzeylerde "noktadan çizgiye" mesafe kullanılır (duvar boyunca
//     kayma serbest, duvara dik kayma cezalı); köşe/bitki gibi yerlerde "noktadan
//     noktaya". Uzun koridor gibi bir yönün belirsiz kaldığı yerlerde o yön sabit hız
//     tahmininde kalır.
//   - Hareket eden nesnelerin (yaya) noktaları eşleşmeyi bozmasın diye uzak
//     eşleşmeler atılır, kalanlar "sağlam" ağırlıkla (Cauchy) tartılır.
//   - Araç anahtar taramadan 0.6 m / 6° uzaklaşınca ya da 3 sn geçince yeni anahtar
//     tarama alınır. Durunca anahtar değişmediği için konum kaymaz.
//
// Sonuç kısa sürede tutarlıdır (birkaç saniyede birkaç cm); uzun yolda yavaşça kayar.
// Bu iş için yeterli: dinamik nesne kararı ve takip son 1-3 saniyeye bakar.
// =============================================================================
#pragma once

#include <cstdint>
#include <vector>

#include <pcl/kdtree/kdtree_flann.h>

#include "nesne/tipler.hpp"

namespace nesne
{

struct OdometriSonucu
{
  Poz2 poz;                 // LiDAR'ın dünya çerçevesindeki konumu ve yönü
  float vx = 0.0f, vy = 0.0f;  // aracın hızı, LiDAR eksenlerinde (x ileri, y sol), m/s
  float wz = 0.0f;          // dönüş hızı, rad/s (+ = sola)
  bool gecerli = false;     // eşleştirme başarılı (false: sabit hız tahmini kullanıldı)
  bool sifirlandi = false;  // geçmiş koptu (ilk tur / zaman atladı): takip ve geçmiş sıfırlanmalı
  int eslesme = 0;          // eşleşen nokta sayısı
  float hata = 0.0f;        // eşleşen noktaların ortalama kalan farkı (m)
};

class LidarOdometri
{
public:
  // bulut: zeminden yüksek, seyreltilmiş noktalar (LiDAR çerçevesi)
  // zaman_s: nokta bulutunun zaman damgası
  OdometriSonucu guncelle(const Bulut & bulut, double zaman_s);

  void sifirla();

private:
  void anahtarYap(const Bulut & bulut);

  // Anahtar tarama: dünya çerçevesinde, z ekseni Z_OLCEK ile küçültülmüş (en yakın
  // komşu aranırken yatay yakınlık öne çıksın, ama farklı yükseklikteki katmanlar
  // birbirine karışmasın)
  Bulut::Ptr anahtar_;
  pcl::KdTreeFLANN<Nokta> agac_;
  std::vector<float> nx_, ny_;     // anahtar noktanın yatay normali (cizgi_ ise)
  std::vector<uint8_t> cizgi_;     // 1: nokta düz bir yüzeyde (noktadan çizgiye mesafe)
  Poz2 anahtar_poz_;
  double anahtar_zaman_ = -1.0;

  Poz2 poz_;
  double son_zaman_ = -1.0;
  double vx_ = 0.0, vy_ = 0.0, wz_ = 0.0;  // dünya çerçevesinde hız (sabit hız tahmini için)
  int ust_uste_hata_ = 0;
};

}  // namespace nesne
