// =============================================================================
// dinamik.hpp — Hangi noktalar hareket eden bir nesneye ait? (görünürlük testi)
//
// Fikir: LiDAR ışını bir yere çarpıp geri döner; ışının geçtiği yol BOŞTUR. Şu an
// bir noktanın bulunduğu yer, 1 saniye önceki taramada boş görülmüşse (o yöne giden
// ışın noktanın çok ARKASINDAN dönmüşse), o noktadaki cisim oraya sonradan gelmiştir:
// hareket ediyordur.
//
//      1 sn önce:   LiDAR ----------------------------> duvar   (ışın boşluktan geçti)
//      şimdi:       LiDAR ----------> yaya                       (aynı yerde artık yaya var)
//
// Duran cisimler bu testi geçemez: o yöne giden ışın ya cismin kendisine ya da önündeki
// bir şeye çarpmıştır. Küme merkezinin kaymasına bakmadığı için, duran nesnelerin
// kümeleri birleşip ayrılınca (merkez 0.6-0.8 m sıçrarken) yanlışlıkla "hareketli"
// denmez; araç giderken nesnenin görünen yüzü değişince de denmez.
//
// Araç hareket ediyorsa geçmiş taramanın nereden çekildiği odometriden bilinir; nokta
// önce dünyaya, sonra geçmiş taramanın LiDAR çerçevesine taşınıp bakılır.
//
// Geçmiş taramalar "menzil görüntüsü" olarak saklanır: yatayda 0.5°, dikeyde 1°'lik
// hücreler, her hücrede o yöne düşen en yakın nokta. Karşılaştırmada noktanın etrafındaki
// birkaç hücreye bakılır ve HEPSİ boş görmüş olmalıdır (kenarlarda yanlış alarm olmasın).
// Hiç dönüş olmayan hücre (gökyüzü, ışığı yutan yüzey) "bilinmiyor" sayılır.
//
// Sınır: LiDAR'dan doğrudan UZAKLAŞAN bir nesne bu testle yakalanmaz (yeni yeri, eski
// halinin arkasında kalır, görülmemiştir). Yol koridorundaki nesneler zaten her zaman
// gösterildiği için aracın önünde giden biri kaçmaz.
// =============================================================================
#pragma once

#include <cstdint>
#include <deque>
#include <vector>

#include "nesne/tipler.hpp"

namespace nesne
{

struct DinamikAyari
{
  float bakis_suresi = 1.0f;  // en eski karşılaştırılan tarama bu kadar önceki (s)
  float pay = 0.3f;           // geçmiş ışın noktanın en az bu kadar arkasından dönmüş olmalı (m)
};

class DinamikAyirici
{
public:
  // Şu anki noktaları geçmiş taramalarla karşılaştırır.
  //   noktalar: LiDAR çerçevesi (genelde zemin dışı noktalar)
  //   dinamik : her nokta için 1 = hareket eden bir cisme ait
  // Dönüş: karşılaştırılabilen geçmiş tarama sayısı (0 ise hiçbir nokta işaretlenmez)
  int isaretle(const Bulut & noktalar, const Poz2 & poz, double zaman_s, const DinamikAyari & a,
    std::vector<uint8_t> & dinamik) const;

  // Bu turun taramasını geçmişe ekler (isaretle'den SONRA çağrılmalı).
  //   tum     : bu turun bütün noktaları (zemin dahil; zemin de ışını durdurur)
  //   gecerli : odometri bu tur başarılıydı (değilse karşılaştırmada kullanılmaz)
  void ekle(const Bulut & tum, const Poz2 & poz, double zaman_s, bool gecerli,
    const DinamikAyari & a);

  void sifirla() {gecmis_.clear();}

private:
  struct Goruntu
  {
    Poz2 poz;
    double zaman = 0.0;
    bool gecerli = false;
    std::vector<float> menzil;  // hücre başına en yakın menzil, 0 = dönüş yok
  };
  std::deque<Goruntu> gecmis_;
};

}  // namespace nesne
