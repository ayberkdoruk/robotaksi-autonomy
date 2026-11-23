// =============================================================================
// takip.hpp — Nesneleri kareden kareye takip eder: numara ve hız
//
// Her turda kümeler yeniden bulunur ve bir önceki turla hiçbir bağları yoktur.
// Takipçi, "bu turdaki şu kutu, önceki turdaki #12 numaralı nesnedir" eşleştirmesini
// yapar. Böylece:
//   - nesne numarası sabit kalır (#12 hep #12)
//   - konum değişiminden hız hesaplanır
//   - etiket kareden kareye titremez (son turların çoğunluğu kullanılır)
//
// Hız için Kalman filtresi kullanılıyor: ölçülen konum biraz titrer (insan döndükçe
// görünen yüzü değişir), Kalman bu titremeyi "sabit hızla hareket" varsayımıyla
// yumuşatır.
//
// NOT: LiDAR'ın sabit durduğu varsayılır. Araçta odometri gelince takip, aracın
// hareketinden arındırılmış bir frame'de (odom) yapılmalı.
// =============================================================================
#pragma once

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "nesne/tipler.hpp"

namespace nesne
{

struct TakipAyari
{
  float kapi = 1.0f;       // tahmin edilen konum ile kutu arasındaki en fazla mesafe (metre)
  int dogrulama = 3;       // aday iz bu kadar tur üst üste görülünce numara alır
  int kayip = 5;           // numaralı iz bu kadar tur görülmezse silinir
  int etiket_gecmisi = 10; // etiket, son bu kadar turun çoğunluğuyla verilir
};

class Takipci
{
public:
  // Dışarıya verilen iz bilgisi (sadece numaralı ve bu turda görülen izler)
  struct Iz
  {
    uint32_t id;
    Kutu kutu;
    std::string etiket;  // son turların çoğunluğu
    float vx, vy;        // m/s
    uint32_t yas;        // kaç turdur takipte
  };

  // zaman_s: nokta bulutunun zaman damgası (saniye)
  std::vector<Iz> guncelle(
    const std::vector<Kutu> & kutular, const std::vector<std::string> & etiketler,
    double zaman_s, const TakipAyari & a);

  void sifirla() {izler_.clear(); son_zaman_ = -1.0;}

private:
  struct IcIz
  {
    uint32_t id = 0;            // 0 = henüz numara almadı (aday)
    Eigen::Vector4d x;          // durum: [x, y, vx, vy]
    Eigen::Matrix4d P;          // durumun belirsizliği
    Kutu kutu;
    std::deque<std::string> etiketler;
    int ust_uste_gorulme = 0;
    int kayip_tur = 0;
    uint32_t yas = 0;
  };

  void tahminEt(IcIz & iz, double dt) const;
  void duzelt(IcIz & iz, double olcum_x, double olcum_y) const;
  static std::string cogunluk(const std::deque<std::string> & etiketler);

  std::vector<IcIz> izler_;
  double son_zaman_ = -1.0;
  uint32_t sonraki_id_ = 1;
};

}  // namespace nesne
