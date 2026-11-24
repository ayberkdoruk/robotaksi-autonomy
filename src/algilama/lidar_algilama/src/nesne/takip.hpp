// =============================================================================
// takip.hpp — Nesneleri kareden kareye takip eder: numara ve hız
//
// Her turda kümeler yeniden bulunur ve bir önceki turla hiçbir bağları yoktur.
// Takipçi, "bu turdaki şu kutu, önceki turdaki #12 numaralı nesnedir" eşleştirmesini
// yapar. Böylece:
//   - nesne numarası sabit kalır (#12 hep #12)
//   - konum değişiminden hız hesaplanır
//   - etiket kareden kareye titremez (son turların çoğunluğu kullanılır)
//   - kutu kareden kareye titremez (merkez Kalman'dan, boyut ve yön yumuşatılarak)
//   - nesnenin hiç hareket edip etmediği bilinir ("hareketli" bayrağı, aşağıya bakın)
//
// Hız için Kalman filtresi kullanılıyor: ölçülen konum biraz titrer (insan döndükçe
// görünen yüzü değişir), Kalman bu titremeyi "sabit hızla hareket" varsayımıyla
// yumuşatır.
//
// Kutu neden her turda biraz farklı çıkar? Kutu, kümenin o turdaki noktalarından sıfırdan
// hesaplanır. LiDAR her turda nesnenin biraz farklı noktalarına çarpar, kümenin kenarı
// 5-10 cm oynar. Kareye yakın nesnelerde "en küçük alanlı dikdörtgen" iki farklı yönde
// neredeyse aynı alanı verir ve yön turdan tura zıplar. Bu yüzden dışarıya verilen kutu
// yumuşatılır. MESAFE ve EN YAKIN NOKTA ise yumuşatılmaz: güvenlik için gecikmemeli.
//
// "Hareketli" neden hıza bakarak değil de yer değiştirmeye bakarak karar verir? Duran
// bir nesnenin kümesi yanındakiyle birleşip ayrıldıkça merkezi sıçrar; Kalman hızı o
// sırada 1 sn'den uzun süre 0.3 m/s'yi aşabiliyor (atölye kaydında ölçüldü). Bu yüzden
// her izin bir "çapa" noktası var (ilk görüldüğü yer):
//   - iz çapadan hareket_mesafesi'nden fazla uzaklaşınca hareketli olur, çapa oraya taşınır
//   - hareketli iz DURMA_SURESI (5 sn) boyunca çapasından uzaklaşmazsa tekrar duran sayılır
// Birleşme sıçramaları genelde 0.6-0.8 m'de kalıyor; 0.8 m eşik ve 5 sn ile atölye kaydında
// duran nesnelerin karelerinin %0.6'sı hareketli göründü, yürüyen yayalar 0.5 m yürüdükten
// sonra en geç 0.5 sn içinde hareketli sayıldı.
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
  // Kutu yumuşatma: 0 = kapalı (ham kutu), 0.9 = çok yumuşak (yavaş tepki).
  // Her turda boyut ve yön, ölçüme doğru (1 - kutu_yumusatma) oranında yaklaşır.
  float kutu_yumusatma = 0.7f;
  // İz çapasından (ilk görüldüğü / son hareket ettiği yer) bundan fazla uzaklaşınca "hareketli" (metre)
  float hareket_mesafesi = 0.8f;
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
    bool hareketli;      // son DURMA_SURESI içinde hareket_mesafesi'nden fazla yer değiştirdi mi
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
    Kutu kutu;                  // son ham ölçüm (mesafe, en yakın nokta buradan)
    Kutu duz;                   // yumuşatılmış boyut / yön / yükseklik (uzunluk < genislik olabilir)
    std::deque<std::string> etiketler;
    int ust_uste_gorulme = 0;
    int kayip_tur = 0;
    uint32_t yas = 0;
    double capa_x = 0.0, capa_y = 0.0;  // ilk görüldüğü / son hareket ettiği yer
    bool hareketli = false;
    double son_hareket = 0.0;           // son kez çapadan uzaklaştığı an (saniye)
  };

  void tahminEt(IcIz & iz, double dt) const;
  void duzelt(IcIz & iz, double olcum_x, double olcum_y) const;
  static void kutuyuYumusat(IcIz & iz, const Kutu & olcum, float yumusatma);
  static Kutu ciktiKutusu(const IcIz & iz, float yumusatma);
  static std::string cogunluk(const std::deque<std::string> & etiketler);

  std::vector<IcIz> izler_;
  double son_zaman_ = -1.0;
  uint32_t sonraki_id_ = 1;
};

}  // namespace nesne
