// =============================================================================
// takip.hpp — Nesneleri kareden kareye takip eder: numara, hız, dinamik mi
//
// Her turda kümeler yeniden bulunur ve bir önceki turla hiçbir bağları yoktur.
// Takipçi, "bu turdaki şu kutu, önceki turdaki #12 numaralı nesnedir" eşleştirmesini
// yapar. Böylece:
//   - nesne numarası sabit kalır (#12 hep #12)
//   - konum değişiminden hız hesaplanır (Kalman filtresi, sabit hız varsayımı)
//   - etiket kareden kareye titremez (son turların çoğunluğu kullanılır)
//   - kutu kareden kareye titremez (merkez Kalman'dan, boyut ve yön yumuşatılarak)
//   - nesnenin dinamik (hareket eden) olup olmadığına birkaç turun kanıtıyla karar verilir
//
// TAKİP DÜNYA ÇERÇEVESİNDE YAPILIR: kutular takipçiye aracın hareketi (odometri)
// çıkarılmış olarak, yere göre konumlarıyla verilir. Araç 1 m/s giderken duran bir
// direk dünyada yerinde durur: hızı ~0 çıkar, kutusu kaymaz ve dönmez.
//
// Kutu neden her turda biraz farklı çıkar? Kutu, kümenin o turdaki noktalarından sıfırdan
// hesaplanır. LiDAR her turda nesnenin biraz farklı noktalarına çarpar, kümenin kenarı
// 5-10 cm oynar. Kareye yakın nesnelerde "en küçük alanlı dikdörtgen" iki farklı yönde
// neredeyse aynı alanı verir ve yön turdan tura zıplar. Bu yüzden dışarıya verilen kutu
// yumuşatılır. MESAFE ve EN YAKIN NOKTA ise yumuşatılmaz: güvenlik için gecikmemeli.
//
// "Dinamik" kararı küme merkezinin hareketine bakmaz (duran nesnelerin kümeleri birleşip
// ayrılınca merkez 0.6-0.8 m sıçrar, araç giderken görünen yüz değişince kayar). Kanıt
// görünürlük testinden gelir (dinamik.hpp): kümenin noktalarının yeterli kısmı, az önce
// boş görülmüş bir yerde mi? Son 3 eşleşmenin en az 2'sinde kanıt varsa iz dinamik olur;
// dinamik_tutma süresi boyunca hiç kanıt gelmezse (yaya durdu) tekrar duran sayılır.
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
  // Dinamik iz bu kadar süre hiç hareket kanıtı vermezse tekrar duran sayılır (saniye)
  float dinamik_tutma = 3.0f;
};

// Bir turdaki tek ölçüm (bir küme)
struct Olcum
{
  Kutu kutu;              // cx, cy, yon DÜNYA çerçevesinde; mesafe / en yakın nokta LiDAR'a göre
  std::string etiket;
  bool dinamik_kanit = false;  // bu turda kümenin yeterli kısmı boş görülmüş yerde
  bool yol_uzerinde = false;   // bu turda kümenin noktaları aracın yol koridorunda
};

class Takipci
{
public:
  // Dışarıya verilen iz bilgisi (sadece numaralı ve bu turda görülen izler)
  struct Iz
  {
    uint32_t id;
    Kutu kutu;           // cx, cy, yon DÜNYA çerçevesinde (yumuşatılmış)
    std::string etiket;  // son turların çoğunluğu
    float vx, vy;        // yere göre hız, dünya eksenlerinde (m/s)
    uint32_t yas;        // kaç turdur takipte
    bool dinamik;        // hareket ediyor (ya da dinamik_tutma süresi içinde etti)
    bool yol_uzerinde;   // bu turun ölçümü yol koridorunda
  };

  // zaman_s: nokta bulutunun zaman damgası (saniye)
  std::vector<Iz> guncelle(const std::vector<Olcum> & olcumler, double zaman_s,
    const TakipAyari & a);

  void sifirla() {izler_.clear(); son_zaman_ = -1.0;}

private:
  struct IcIz
  {
    uint32_t id = 0;            // 0 = henüz numara almadı (aday)
    Eigen::Vector4d x;          // durum: [x, y, vx, vy] (dünya)
    Eigen::Matrix4d P;          // durumun belirsizliği
    Kutu kutu;                  // son ham ölçüm (mesafe, en yakın nokta buradan)
    Kutu duz;                   // yumuşatılmış boyut / yön / yükseklik (uzunluk < genislik olabilir)
    std::deque<std::string> etiketler;
    std::deque<bool> kanitlar;  // son eşleşmelerin dinamik kanıtı
    bool yol_uzerinde = false;
    int ust_uste_gorulme = 0;
    int kayip_tur = 0;
    uint32_t yas = 0;
    bool dinamik = false;
    double son_kanit = 0.0;     // son dinamik kanıtın zamanı (saniye)
  };

  void tahminEt(IcIz & iz, double dt) const;
  void duzelt(IcIz & iz, double olcum_x, double olcum_y) const;
  static void kutuyuYumusat(IcIz & iz, const Kutu & olcum, float yumusatma);
  static Kutu ciktiKutusu(const IcIz & iz, float yumusatma);
  static std::string cogunluk(const std::deque<std::string> & etiketler);
  static void kanitIsle(IcIz & iz, bool kanit, double zaman_s, const TakipAyari & a);

  std::vector<IcIz> izler_;
  double son_zaman_ = -1.0;
  uint32_t sonraki_id_ = 1;
};

}  // namespace nesne
