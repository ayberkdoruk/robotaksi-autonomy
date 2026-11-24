// =============================================================================
// zemin.hpp — Zemin düzlemini her turda ölçüp noktaları ayırır
//
// Neden sabit bir "z_min" yetmiyor? Araçta yol eğimli olabilir, LiDAR hafif
// yamuk takılabilir. Sabit yükseklik sınırı o zaman ya zemini engel sanar ya da
// alçak engelleri kaçırır. Burada zemin düzlemini her turda RANSAC ile buluyoruz.
//
// RANSAC kısaca: rastgele 3 nokta seç, bir düzlem geçir, bu düzleme yakın kaç
// nokta var say. Bunu çok kez tekrarla, en çok noktayı toplayan düzlemi seç.
//
// Zemin nerede aranır?
//   1) Açılışta (ya da zemin uzun süre kaybolunca) — İLK ARAMA:
//      - otomatik açıksa: ipucu kullanılmaz. LiDAR'ın altındaki bütün yatay
//        yüzeyler sırayla bulunur, yeterince geniş olanların EN ALÇAĞI zemin
//        seçilir. Masa, tezgâh, kaldırım her zaman zeminin üstündedir.
//      - otomatik kapalıysa: sadece zemin_z ± zemin_bant içinde aranır.
//   2) Sonraki turlarda — TAKİP: sadece bir önceki düzlemin ± zemin_bant
//      yakınındaki noktalara bakılır (hızlı, ve masaya kapılmaz).
// Hiç düzlem bulunamadıysa zemin_z yüksekliğinde düz bir zemin varsayılır.
// =============================================================================
#pragma once

#include "nesne/tipler.hpp"

namespace nesne
{

struct ZeminAyari
{
  bool otomatik = true;                // ilk aramada zemin ipucusuz bulunsun (zemin_z'ye bakmadan)
  float zemin_z = -0.81f;              // beklenen zemin yüksekliği (LiDAR'a göre, metre)
  float zemin_bant = 0.30f;            // RANSAC'a sadece bu bandın içindeki noktalar girer
  float zemin_esik = 0.10f;            // düzleme bundan yakın nokta "zemin" sayılır
  float zemin_max_egim_derece = 10.0f; // düzlem en fazla bu kadar eğik olabilir
  float max_yukseklik = 2.5f;          // zeminden bu kadar yukarısı (tavan, dal) atılır
  int zemin_min_nokta = 30;            // düzlemin kabulü için gereken en az nokta
};

// Kullanılan düzlem nereden geldi (robotaksi_msgs/ZeminDurumu sabitleriyle aynı sıra)
enum class ZeminKaynagi
{
  OLCULDU = 0,     // bu turda ölçüldü
  ONCEKI = 1,      // bu turda bulunamadı, son ölçülen düzlem kullanılıyor
  VARSAYILAN = 2,  // hiç ölçülemedi, zemin_z yüksekliğinde düz zemin varsayılıyor
};

class ZeminAyirici
{
public:
  struct Sonuc
  {
    Bulut::Ptr nesne;      // zeminin üstünde, max_yukseklik altında kalan noktalar
    Bulut::Ptr zemin;      // zemin (ve zeminin altında kalan yansıma) noktaları
    ZeminKaynagi kaynak = ZeminKaynagi::VARSAYILAN;
    bool ilk_arama = false;     // bu tur ilk arama yapıldı (takip değil)
    int yatay_yuzey_sayisi = 0; // otomatik ilk aramada bulunan yatay yüzey sayısı
  };

  Sonuc ayir(const Bulut & girdi, const ZeminAyari & a);

  const Duzlem & duzlem() const {return duzlem_;}

  // Önceki düzlemi unut, ilk aramayı yeniden yap (ör. zemin_otomatik değişince)
  void sifirla()
  {
    duzlem_ = Duzlem{};
    olculmus_ = false;
    ust_uste_red_ = 0;
    bulunamayan_tur_ = 0;
  }

private:
  // Otomatik ilk arama: LiDAR'ın altındaki yatay yüzeylerin yeterince geniş olan en alçağı
  bool enAlcakYatayYuzey(const Bulut & girdi, const ZeminAyari & a, Duzlem & sonuc,
    int & yuzey_sayisi) const;

  Duzlem duzlem_;            // kullanılan düzlem
  bool olculmus_ = false;    // duzlem_ gerçekten ölçüldü mü (false: varsayılan düz zemin)
  int ust_uste_red_ = 0;     // takipte üst üste kaç kez "öncekinden çok farklı" diye reddedildi
  int bulunamayan_tur_ = 0;  // takipte üst üste kaç tur düzlem bulunamadı
};

}  // namespace nesne
