// =============================================================================
// zemin.hpp — Zemin düzlemini her turda ölçüp noktaları ayırır
//
// Neden sabit bir "z_min" yetmiyor? Araçta yol eğimli olabilir, LiDAR hafif
// yamuk takılabilir. Sabit yükseklik sınırı o zaman ya zemini engel sanar ya da
// alçak engelleri kaçırır. Burada zemin düzlemini her turda RANSAC ile buluyoruz.
//
// RANSAC kısaca: rastgele 3 nokta seç, bir düzlem geçir, bu düzleme yakın kaç
// nokta var say. Bunu çok kez tekrarla, en çok noktayı toplayan düzlemi seç.
// =============================================================================
#pragma once

#include "nesne/tipler.hpp"

namespace nesne
{

struct ZeminAyari
{
  float zemin_z = -0.81f;              // beklenen zemin yüksekliği (LiDAR'a göre, metre)
  float zemin_bant = 0.30f;            // RANSAC'a sadece zemin_z ± bant içindeki noktalar girer
  float zemin_esik = 0.10f;            // düzleme bundan yakın nokta "zemin" sayılır
  float zemin_max_egim_derece = 10.0f; // düzlem en fazla bu kadar eğik olabilir
  float max_yukseklik = 2.5f;          // zeminden bu kadar yukarısı (tavan, dal) atılır
  int zemin_min_nokta = 30;            // düzlemin kabulü için gereken en az nokta
};

class ZeminAyirici
{
public:
  struct Sonuc
  {
    Bulut::Ptr nesne;      // zeminin üstünde, max_yukseklik altında kalan noktalar
    Bulut::Ptr zemin;      // zemin (ve zeminin altında kalan yansıma) noktaları
    bool yeni_duzlem_bulundu = false;  // false: önceki / varsayılan düzlem kullanıldı
  };

  Sonuc ayir(const Bulut & girdi, const ZeminAyari & a);

  const Duzlem & duzlem() const {return duzlem_;}

  // Önceki düzlemi unut (ör. zemin_z parametresi değişince)
  void sifirla()
  {
    duzlem_ = Duzlem{};
    ust_uste_red_ = 0;
  }

private:
  Duzlem duzlem_;        // son kabul edilen düzlem
  int ust_uste_red_ = 0; // üst üste kaç kez yeni düzlem "önceki düzlemden çok farklı" diye reddedildi
};

}  // namespace nesne
