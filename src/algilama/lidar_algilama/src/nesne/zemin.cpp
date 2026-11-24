#include "nesne/zemin.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace nesne
{

namespace
{
constexpr float DERECE_RADYAN = 3.14159265358979323846f / 180.0f;

// Takipte yeni bulunan düzlem öncekinden bu kadar (metre) fazla yukarı/aşağı
// kaymışsa şüpheli sayılır: zemin bir turda (0.1 sn) 20 cm zıplamaz, büyük
// ihtimalle RANSAC bir masa/kaldırım yüzeyine kapılmıştır.
constexpr float MAX_ZIPLAMA = 0.20f;

// Ama takip yanlış düzleme oturduysa sonsuza kadar ona takılı kalmayalım:
// bu kadar tur üst üste reddedilen yeni düzlem sonunda kabul edilir.
constexpr int MAX_UST_USTE_RED = 10;

// Takipte bu kadar tur (10 Hz'de 2 sn) üst üste düzlem bulunamazsa ilk arama
// yeniden yapılır. O sürede son ölçülen düzlem kullanılmaya devam eder.
constexpr int YENIDEN_ARA_TUR = 20;

// --- Otomatik ilk arama ---
// Zemin LiDAR'dan en az bu kadar aşağıdadır (zemin_z'nin üst sınırıyla aynı).
// Bundan yukarıdaki yüzeyler (LiDAR'ın üstünde durduğu masa, tavan) hiç aday olmaz.
constexpr float EN_YUKSEK_ZEMIN = -0.30f;
// En fazla bu kadar yatay yüzey sırayla çıkarılır (zemin + birkaç masa/tezgâh yeter)
constexpr int MAX_YATAY_YUZEY = 5;
// Bir yüzeyin zemin adayı olması için en geniş yüzeyin en az bu oranında nokta
// toplaması gerekir. Böylece zeminin altına düşen birkaç yansıma noktası ya da
// küçük bir çukur "en alçak yüzey" seçilmez.
constexpr float MIN_YUZEY_ORANI = 0.25f;

// RANSAC ile adaylar içinde (z eksenine dik) yatay bir düzlem arar.
//   icerdekiler: bulunan düzlemin noktaları (düzlem eğim yüzünden reddedilse de doldurulur)
//   dönüş      : en az zemin_min_nokta noktalı, eğimi uygun bir düzlem bulundu mu
bool yatayDuzlemAra(
  const Bulut::Ptr & adaylar, const ZeminAyari & a, Duzlem & duzlem, std::vector<int> & icerdekiler)
{
  icerdekiler.clear();
  if (static_cast<int>(adaylar->size()) < std::max(3, a.zemin_min_nokta)) {
    return false;
  }
  pcl::SACSegmentation<Nokta> seg;
  seg.setOptimizeCoefficients(true);
  // PERPENDICULAR_PLANE: verilen eksene (z) DİK düzlem ara, yani yatay düzlem.
  // EpsAngle kadar eğikliğe izin verilir. Duvarlar böylece hiç aday olmaz.
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
  seg.setEpsAngle(a.zemin_max_egim_derece * DERECE_RADYAN);
  seg.setDistanceThreshold(a.zemin_esik);
  seg.setMaxIterations(100);
  seg.setInputCloud(adaylar);

  pcl::PointIndices bulunan;
  pcl::ModelCoefficients katsayi;
  seg.segment(bulunan, katsayi);
  icerdekiler = bulunan.indices;

  if (static_cast<int>(icerdekiler.size()) < a.zemin_min_nokta || katsayi.values.size() != 4) {
    return false;
  }
  // Normali birim uzunluğa getir ve yukarı baktır (c > 0)
  const float na = katsayi.values[0], nb = katsayi.values[1];
  const float nc = katsayi.values[2], nd = katsayi.values[3];
  const float boy = std::sqrt(na * na + nb * nb + nc * nc);
  if (boy < 1e-6f) {
    return false;
  }
  const float isaret = (nc < 0.0f) ? -1.0f : 1.0f;
  duzlem = Duzlem{isaret * na / boy, isaret * nb / boy, isaret * nc / boy, isaret * nd / boy, true};
  // setOptimizeCoefficients düzlemi biraz döndürebilir; eğimi tekrar kontrol et
  return duzlem.c >= std::cos(a.zemin_max_egim_derece * DERECE_RADYAN);
}
}  // namespace

bool ZeminAyirici::enAlcakYatayYuzey(
  const Bulut & girdi, const ZeminAyari & a, Duzlem & sonuc, int & yuzey_sayisi) const
{
  Bulut::Ptr kalan(new Bulut);
  kalan->reserve(girdi.size());
  for (const auto & p : girdi) {
    if (p.z <= EN_YUKSEK_ZEMIN) {
      kalan->push_back(p);
    }
  }

  // En büyük yatay yüzeyi bul, noktalarını çıkar, kalanlarda tekrar ara...
  struct Yuzey
  {
    Duzlem duzlem;
    int nokta;
  };
  std::vector<Yuzey> yuzeyler;
  std::vector<int> icerdekiler;
  for (int i = 0; i < MAX_YATAY_YUZEY; ++i) {
    Duzlem d;
    const bool uygun = yatayDuzlemAra(kalan, a, d, icerdekiler);
    if (static_cast<int>(icerdekiler.size()) < a.zemin_min_nokta) {
      break;  // kalan noktalarda anlamlı bir yüzey yok
    }
    if (uygun) {
      yuzeyler.push_back({d, static_cast<int>(icerdekiler.size())});
    }
    // Bu yüzeyin noktalarını çıkar (eğik diye reddedilse bile: aynı yüzey tekrar bulunmasın)
    std::vector<bool> cikar(kalan->size(), false);
    for (int j : icerdekiler) {
      cikar[j] = true;
    }
    Bulut::Ptr yeni(new Bulut);
    yeni->reserve(kalan->size() - icerdekiler.size());
    for (size_t j = 0; j < kalan->size(); ++j) {
      if (!cikar[j]) {
        yeni->push_back((*kalan)[j]);
      }
    }
    kalan = yeni;
  }

  yuzey_sayisi = static_cast<int>(yuzeyler.size());
  if (yuzeyler.empty()) {
    return false;
  }
  int en_cok = 0;
  for (const auto & y : yuzeyler) {
    en_cok = std::max(en_cok, y.nokta);
  }
  // Yeterince geniş yüzeylerin en alçağı
  const Yuzey * secilen = nullptr;
  for (const auto & y : yuzeyler) {
    if (y.nokta >= MIN_YUZEY_ORANI * en_cok &&
      (!secilen || y.duzlem.merkezZ() < secilen->duzlem.merkezZ()))
    {
      secilen = &y;
    }
  }
  sonuc = secilen->duzlem;
  return true;
}

ZeminAyirici::Sonuc ZeminAyirici::ayir(const Bulut & girdi, const ZeminAyari & a)
{
  Sonuc s;
  s.nesne.reset(new Bulut);
  s.zemin.reset(new Bulut);

  // --- 1) Düzlem ara: takip ya da ilk arama ---
  const bool takip = olculmus_ && bulunamayan_tur_ < YENIDEN_ARA_TUR;
  Duzlem yeni;
  bool bulundu = false;
  if (takip) {
    // Bir önceki düzlemin yakınındaki noktalar
    Bulut::Ptr adaylar(new Bulut);
    adaylar->reserve(girdi.size() / 4);
    for (const auto & p : girdi) {
      if (std::fabs(duzlem_.yukseklik(p)) <= a.zemin_bant) {
        adaylar->push_back(p);
      }
    }
    std::vector<int> icerdekiler;
    bulundu = yatayDuzlemAra(adaylar, a, yeni, icerdekiler);
  } else if (a.otomatik) {
    s.ilk_arama = true;
    bulundu = enAlcakYatayYuzey(girdi, a, yeni, s.yatay_yuzey_sayisi);
  } else {
    // Otomatik kapalı: sadece beklenen yüksekliğin yakını. Masa düzeninde bulutun
    // ~%40'ı masa yüzeyine düşüyor (z ≈ -0.12); tüm buluta RANSAC uygulasak "en büyük
    // düz yüzey" olarak masayı seçerdi. Bant bunu önler.
    s.ilk_arama = true;
    Bulut::Ptr adaylar(new Bulut);
    adaylar->reserve(girdi.size() / 4);
    for (const auto & p : girdi) {
      if (std::fabs(p.z - a.zemin_z) <= a.zemin_bant) {
        adaylar->push_back(p);
      }
    }
    std::vector<int> icerdekiler;
    bulundu = yatayDuzlemAra(adaylar, a, yeni, icerdekiler);
  }

  // --- 2) Kabul et ya da reddet ---
  if (bulundu) {
    bulunamayan_tur_ = 0;
    const bool ziplama_yok = std::fabs(yeni.merkezZ() - duzlem_.merkezZ()) <= MAX_ZIPLAMA;
    // İlk aramanın sonucu doğrudan kabul edilir; takipte ani sıçrama şüphelidir
    if (!takip || ziplama_yok || ust_uste_red_ >= MAX_UST_USTE_RED) {
      duzlem_ = yeni;
      olculmus_ = true;
      ust_uste_red_ = 0;
      s.kaynak = ZeminKaynagi::OLCULDU;
    } else {
      ++ust_uste_red_;
      s.kaynak = ZeminKaynagi::ONCEKI;
    }
  } else {
    if (takip) {
      ++bulunamayan_tur_;
    }
    s.kaynak = olculmus_ ? ZeminKaynagi::ONCEKI : ZeminKaynagi::VARSAYILAN;
  }

  // --- 3) Hiç ölçülemediyse: zemin_z yüksekliğinde düz bir düzlem varsay ---
  if (!olculmus_) {
    duzlem_ = Duzlem{0.0f, 0.0f, 1.0f, -a.zemin_z, true};
  }

  // --- 4) Her noktayı zemin üstü yüksekliğine göre ayır ---
  s.nesne->reserve(girdi.size());
  s.zemin->reserve(girdi.size() / 4);
  for (const auto & p : girdi) {
    const float h = duzlem_.yukseklik(p);
    if (h < a.zemin_esik) {
      s.zemin->push_back(p);   // zemin veya zeminin altı (yansıma)
    } else if (h <= a.max_yukseklik) {
      s.nesne->push_back(p);
    }
    // h > max_yukseklik: tavan / ağaç dalı, atılır
  }
  return s;
}

}  // namespace nesne
