#include "nesne/zemin.hpp"

#include <algorithm>
#include <cmath>

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

// Yeni bulunan düzlem, öncekinden bu kadar (metre) fazla yukarı/aşağı kaymışsa
// şüpheli sayılır: zemin bir turda (0.1 sn) 20 cm zıplamaz, büyük ihtimalle
// RANSAC bir masa/kaldırım yüzeyine kapılmıştır.
constexpr float MAX_ZIPLAMA = 0.20f;

// Ama ilk düzlem yanlışsa sonsuza kadar ona takılı kalmayalım:
// bu kadar tur üst üste reddedilen yeni düzlem sonunda kabul edilir.
constexpr int MAX_UST_USTE_RED = 10;
}  // namespace

ZeminAyirici::Sonuc ZeminAyirici::ayir(const Bulut & girdi, const ZeminAyari & a)
{
  Sonuc s;
  s.nesne.reset(new Bulut);
  s.zemin.reset(new Bulut);

  // --- 1) RANSAC adayları: sadece beklenen zemin yüksekliğine yakın noktalar ---
  // Masa düzeninde bulutun ~%40'ı masa yüzeyine düşüyor (z ≈ -0.12). Tüm buluta
  // RANSAC uygulasak "en büyük düz yüzey" olarak masayı seçerdi. Bant, bunu önler.
  Bulut::Ptr adaylar(new Bulut);
  adaylar->reserve(girdi.size() / 4);
  for (const auto & p : girdi) {
    if (std::fabs(p.z - a.zemin_z) <= a.zemin_bant) {
      adaylar->push_back(p);
    }
  }

  // --- 2) RANSAC ile düzlem ara ---
  if (static_cast<int>(adaylar->size()) >= std::max(3, a.zemin_min_nokta)) {
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

    pcl::PointIndices icerdekiler;
    pcl::ModelCoefficients katsayi;
    seg.segment(icerdekiler, katsayi);

    if (static_cast<int>(icerdekiler.indices.size()) >= a.zemin_min_nokta &&
      katsayi.values.size() == 4)
    {
      // Normali birim uzunluğa getir ve yukarı baktır (c > 0)
      float na = katsayi.values[0], nb = katsayi.values[1];
      float nc = katsayi.values[2], nd = katsayi.values[3];
      const float boy = std::sqrt(na * na + nb * nb + nc * nc);
      if (boy > 1e-6f) {
        const float isaret = (nc < 0.0f) ? -1.0f : 1.0f;
        Duzlem yeni{isaret * na / boy, isaret * nb / boy, isaret * nc / boy,
          isaret * nd / boy, true};

        // setOptimizeCoefficients düzlemi biraz döndürebilir; eğimi tekrar kontrol et
        const bool egim_uygun = yeni.c >= std::cos(a.zemin_max_egim_derece * DERECE_RADYAN);
        const bool ziplama_yok = !duzlem_.gecerli ||
          std::fabs(yeni.merkezZ() - duzlem_.merkezZ()) <= MAX_ZIPLAMA;

        if (egim_uygun && (ziplama_yok || ust_uste_red_ >= MAX_UST_USTE_RED)) {
          duzlem_ = yeni;
          ust_uste_red_ = 0;
          s.yeni_duzlem_bulundu = true;
        } else if (egim_uygun) {
          ++ust_uste_red_;
        }
      }
    }
  }

  // --- 3) Hiç düzlem yoksa: zemin_z yüksekliğinde düz bir düzlem varsay ---
  // (Önceki geçerli düzlem varsa o kullanılmaya devam eder.)
  if (!duzlem_.gecerli) {
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
