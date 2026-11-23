#include "nesne/kumeleme.hpp"

#include <cmath>

#include <pcl/kdtree/kdtree_flann.h>

namespace nesne
{

std::vector<std::vector<int>> kumele(const Bulut::ConstPtr & bulut, const KumeAyari & a)
{
  std::vector<std::vector<int>> kumeler;
  const int n = static_cast<int>(bulut->size());
  if (n == 0) {
    return kumeler;
  }

  // KD ağacı: "şu noktanın R yarıçapındaki komşuları kim?" sorusunu hızlı cevaplar
  pcl::KdTreeFLANN<Nokta> agac;
  agac.setInputCloud(bulut);

  std::vector<bool> islendi(n, false);
  std::vector<int> komsular;
  std::vector<float> komsu_mesafe_kare;
  std::vector<int> kume;
  const float yatay_kare = a.yatay_tol * a.yatay_tol;

  for (int baslangic = 0; baslangic < n; ++baslangic) {
    if (islendi[baslangic]) {
      continue;
    }
    // Yeni küme: başlangıç noktasından büyüt
    kume.clear();
    kume.push_back(baslangic);
    islendi[baslangic] = true;

    // "kume" listesi aynı zamanda kuyruk: sırayla her noktanın komşularına bak
    for (size_t i = 0; i < kume.size(); ++i) {
      const Nokta & p = (*bulut)[kume[i]];
      const float r = std::sqrt(p.x * p.x + p.y * p.y);
      const float dikey_tol = a.dikey_taban + a.dikey_artis * r;

      // Arama yarıçapı, yatay ve dikey eşiğin oluşturduğu kutuyu içine alan küre.
      // Yarıçap noktanın mesafesine göre değişir: yakında küçük (hızlı), uzakta büyük.
      const float yaricap = std::sqrt(yatay_kare + dikey_tol * dikey_tol);
      agac.radiusSearch(p, yaricap, komsular, komsu_mesafe_kare);

      for (int k : komsular) {
        if (islendi[k]) {
          continue;
        }
        const Nokta & q = (*bulut)[k];
        const float dx = q.x - p.x;
        const float dy = q.y - p.y;
        if (dx * dx + dy * dy <= yatay_kare && std::fabs(q.z - p.z) <= dikey_tol) {
          islendi[k] = true;
          kume.push_back(k);
        }
      }
    }

    const int boy = static_cast<int>(kume.size());
    if (boy >= a.min_nokta && boy <= a.max_nokta) {
      kumeler.push_back(kume);
    }
  }
  return kumeler;
}

}  // namespace nesne
