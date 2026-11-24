#include "nesne/dinamik.hpp"

#include <algorithm>
#include <cmath>

namespace nesne
{

namespace
{
constexpr double PI = 3.14159265358979323846;
constexpr double DERECE = 180.0 / PI;

// Menzil görüntüsünün hücreleri
constexpr int AZ_SAYI = 720;                       // yatay: 360° / 0.5°
constexpr double AZ_ADIM = 2.0 * PI / AZ_SAYI;
constexpr double EL_MIN = -60.0;                   // dikey: -60° .. +40°, 1°'lik
constexpr double EL_ADIM = 1.0;
constexpr int EL_SAYI = 100;

constexpr double MIN_MENZIL = 0.5;    // m: LiDAR'a bundan yakın noktalara bakılmaz
constexpr double PAY_ARTIS = 0.02;    // m/m: uzakta odometrinin küçük açı hatası büyür
constexpr double KENAR = 0.10;        // m: noktanın etrafında bu genişlikteki hücrelere bakılır
constexpr double MIN_YAS = 0.15;      // s: bundan yeni taramalar karşılaştırılmaz (yaya az yer değiştirmiş olur)
constexpr double GECMIS_EK = 0.3;     // s: geçmiş bakis_suresi'nden bu kadar fazla tutulur

int elHucresi(double el_derece)
{
  return static_cast<int>(std::floor((el_derece - EL_MIN) / EL_ADIM));
}

int azHucresi(double az)
{
  int i = static_cast<int>(std::floor((az + PI) / AZ_ADIM));
  return std::clamp(i, 0, AZ_SAYI - 1);
}
}  // namespace

void DinamikAyirici::ekle(
  const Bulut & tum, const Poz2 & poz, double zaman_s, bool gecerli, const DinamikAyari & a)
{
  // Zaman geri gittiyse (kayıt başa sardı) geçmiş anlamsız
  if (!gecmis_.empty() && zaman_s <= gecmis_.back().zaman) {
    gecmis_.clear();
  }
  Goruntu g;
  g.poz = poz;
  g.zaman = zaman_s;
  g.gecerli = gecerli;
  g.menzil.assign(static_cast<size_t>(AZ_SAYI) * EL_SAYI, 0.0f);
  for (const auto & p : tum) {
    const double yatay = std::hypot(p.x, p.y);
    const double r = std::hypot(yatay, static_cast<double>(p.z));
    if (r < MIN_MENZIL) {
      continue;
    }
    const int e = elHucresi(std::atan2(p.z, yatay) * DERECE);
    if (e < 0 || e >= EL_SAYI) {
      continue;
    }
    float & hucre = g.menzil[static_cast<size_t>(e) * AZ_SAYI + azHucresi(std::atan2(p.y, p.x))];
    if (hucre <= 0.0f || r < hucre) {
      hucre = static_cast<float>(r);
    }
  }
  gecmis_.push_back(std::move(g));
  while (!gecmis_.empty() && zaman_s - gecmis_.front().zaman > a.bakis_suresi + GECMIS_EK) {
    gecmis_.pop_front();
  }
}

int DinamikAyirici::isaretle(
  const Bulut & noktalar, const Poz2 & poz, double zaman_s, const DinamikAyari & a,
  std::vector<uint8_t> & dinamik) const
{
  dinamik.assign(noktalar.size(), 0);

  // Karşılaştırılacak geçmiş taramalar: bakis_suresi'nin 1/3'ü, 2/3'ü ve kendisi kadar
  // önceki taramalara en yakın olanlar. Yakın geçmiş yeni başlayan hareketi, uzak geçmiş
  // yavaş hareketi ve nesnenin tamamını (kendi eski yerinden çıkmış olsun) yakalar.
  std::vector<const Goruntu *> secilen;
  for (int k = 1; k <= 3; ++k) {
    const double hedef = a.bakis_suresi * k / 3.0;
    const Goruntu * en_iyi = nullptr;
    double en_iyi_fark = 1e9;
    for (const auto & g : gecmis_) {
      const double yas = zaman_s - g.zaman;
      if (!g.gecerli || yas < MIN_YAS) {
        continue;
      }
      const double fark = std::fabs(yas - hedef);
      if (fark < en_iyi_fark) {
        en_iyi_fark = fark;
        en_iyi = &g;
      }
    }
    if (en_iyi && std::find(secilen.begin(), secilen.end(), en_iyi) == secilen.end()) {
      secilen.push_back(en_iyi);
    }
  }
  if (secilen.empty()) {
    return 0;
  }

  for (size_t i = 0; i < noktalar.size(); ++i) {
    const Nokta & p = noktalar[i];
    double wx, wy;
    poz.dunyaya(p.x, p.y, wx, wy);
    for (const Goruntu * g : secilen) {
      // Nokta, geçmiş taramanın LiDAR'ından nasıl görünürdü?
      double px, py;
      g->poz.lidara(wx, wy, px, py);
      const double yatay = std::hypot(px, py);
      const double r = std::hypot(yatay, static_cast<double>(p.z));
      if (r < MIN_MENZIL) {
        continue;
      }
      const int e0 = elHucresi(std::atan2(p.z, yatay) * DERECE);
      const int a0 = azHucresi(std::atan2(py, px));
      // Bakılacak komşu hücre sayısı: noktanın etrafında ~KENAR genişliği (yakında çok
      // hücre, uzakta az). Dikeyde en az ±2: Helios'un katmanları eşit aralıklı değil.
      const double yari_aci = std::atan2(KENAR, r) * DERECE;
      const int na = std::clamp(static_cast<int>(std::ceil(yari_aci / 0.5)), 1, 8);
      const int ne = std::clamp(static_cast<int>(std::ceil(yari_aci / EL_ADIM)), 2, 5);
      const double esik = r + a.pay + PAY_ARTIS * r;

      bool donus_var = false;  // en az bir hücrede ölçüm var mı
      bool bos = true;         // ölçüm olan bütün hücreler noktanın arkasını mı gördü
      for (int de = -ne; de <= ne && bos; ++de) {
        const int e = e0 + de;
        if (e < 0 || e >= EL_SAYI) {
          continue;
        }
        const float * satir = &g->menzil[static_cast<size_t>(e) * AZ_SAYI];
        for (int da = -na; da <= na; ++da) {
          const float v = satir[(a0 + da + AZ_SAYI) % AZ_SAYI];
          if (v <= 0.0f) {
            continue;
          }
          donus_var = true;
          if (v <= esik) {
            bos = false;  // bu yöndeki ışın noktaya (ya da önüne) çarpmış: orası boş değildi
            break;
          }
        }
      }
      if (donus_var && bos) {
        dinamik[i] = 1;
        break;
      }
    }
  }
  return static_cast<int>(secilen.size());
}

}  // namespace nesne
