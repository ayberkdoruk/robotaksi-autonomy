#include "nesne/kutu.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace nesne
{

namespace
{
constexpr float PI_F = 3.14159265358979323846f;
constexpr float MIN_BOYUT = 0.05f;  // tek noktalı / tek doğru üzerindeki kümelerde kutu sıfır olmasın

struct Nokta2
{
  float x, y;
};

float capraz(const Nokta2 & o, const Nokta2 & a, const Nokta2 & b)
{
  return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

// Dışbükey zarf ("monotone chain" yöntemi): noktaları saran en küçük dışbükey
// çokgen. Düşünün ki noktalara çivi çakılmış ve etrafına lastik geçirilmiş;
// lastiğin değdiği çiviler zarfın köşeleridir. En küçük dikdörtgenin bir kenarı
// her zaman bu zarfın bir kenarıyla çakışır; bu yüzden sadece zarfa bakmak yeter.
std::vector<Nokta2> disbukeyZarf(std::vector<Nokta2> p)
{
  std::sort(p.begin(), p.end(), [](const Nokta2 & a, const Nokta2 & b) {
      return a.x < b.x || (a.x == b.x && a.y < b.y);
    });
  p.erase(std::unique(p.begin(), p.end(), [](const Nokta2 & a, const Nokta2 & b) {
      return a.x == b.x && a.y == b.y;
    }), p.end());
  if (p.size() < 3) {
    return p;
  }
  std::vector<Nokta2> zarf(2 * p.size());
  size_t k = 0;
  for (size_t i = 0; i < p.size(); ++i) {  // alt zincir
    while (k >= 2 && capraz(zarf[k - 2], zarf[k - 1], p[i]) <= 0.0f) {--k;}
    zarf[k++] = p[i];
  }
  for (size_t i = p.size() - 1, t = k + 1; i > 0; --i) {  // üst zincir
    while (k >= t && capraz(zarf[k - 2], zarf[k - 1], p[i - 1]) <= 0.0f) {--k;}
    zarf[k++] = p[i - 1];
  }
  zarf.resize(k - 1);
  return zarf;
}
}  // namespace

Kutu kutuHesapla(const Bulut & bulut, const std::vector<int> & indeks, const Duzlem & zemin)
{
  Kutu k;
  k.nokta_sayisi = static_cast<int>(indeks.size());
  k.alt_z = std::numeric_limits<float>::max();
  k.ust_z = std::numeric_limits<float>::lowest();
  k.tepe_yuksekligi = std::numeric_limits<float>::lowest();
  float en_yakin_kare = std::numeric_limits<float>::max();

  std::vector<Nokta2> xy;
  xy.reserve(indeks.size());
  for (int i : indeks) {
    const Nokta & p = bulut[i];
    xy.push_back({p.x, p.y});
    k.alt_z = std::min(k.alt_z, p.z);
    k.ust_z = std::max(k.ust_z, p.z);
    k.tepe_yuksekligi = std::max(k.tepe_yuksekligi, zemin.yukseklik(p));
    const float r_kare = p.x * p.x + p.y * p.y;
    if (r_kare < en_yakin_kare) {
      en_yakin_kare = r_kare;
      k.yakin_x = p.x;
      k.yakin_y = p.y;
      k.yakin_z = p.z;
    }
  }
  k.mesafe = std::sqrt(en_yakin_kare);

  // --- En küçük alanlı dönük dikdörtgen ---
  // Zarfın her kenarı için: o kenarı x eksenine çevirecek kadar döndür, eksene
  // hizalı sınırları bul, alanı hesapla. En küçük alanlı olanı seç.
  const std::vector<Nokta2> zarf = disbukeyZarf(xy);
  float en_iyi_alan = std::numeric_limits<float>::max();
  float en_iyi_aci = 0.0f, en_iyi_u = 0.0f, en_iyi_v = 0.0f;
  float en_iyi_du = 0.0f, en_iyi_dv = 0.0f;

  const size_t kenar_sayisi = zarf.size() < 2 ? 1 : zarf.size();
  for (size_t e = 0; e < kenar_sayisi; ++e) {
    float aci = 0.0f;
    if (zarf.size() >= 2) {
      const Nokta2 & a = zarf[e];
      const Nokta2 & b = zarf[(e + 1) % zarf.size()];
      aci = std::atan2(b.y - a.y, b.x - a.x);
    }
    const float c = std::cos(aci), s = std::sin(aci);
    float u_min = std::numeric_limits<float>::max(), u_max = std::numeric_limits<float>::lowest();
    float v_min = u_min, v_max = u_max;
    for (const auto & p : zarf) {
      // p'yi -aci kadar döndür: (u, v) kenara göre koordinatlar
      const float u = c * p.x + s * p.y;
      const float v = -s * p.x + c * p.y;
      u_min = std::min(u_min, u); u_max = std::max(u_max, u);
      v_min = std::min(v_min, v); v_max = std::max(v_max, v);
    }
    const float alan = (u_max - u_min) * (v_max - v_min);
    if (alan < en_iyi_alan) {
      en_iyi_alan = alan;
      en_iyi_aci = aci;
      en_iyi_u = (u_min + u_max) / 2.0f;
      en_iyi_v = (v_min + v_max) / 2.0f;
      en_iyi_du = u_max - u_min;
      en_iyi_dv = v_max - v_min;
    }
  }

  // Merkezi geri döndür (u, v) -> (x, y)
  const float c = std::cos(en_iyi_aci), s = std::sin(en_iyi_aci);
  k.cx = c * en_iyi_u - s * en_iyi_v;
  k.cy = s * en_iyi_u + c * en_iyi_v;

  // Uzunluk her zaman uzun kenar olsun; yön de uzun kenarın yönü
  float yon = en_iyi_aci;
  if (en_iyi_dv > en_iyi_du) {
    std::swap(en_iyi_du, en_iyi_dv);
    yon += PI_F / 2.0f;
  }
  // Kutu 180° simetrik: yönü -90°..+90° aralığına getir
  while (yon > PI_F / 2.0f) {yon -= PI_F;}
  while (yon <= -PI_F / 2.0f) {yon += PI_F;}

  k.uzunluk = std::max(en_iyi_du, MIN_BOYUT);
  k.genislik = std::max(en_iyi_dv, MIN_BOYUT);
  k.yon = yon;
  k.yukseklik = std::max(k.ust_z - k.alt_z, MIN_BOYUT);
  k.cz = (k.alt_z + k.ust_z) / 2.0f;
  return k;
}

std::string etiketVer(const Kutu & k, const EtiketAyari & a)
{
  // Önce yapı: uzun ve ince (duvarın tek yüzü) ya da çok uzun
  if ((k.uzunluk >= a.yapi_min_uzunluk && k.genislik <= a.yapi_max_genislik) ||
    k.uzunluk >= a.yapi_kesin_uzunluk)
  {
    return ETIKET_YAPI;
  }
  // Yaya adayı: insan boyunda tepe noktası ve insan genişliğinde taban.
  // Kutunun yüksekliği yerine TEPE noktasının zeminden yüksekliği kullanılıyor:
  // bacaklar bir şeyin arkasında kalıp görünmese de doğru çalışır.
  if (k.tepe_yuksekligi >= a.yaya_min_tepe && k.tepe_yuksekligi <= a.yaya_max_tepe &&
    k.uzunluk <= a.yaya_max_uzunluk && k.genislik <= a.yaya_max_genislik)
  {
    return ETIKET_YAYA;
  }
  return ETIKET_ENGEL;
}

}  // namespace nesne
