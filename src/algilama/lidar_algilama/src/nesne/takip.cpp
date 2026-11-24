#include "nesne/takip.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <tuple>

namespace nesne
{

namespace
{
// Kalman ayarları
constexpr double IVME_GURULTU = 2.0;   // m/s²: nesnenin hızı turdan tura ne kadar değişebilir
constexpr double OLCUM_GURULTU = 0.15; // m: küme merkezinin ölçüm titremesi
constexpr double ILK_HIZ_BELIRSIZLIGI = 2.0;  // m/s: yeni izin hızını bilmiyoruz

// Kayıt "--loop" ile başa sardığında zaman geriye atlar; ya da veri bir süre kesilir.
// O durumda eski izlerle eşleştirmek anlamsız: hepsini sıfırla.
constexpr double MAX_DT = 1.0;

// Kutu yumuşatma
constexpr float PI_F = 3.14159265358979323846f;
// Yön bir turda en fazla bu kadar döner. Kareye yakın nesnelerde tek turluk 30-60°
// zıplamalar böylece birkaç dereceye iner; gerçekten dönen bir araç (ör. 30°/s = 3°/tur)
// rahatça izlenir.
constexpr float MAX_YON_ADIMI = 15.0f * PI_F / 180.0f;

// Dinamik kararı: son KANIT_PENCERE eşleşmenin en az KANIT_GEREKEN'inde kanıt olmalı
constexpr size_t KANIT_PENCERE = 3;
constexpr int KANIT_GEREKEN = 2;

// a - b farkı, kutu 180° simetrik olduğu için (-90°, +90°] aralığında
float yonFarki(float a, float b)
{
  float d = a - b;
  while (d > PI_F / 2.0f) {d -= PI_F;}
  while (d <= -PI_F / 2.0f) {d += PI_F;}
  return d;
}

float yonuDuzelt(float a)
{
  return yonFarki(a, 0.0f);
}
}  // namespace

void Takipci::kutuyuYumusat(IcIz & iz, const Kutu & olcum, float yumusatma)
{
  const float alfa = 1.0f - yumusatma;
  Kutu & d = iz.duz;
  // Aynı dikdörtgen iki şekilde yazılabilir: (uzunluk, genislik, yon) ya da
  // (genislik, uzunluk, yon + 90°). Önceki yöne yakın olanı seç; yoksa uzunluk ile
  // genişlik birbirine yakınken kutu her turda 90° dönmüş gibi görünürdü.
  float u = olcum.uzunluk, g = olcum.genislik, yon = olcum.yon;
  if (std::fabs(yonFarki(yon + PI_F / 2.0f, d.yon)) < std::fabs(yonFarki(yon, d.yon))) {
    std::swap(u, g);
    yon += PI_F / 2.0f;
  }
  const float fark = std::clamp(yonFarki(yon, d.yon), -MAX_YON_ADIMI, MAX_YON_ADIMI);
  d.yon = yonuDuzelt(d.yon + alfa * fark);
  d.uzunluk += alfa * (u - d.uzunluk);
  d.genislik += alfa * (g - d.genislik);
  d.yukseklik += alfa * (olcum.yukseklik - d.yukseklik);
  d.cz += alfa * (olcum.cz - d.cz);
}

Kutu Takipci::ciktiKutusu(const IcIz & iz, float yumusatma)
{
  if (yumusatma <= 0.0f) {
    return iz.kutu;  // yumuşatma kapalı: ham kutu
  }
  // Mesafe, en yakın nokta, tepe yüksekliği, nokta sayısı: son ham ölçüm
  Kutu k = iz.kutu;
  k.cx = static_cast<float>(iz.x(0));  // merkez: Kalman
  k.cy = static_cast<float>(iz.x(1));
  k.cz = iz.duz.cz;
  k.yukseklik = iz.duz.yukseklik;
  k.alt_z = k.cz - k.yukseklik / 2.0f;
  k.ust_z = k.cz + k.yukseklik / 2.0f;
  // Dışarıya her zaman uzunluk >= genislik (Kutu'nun tanımı)
  if (iz.duz.uzunluk >= iz.duz.genislik) {
    k.uzunluk = iz.duz.uzunluk;
    k.genislik = iz.duz.genislik;
    k.yon = iz.duz.yon;
  } else {
    k.uzunluk = iz.duz.genislik;
    k.genislik = iz.duz.uzunluk;
    k.yon = yonuDuzelt(iz.duz.yon + PI_F / 2.0f);
  }
  return k;
}

void Takipci::tahminEt(IcIz & iz, double dt) const
{
  // Sabit hız modeli: yeni konum = eski konum + hız * dt
  Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
  F(0, 2) = dt;
  F(1, 3) = dt;
  // Bilinmeyen ivmenin yarattığı belirsizlik
  Eigen::Matrix<double, 4, 2> G;
  G << dt * dt / 2.0, 0.0,
    0.0, dt * dt / 2.0,
    dt, 0.0,
    0.0, dt;
  const Eigen::Matrix4d Q = G * G.transpose() * (IVME_GURULTU * IVME_GURULTU);
  iz.x = F * iz.x;
  iz.P = F * iz.P * F.transpose() + Q;
}

void Takipci::duzelt(IcIz & iz, double olcum_x, double olcum_y) const
{
  // Sadece konumu ölçüyoruz (hızı değil)
  Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
  H(0, 0) = 1.0;
  H(1, 1) = 1.0;
  const Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * (OLCUM_GURULTU * OLCUM_GURULTU);
  const Eigen::Vector2d z(olcum_x, olcum_y);
  const Eigen::Vector2d y = z - H * iz.x;              // tahmin ile ölçüm farkı
  const Eigen::Matrix2d S = H * iz.P * H.transpose() + R;
  const Eigen::Matrix<double, 4, 2> K = iz.P * H.transpose() * S.inverse();  // ne kadar güvenelim
  iz.x += K * y;
  iz.P = (Eigen::Matrix4d::Identity() - K * H) * iz.P;
}

std::string Takipci::cogunluk(const std::deque<std::string> & etiketler)
{
  std::map<std::string, int> sayac;
  for (const auto & e : etiketler) {
    ++sayac[e];
  }
  std::string en_cok;
  int en_cok_sayi = -1;
  for (const auto & [etiket, sayi] : sayac) {
    if (sayi > en_cok_sayi) {
      en_cok = etiket;
      en_cok_sayi = sayi;
    }
  }
  return en_cok;
}

void Takipci::kanitIsle(IcIz & iz, bool kanit, double zaman_s, const TakipAyari & a)
{
  iz.kanitlar.push_back(kanit);
  while (iz.kanitlar.size() > KANIT_PENCERE) {
    iz.kanitlar.pop_front();
  }
  const int sayi = static_cast<int>(std::count(iz.kanitlar.begin(), iz.kanitlar.end(), true));
  if (sayi >= KANIT_GEREKEN) {
    iz.dinamik = true;
  }
  if (kanit) {
    iz.son_kanit = zaman_s;
  }
  if (iz.dinamik && zaman_s - iz.son_kanit > a.dinamik_tutma) {
    iz.dinamik = false;  // uzun süredir hareket kanıtı yok: durdu
  }
}

std::vector<Takipci::Iz> Takipci::guncelle(
  const std::vector<Olcum> & olcumler, double zaman_s, const TakipAyari & a)
{
  // --- 0) Zaman kontrolü ---
  double dt = 0.0;
  if (son_zaman_ >= 0.0) {
    dt = zaman_s - son_zaman_;
    if (dt <= 0.0 || dt > MAX_DT) {
      izler_.clear();
      dt = 0.0;
    }
  }
  son_zaman_ = zaman_s;

  // --- 1) Her izin bu turdaki konumunu tahmin et ---
  if (dt > 0.0) {
    for (auto & iz : izler_) {
      tahminEt(iz, dt);
    }
  }

  // --- 2) Eşleştirme: en yakın çiftten başlayarak (açgözlü) ---
  std::vector<std::tuple<double, size_t, size_t>> ciftler;  // (mesafe, iz, kutu)
  for (size_t i = 0; i < izler_.size(); ++i) {
    for (size_t j = 0; j < olcumler.size(); ++j) {
      const double dx = olcumler[j].kutu.cx - izler_[i].x(0);
      const double dy = olcumler[j].kutu.cy - izler_[i].x(1);
      const double m = std::sqrt(dx * dx + dy * dy);
      if (m <= a.kapi) {
        ciftler.emplace_back(m, i, j);
      }
    }
  }
  std::sort(ciftler.begin(), ciftler.end());
  std::vector<bool> iz_eslesti(izler_.size(), false);
  std::vector<bool> kutu_eslesti(olcumler.size(), false);

  for (const auto & [m, i, j] : ciftler) {
    if (iz_eslesti[i] || kutu_eslesti[j]) {
      continue;
    }
    iz_eslesti[i] = true;
    kutu_eslesti[j] = true;
    IcIz & iz = izler_[i];
    const Olcum & o = olcumler[j];
    duzelt(iz, o.kutu.cx, o.kutu.cy);
    iz.kutu = o.kutu;
    kutuyuYumusat(iz, o.kutu, a.kutu_yumusatma);
    kanitIsle(iz, o.dinamik_kanit, zaman_s, a);
    iz.yol_uzerinde = o.yol_uzerinde;
    iz.etiketler.push_back(o.etiket);
    while (static_cast<int>(iz.etiketler.size()) > a.etiket_gecmisi) {
      iz.etiketler.pop_front();
    }
    ++iz.ust_uste_gorulme;
    ++iz.yas;
    iz.kayip_tur = 0;
    if (iz.id == 0 && iz.ust_uste_gorulme >= a.dogrulama) {
      iz.id = sonraki_id_++;
    }
  }

  // --- 3) Eşleşmeyen izler: aday ise hemen sil, numaralı ise sabret ---
  for (size_t i = 0; i < izler_.size(); ++i) {
    if (!iz_eslesti[i]) {
      izler_[i].ust_uste_gorulme = 0;
      ++izler_[i].kayip_tur;
    }
  }
  izler_.erase(std::remove_if(izler_.begin(), izler_.end(), [&a](const IcIz & iz) {
      return iz.kayip_tur > 0 && (iz.id == 0 || iz.kayip_tur > a.kayip);
    }), izler_.end());

  // --- 4) Eşleşmeyen kutular: yeni aday iz ---
  for (size_t j = 0; j < olcumler.size(); ++j) {
    if (kutu_eslesti[j]) {
      continue;
    }
    const Olcum & o = olcumler[j];
    IcIz iz;
    iz.x << o.kutu.cx, o.kutu.cy, 0.0, 0.0;
    iz.P = Eigen::Matrix4d::Zero();
    iz.P(0, 0) = iz.P(1, 1) = OLCUM_GURULTU * OLCUM_GURULTU;
    iz.P(2, 2) = iz.P(3, 3) = ILK_HIZ_BELIRSIZLIGI * ILK_HIZ_BELIRSIZLIGI;
    iz.kutu = o.kutu;
    iz.duz = o.kutu;
    iz.etiketler.push_back(o.etiket);
    kanitIsle(iz, o.dinamik_kanit, zaman_s, a);
    iz.yol_uzerinde = o.yol_uzerinde;
    iz.ust_uste_gorulme = 1;
    iz.yas = 1;
    if (a.dogrulama <= 1) {
      iz.id = sonraki_id_++;
    }
    izler_.push_back(iz);
  }

  // --- 5) Sadece numaralı VE bu turda görülen izleri ver ---
  // (Kaybolmuş ama henüz silinmemiş izler yayınlanmaz: ekranda hayalet kutu olmasın.)
  std::vector<Iz> sonuc;
  for (const auto & iz : izler_) {
    if (iz.id != 0 && iz.kayip_tur == 0) {
      sonuc.push_back({iz.id, ciktiKutusu(iz, a.kutu_yumusatma), cogunluk(iz.etiketler),
          static_cast<float>(iz.x(2)), static_cast<float>(iz.x(3)), iz.yas, iz.dinamik,
          iz.yol_uzerinde});
    }
  }
  return sonuc;
}

}  // namespace nesne
