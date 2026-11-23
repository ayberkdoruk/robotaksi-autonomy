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
}  // namespace

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

std::vector<Takipci::Iz> Takipci::guncelle(
  const std::vector<Kutu> & kutular, const std::vector<std::string> & etiketler,
  double zaman_s, const TakipAyari & a)
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
    for (size_t j = 0; j < kutular.size(); ++j) {
      const double dx = kutular[j].cx - izler_[i].x(0);
      const double dy = kutular[j].cy - izler_[i].x(1);
      const double m = std::sqrt(dx * dx + dy * dy);
      if (m <= a.kapi) {
        ciftler.emplace_back(m, i, j);
      }
    }
  }
  std::sort(ciftler.begin(), ciftler.end());
  std::vector<bool> iz_eslesti(izler_.size(), false);
  std::vector<bool> kutu_eslesti(kutular.size(), false);

  for (const auto & [m, i, j] : ciftler) {
    if (iz_eslesti[i] || kutu_eslesti[j]) {
      continue;
    }
    iz_eslesti[i] = true;
    kutu_eslesti[j] = true;
    IcIz & iz = izler_[i];
    duzelt(iz, kutular[j].cx, kutular[j].cy);
    iz.kutu = kutular[j];
    iz.etiketler.push_back(etiketler[j]);
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
  for (size_t j = 0; j < kutular.size(); ++j) {
    if (kutu_eslesti[j]) {
      continue;
    }
    IcIz iz;
    iz.x << kutular[j].cx, kutular[j].cy, 0.0, 0.0;
    iz.P = Eigen::Matrix4d::Zero();
    iz.P(0, 0) = iz.P(1, 1) = OLCUM_GURULTU * OLCUM_GURULTU;
    iz.P(2, 2) = iz.P(3, 3) = ILK_HIZ_BELIRSIZLIGI * ILK_HIZ_BELIRSIZLIGI;
    iz.kutu = kutular[j];
    iz.etiketler.push_back(etiketler[j]);
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
      sonuc.push_back({iz.id, iz.kutu, cogunluk(iz.etiketler),
          static_cast<float>(iz.x(2)), static_cast<float>(iz.x(3)), iz.yas});
    }
  }
  return sonuc;
}

}  // namespace nesne
