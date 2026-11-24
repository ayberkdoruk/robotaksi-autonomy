#include "nesne/odometri.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>

namespace nesne
{

namespace
{
constexpr double PI = 3.14159265358979323846;

constexpr float Z_OLCEK = 0.5f;            // komşu aramasında dikey farkın ağırlığı
constexpr double ESLESME_MESAFESI = 0.6;   // m: yatayda bundan uzak eşleşme yok sayılır
constexpr double SAGLAM_SIGMA = 0.10;      // m: Cauchy ağırlığı (bundan büyük farklar zayıflar)
constexpr int MAX_TUR = 15;                // ICP en fazla bu kadar tur
constexpr size_t MAX_KAYNAK = 3000;        // hız için yeni taramadan en fazla bu kadar nokta
constexpr int MIN_ESLESME = 60;            // bundan az eşleşme: sonuç güvenilmez
constexpr double MAX_HATA = 0.25;          // m: eşleşmelerin ortalama farkı bundan büyükse güvenilmez
constexpr double MAX_HIZ = 6.0;            // m/s: araç bundan hızlı gidemez (yanlış eşleşme)
constexpr double MAX_DONUS = 1.5;          // rad/s
constexpr double SONUMLEME = 1.0;          // belirsiz yönde (uzun koridor) tahminden kaçmasın
constexpr double NORMAL_YARICAP = 0.6;     // m: anahtar noktanın yüzey yönü bu çevreden bulunur
constexpr double CIZGI_ORANI = 0.1;        // küçük/büyük yayılım oranı bundan küçükse düz yüzey
constexpr double ANAHTAR_MESAFE = 0.6;     // m: anahtar taramadan bu kadar uzaklaşınca yenisi
constexpr double ANAHTAR_ACI = 6.0 * PI / 180.0;
constexpr double ANAHTAR_SURE = 3.0;       // s: durunca da bu sürede bir yenilenir (eski yayalar silinsin)
constexpr int HATA_SINIRI = 5;             // üst üste bu kadar başarısız tur: anahtarı yeniden al
constexpr double MAX_DT = 1.0;             // s: daha uzun boşlukta geçmiş kopar
constexpr double HIZ_YUMUSATMA = 0.5;      // hız tahmininin yeni ölçüme yaklaşma oranı

double aciDuzelt(double a)
{
  while (a > PI) {a -= 2.0 * PI;}
  while (a <= -PI) {a += 2.0 * PI;}
  return a;
}
}  // namespace

void LidarOdometri::sifirla()
{
  anahtar_.reset();
  nx_.clear();
  ny_.clear();
  cizgi_.clear();
  anahtar_zaman_ = -1.0;
  poz_ = Poz2{};
  son_zaman_ = -1.0;
  vx_ = vy_ = wz_ = 0.0;
  ust_uste_hata_ = 0;
}

void LidarOdometri::anahtarYap(const Bulut & bulut)
{
  anahtar_.reset(new Bulut);
  anahtar_->reserve(bulut.size());
  const double c = std::cos(poz_.yaw), s = std::sin(poz_.yaw);
  for (const auto & p : bulut) {
    anahtar_->push_back(Nokta(
        static_cast<float>(poz_.x + c * p.x - s * p.y),
        static_cast<float>(poz_.y + s * p.x + c * p.y),
        p.z * Z_OLCEK));
  }
  anahtar_poz_ = poz_;
  const size_t n = anahtar_->size();
  nx_.assign(n, 0.0f);
  ny_.assign(n, 0.0f);
  cizgi_.assign(n, 0);
  if (n == 0) {
    return;
  }
  agac_.setInputCloud(anahtar_);

  // Her anahtar noktanın çevresindeki noktaların yatay yayılımı: bir yönde uzun, diğerinde
  // ince ise nokta düz bir yüzeyde (duvar) demektir; normali ince yöndür.
  std::vector<int> komsu;
  std::vector<float> uzaklik;
  for (size_t i = 0; i < n; ++i) {
    if (agac_.radiusSearch((*anahtar_)[i], NORMAL_YARICAP, komsu, uzaklik) < 4) {
      continue;
    }
    double mx = 0.0, my = 0.0;
    for (int k : komsu) {
      mx += (*anahtar_)[k].x;
      my += (*anahtar_)[k].y;
    }
    mx /= komsu.size();
    my /= komsu.size();
    double cxx = 0.0, cxy = 0.0, cyy = 0.0;
    for (int k : komsu) {
      const double dx = (*anahtar_)[k].x - mx, dy = (*anahtar_)[k].y - my;
      cxx += dx * dx;
      cxy += dx * dy;
      cyy += dy * dy;
    }
    // 2x2 simetrik matrisin özdeğerleri
    const double yari_iz = (cxx + cyy) / 2.0;
    const double fark = std::sqrt((cxx - cyy) * (cxx - cyy) / 4.0 + cxy * cxy);
    const double kucuk = yari_iz - fark, buyuk = yari_iz + fark;
    if (buyuk <= 1e-9 || kucuk > CIZGI_ORANI * buyuk) {
      continue;  // düz değil (köşe, bitki, tek direk): noktadan noktaya
    }
    // Küçük özdeğerin özvektörü = yüzeyin normali
    double vx, vy;
    if (std::fabs(cxy) > 1e-12) {
      vx = kucuk - cyy;
      vy = cxy;
    } else if (cxx <= cyy) {
      vx = 1.0;
      vy = 0.0;
    } else {
      vx = 0.0;
      vy = 1.0;
    }
    const double boy = std::hypot(vx, vy);
    nx_[i] = static_cast<float>(vx / boy);
    ny_[i] = static_cast<float>(vy / boy);
    cizgi_[i] = 1;
  }
}

OdometriSonucu LidarOdometri::guncelle(const Bulut & bulut, double zaman_s)
{
  OdometriSonucu sonuc;

  // --- 0) Zaman: ilk tur ya da kopukluk (kayıt başa sardı, veri kesildi) ---
  const double dt = son_zaman_ >= 0.0 ? zaman_s - son_zaman_ : -1.0;
  if (dt <= 0.0 || dt > MAX_DT || !anahtar_ || anahtar_->empty()) {
    // Konum dünyada olduğu yerde kalır; sadece geçmiş (hız, anahtar) unutulur
    vx_ = vy_ = wz_ = 0.0;
    son_zaman_ = zaman_s;
    anahtarYap(bulut);
    anahtar_zaman_ = zaman_s;
    ust_uste_hata_ = 0;
    sonuc.poz = poz_;
    sonuc.gecerli = !bulut.empty();
    sonuc.sifirlandi = true;
    return sonuc;
  }
  son_zaman_ = zaman_s;

  // --- 1) Sabit hız tahmini ---
  Poz2 tahmin;
  tahmin.x = poz_.x + vx_ * dt;
  tahmin.y = poz_.y + vy_ * dt;
  tahmin.yaw = aciDuzelt(poz_.yaw + wz_ * dt);

  // --- 2) ICP ---
  Poz2 p = tahmin;
  const size_t adim = std::max<size_t>(1, (bulut.size() + MAX_KAYNAK - 1) / MAX_KAYNAK);
  std::vector<int> idx(1);
  std::vector<float> d2(1);
  int eslesme = 0;
  double hata_kare = 0.0;
  for (int tur = 0; tur < MAX_TUR; ++tur) {
    const double c = std::cos(p.yaw), s = std::sin(p.yaw);
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    Eigen::Vector3d g = Eigen::Vector3d::Zero();
    eslesme = 0;
    hata_kare = 0.0;
    for (size_t i = 0; i < bulut.size(); i += adim) {
      const Nokta & q = bulut[i];
      // Noktanın şu anki tahminle dünya konumu ve LiDAR'a göre kolu (dönüş için)
      const double ax = c * q.x - s * q.y, ay = s * q.x + c * q.y;
      const double wx = p.x + ax, wy = p.y + ay;
      const Nokta ara(static_cast<float>(wx), static_cast<float>(wy), q.z * Z_OLCEK);
      if (agac_.nearestKSearch(ara, 1, idx, d2) < 1) {
        continue;
      }
      const Nokta & b = (*anahtar_)[idx[0]];
      const double dx = wx - b.x, dy = wy - b.y;
      if (dx * dx + dy * dy > ESLESME_MESAFESI * ESLESME_MESAFESI) {
        continue;
      }
      ++eslesme;
      // Küçük kaydırma (tx, ty) ve LiDAR etrafında küçük dönüş (dθ) sonrası nokta:
      //   w' ≈ w + (tx, ty) + dθ * (-ay, ax)
      if (cizgi_[idx[0]]) {
        // noktadan çizgiye: farkın yüzey normali üzerindeki kısmı
        const double nx = nx_[idx[0]], ny = ny_[idx[0]];
        const double r = nx * dx + ny * dy;
        const Eigen::Vector3d J(nx, ny, -nx * ay + ny * ax);
        const double w = 1.0 / (1.0 + (r / SAGLAM_SIGMA) * (r / SAGLAM_SIGMA));
        H += w * J * J.transpose();
        g += w * J * r;
        hata_kare += r * r;
      } else {
        // noktadan noktaya: x ve y farkı ayrı ayrı
        const double w = 1.0 / (1.0 + (dx * dx + dy * dy) / (SAGLAM_SIGMA * SAGLAM_SIGMA));
        const Eigen::Vector3d Jx(1.0, 0.0, -ay), Jy(0.0, 1.0, ax);
        H += w * (Jx * Jx.transpose() + Jy * Jy.transpose());
        g += w * (Jx * dx + Jy * dy);
        hata_kare += dx * dx + dy * dy;
      }
    }
    if (eslesme < MIN_ESLESME) {
      break;
    }
    H += SONUMLEME * Eigen::Matrix3d::Identity();
    const Eigen::Vector3d d = -H.ldlt().solve(g);
    p.x += d(0);
    p.y += d(1);
    p.yaw = aciDuzelt(p.yaw + d(2));
    if (std::fabs(d(0)) < 1e-3 && std::fabs(d(1)) < 1e-3 && std::fabs(d(2)) < 1e-4) {
      break;
    }
  }

  // --- 3) Sonuç güvenilir mi? ---
  const double hata = eslesme > 0 ? std::sqrt(hata_kare / eslesme) : 1e9;
  const double hiz = std::hypot(p.x - poz_.x, p.y - poz_.y) / dt;
  const double donus = std::fabs(aciDuzelt(p.yaw - poz_.yaw)) / dt;
  const bool gecerli = eslesme >= MIN_ESLESME && hata <= MAX_HATA &&
    hiz <= MAX_HIZ && donus <= MAX_DONUS;

  if (gecerli) {
    const double yeni_vx = (p.x - poz_.x) / dt;
    const double yeni_vy = (p.y - poz_.y) / dt;
    const double yeni_wz = aciDuzelt(p.yaw - poz_.yaw) / dt;
    vx_ += HIZ_YUMUSATMA * (yeni_vx - vx_);
    vy_ += HIZ_YUMUSATMA * (yeni_vy - vy_);
    wz_ += HIZ_YUMUSATMA * (yeni_wz - wz_);
    poz_ = p;
    ust_uste_hata_ = 0;
  } else {
    poz_ = tahmin;  // bu tur için sabit hız varsayımı
    ++ust_uste_hata_;
  }

  // --- 4) Anahtar taramayı yenile ---
  const bool uzaklasti =
    std::hypot(poz_.x - anahtar_poz_.x, poz_.y - anahtar_poz_.y) > ANAHTAR_MESAFE ||
    std::fabs(aciDuzelt(poz_.yaw - anahtar_poz_.yaw)) > ANAHTAR_ACI;
  if ((gecerli && (uzaklasti || zaman_s - anahtar_zaman_ > ANAHTAR_SURE)) ||
    ust_uste_hata_ >= HATA_SINIRI)
  {
    anahtarYap(bulut);
    anahtar_zaman_ = zaman_s;
    if (ust_uste_hata_ >= HATA_SINIRI) {
      vx_ = vy_ = wz_ = 0.0;  // hız tahmini de güvenilmez
      ust_uste_hata_ = 0;
    }
  }

  // --- 5) Dışarıya: hız LiDAR eksenlerinde ---
  const double c = std::cos(poz_.yaw), s = std::sin(poz_.yaw);
  sonuc.poz = poz_;
  sonuc.vx = static_cast<float>(c * vx_ + s * vy_);
  sonuc.vy = static_cast<float>(-s * vx_ + c * vy_);
  sonuc.wz = static_cast<float>(wz_);
  sonuc.gecerli = gecerli;
  sonuc.eslesme = eslesme;
  sonuc.hata = static_cast<float>(eslesme > 0 ? hata : 0.0);
  return sonuc;
}

}  // namespace nesne
