// lidar_surus_node: düz git, önüne engel ya da yaya gelirse dur.
//
// Girdi : /engel/durum  (robotaksi_msgs/EngelDurumu, engel_node)  -> yelpazede DUR / DİKKAT
//         /nesne/liste  (robotaksi_msgs/NesneDizisi, nesne_node)  -> yol koridorundaki nesneler
// Çıktı : /arac/komut   (robotaksi_msgs/AracKomutu, 20 Hz)        -> arac_haberlesme/serial_comm_node
//         /lidar_surus/durum (std_msgs/String)                    -> şu anki karar + sebep
//
// Karar sırası (ilk tutan kazanır), her 50 ms'de bir:
//   1) surus_aktif = false            -> BEKLIYOR    (fren)
//   2) azami_surus_suresi doldu       -> SURE_DOLDU  (fren, yeniden aktif edilene kadar)
//   3) algılama verisi bayat / yok    -> VERI_YOK    (fren)  LiDAR ya da düğüm düşerse araç durur
//   4) engel DUR ya da koridorda yakın / çarpışmak üzere nesne -> DUR (fren)
//   5) son DUR / VERI_YOK'tan beri kalkis_bekleme geçmedi -> KALKIS_BEKLE (fren)
//   6) engel DİKKAT ya da koridorda nesne -> DIKKAT (dikkatte_gaz_kes ise gaz kesilir, fren yok)
//   7) yol boş                        -> GIDIYOR     (gaz)
// Direksiyon: yon_tutma = "kapali" ise hep duz_direksiyon. "lidar" ya da "imu" ise sürüş
// başlatıldığı andaki yön hedef alınır; araç saparsa direksiyon ters yöne düzeltilir:
//   direksiyon = duz_direksiyon - yon_kazanc * (hedef_yon - yon)   [derece, sola dönüş +]
//   (25 = tam sol, 165 = tam sağ: sola düzeltmek için değer küçülür)
// Yön verisi veri_zaman_asimi'ndan uzun süre gelmezse VERI_YOK (fren).
//
// Araç önce surus_aktif=false (fren) ile açılır. Başlatmak:
//   ros2 param set /lidar_surus surus_aktif true
// Durdurmak:
//   ros2 param set /lidar_surus surus_aktif false

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <robotaksi_msgs/msg/arac_komutu.hpp>
#include <robotaksi_msgs/msg/engel_durumu.hpp>
#include <robotaksi_msgs/msg/nesne_dizisi.hpp>

#include <chrono>
#include <cstdio>
#include <algorithm>
#include <cmath>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

using robotaksi_msgs::msg::AracKomutu;
using robotaksi_msgs::msg::EngelDurumu;
using robotaksi_msgs::msg::NesneDizisi;

class LidarSurusNode : public rclcpp::Node
{
public:
  LidarSurusNode()
  : Node("lidar_surus"), saat_(RCL_STEADY_TIME)
  {
    declare_parameter<bool>("surus_aktif", false);
    declare_parameter<int64_t>("duz_direksiyon", 84);
    declare_parameter<bool>("engel_kullan", true);
    declare_parameter<bool>("nesne_kullan", true);
    declare_parameter<double>("veri_zaman_asimi", 0.5);
    declare_parameter<double>("nesne_dur_mesafesi", 3.0);
    declare_parameter<double>("nesne_dikkat_mesafesi", 6.0);
    declare_parameter<double>("carpisma_suresi_dur", 2.5);
    declare_parameter<bool>("dikkatte_gaz_kes", false);
    declare_parameter<double>("kalkis_bekleme", 2.0);
    declare_parameter<double>("azami_surus_suresi", 0.0);
    yon_kaynagi_ = declare_parameter<std::string>("yon_tutma", "kapali");
    declare_parameter<double>("yon_kazanc", 1.0);
    declare_parameter<int64_t>("yon_siniri", 15);
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "/zed/zed_node/imu/data");

    komut_pub_ = create_publisher<AracKomutu>("/arac/komut", 10);
    durum_pub_ = create_publisher<std_msgs::msg::String>("/lidar_surus/durum", 10);

    engel_sub_ = create_subscription<EngelDurumu>(
      "/engel/durum", 10, [this](const EngelDurumu::SharedPtr m) {
        engel_ = *m;
        engel_zamani_ = saat_.now();
        engel_alindi_ = true;
      });
    nesne_sub_ = create_subscription<NesneDizisi>(
      "/nesne/liste", 10, [this](const NesneDizisi::SharedPtr m) {
        nesneler_ = *m;
        nesne_zamani_ = saat_.now();
        nesne_alindi_ = true;
      });

    // Yön kaynakları (yon_tutma). İkisine de abone olunur; hangisinin kullanılacağı
    // yon_tutma parametresinden okunur, çalışırken değiştirilebilir.
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/nesne/odometri", 10, [this](const nav_msgs::msg::Odometry::SharedPtr m) {
        yonGeldi("lidar", m->pose.pose.orientation.x, m->pose.pose.orientation.y,
          m->pose.pose.orientation.z, m->pose.pose.orientation.w);
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Imu::SharedPtr m) {
        yonGeldi("imu", m->orientation.x, m->orientation.y, m->orientation.z, m->orientation.w);
      });

    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { tur(); });

    RCLCPP_INFO(get_logger(),
      "lidar_surus hazir (surus_aktif=false, FREN). Baslatmak icin: "
      "ros2 param set /lidar_surus surus_aktif true");
  }

private:
  enum class Durum { BEKLIYOR, SURE_DOLDU, VERI_YOK, DUR, KALKIS_BEKLE, DIKKAT, GIDIYOR };

  static const char * ad(Durum d)
  {
    switch (d) {
      case Durum::BEKLIYOR: return "BEKLIYOR";
      case Durum::SURE_DOLDU: return "SURE_DOLDU";
      case Durum::VERI_YOK: return "VERI_YOK";
      case Durum::DUR: return "DUR";
      case Durum::KALKIS_BEKLE: return "KALKIS_BEKLE";
      case Durum::DIKKAT: return "DIKKAT";
      default: return "GIDIYOR";
    }
  }

  // Dörtlükten (quaternion) z ekseni etrafındaki açıyı (yaw, derece) çıkarır
  void yonGeldi(const char * kaynak, double x, double y, double z, double w)
  {
    if (get_parameter("yon_tutma").as_string() != kaynak) {
      return;
    }
    yon_ = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)) * 180.0 / M_PI;
    yon_zamani_ = saat_.now();
    yon_alindi_ = true;
  }

  // Yol koridorundaki nesnelere bakar: 2 = dur, 1 = dikkat, 0 = boş
  int nesneKarari(std::string & sebep) const
  {
    const double dur_m = get_parameter("nesne_dur_mesafesi").as_double();
    const double dikkat_m = get_parameter("nesne_dikkat_mesafesi").as_double();
    const double ttc_dur = get_parameter("carpisma_suresi_dur").as_double();

    int karar = 0;
    char yazi[160];
    for (const auto & n : nesneler_.nesneler) {
      if (!n.yol_uzerinde) {
        continue;
      }
      const bool ttc_tehlike = n.carpisma_suresi > 0.0 && n.carpisma_suresi < ttc_dur;
      if (n.mesafe < dur_m || ttc_tehlike) {
        std::snprintf(yazi, sizeof(yazi), "nesne #%u %s %.1f m%s", n.id, n.etiket.c_str(),
          n.mesafe, ttc_tehlike ? " (carpisma yakin)" : "");
        sebep = yazi;
        return 2;
      }
      if (n.mesafe < dikkat_m && karar < 1) {
        std::snprintf(yazi, sizeof(yazi), "nesne #%u %s %.1f m", n.id, n.etiket.c_str(),
          n.mesafe);
        sebep = yazi;
        karar = 1;
      }
    }
    return karar;
  }

  void tur()
  {
    const rclcpp::Time simdi = saat_.now();
    const bool aktif = get_parameter("surus_aktif").as_bool();
    const bool engel_kullan = get_parameter("engel_kullan").as_bool();
    const bool nesne_kullan = get_parameter("nesne_kullan").as_bool();
    const double zaman_asimi = get_parameter("veri_zaman_asimi").as_double();
    const double azami_sure = get_parameter("azami_surus_suresi").as_double();
    const std::string yon_tutma = get_parameter("yon_tutma").as_string();
    const bool yon_acik = yon_tutma == "lidar" || yon_tutma == "imu";
    if (yon_tutma != yon_kaynagi_) {   // kaynak değişti: eski kaynağın açısı geçersiz
      yon_kaynagi_ = yon_tutma;
      yon_alindi_ = false;
      hedef_var_ = false;
    }
    const bool yon_taze = yon_alindi_ && (simdi - yon_zamani_).seconds() <= zaman_asimi;

    // Aktif edildiği an: süreyi başlat, kalkmadan önce kalkis_bekleme kadar bekle
    if (aktif && !onceki_aktif_) {
      baslangic_ = simdi;
      hedef_var_ = false;   // hedef yön: aktif edildikten sonraki ilk yön verisi
      son_dur_ = simdi;
      sure_doldu_ = false;
    }
    onceki_aktif_ = aktif;

    Durum d = Durum::GIDIYOR;
    std::string sebep;

    char yazi[160];
    if (!aktif) {
      d = Durum::BEKLIYOR;
    } else if (sure_doldu_ ||
      (azami_sure > 0.0 && (simdi - baslangic_).seconds() > azami_sure))
    {
      sure_doldu_ = true;
      d = Durum::SURE_DOLDU;
      std::snprintf(yazi, sizeof(yazi), "azami_surus_suresi %.0f sn doldu", azami_sure);
      sebep = yazi;
    } else if (yon_acik && !yon_taze) {
      d = Durum::VERI_YOK;
      sebep = yon_tutma == "imu" ?
        "yon (IMU) gelmiyor: " + get_parameter("imu_topic").as_string() :
        std::string("yon (/nesne/odometri) gelmiyor");
    } else if (engel_kullan &&
      (!engel_alindi_ || (simdi - engel_zamani_).seconds() > zaman_asimi))
    {
      d = Durum::VERI_YOK;
      sebep = "/engel/durum gelmiyor (engel_node / LiDAR?)";
    } else if (nesne_kullan &&
      (!nesne_alindi_ || (simdi - nesne_zamani_).seconds() > zaman_asimi))
    {
      d = Durum::VERI_YOK;
      sebep = "/nesne/liste gelmiyor (nesne_node?)";
    } else {
      int nesne = 0;
      std::string nesne_sebep;
      if (nesne_kullan) {
        nesne = nesneKarari(nesne_sebep);
      }
      const bool engel_dur = engel_kullan && engel_.durum == EngelDurumu::DUR;
      const bool engel_dikkat = engel_kullan && engel_.durum == EngelDurumu::DIKKAT;

      if (engel_dur || nesne == 2) {
        d = Durum::DUR;
        if (engel_dur) {
          std::snprintf(yazi, sizeof(yazi), "engel %.1f m", engel_.mesafe);
          sebep = yazi;
        } else {
          sebep = nesne_sebep;
        }
        son_dur_ = simdi;
      } else if ((simdi - son_dur_).seconds() < get_parameter("kalkis_bekleme").as_double()) {
        d = Durum::KALKIS_BEKLE;
        std::snprintf(yazi, sizeof(yazi), "%.1f sn",
          get_parameter("kalkis_bekleme").as_double() - (simdi - son_dur_).seconds());
        sebep = yazi;
      } else if (engel_dikkat || nesne == 1) {
        d = Durum::DIKKAT;
        if (engel_dikkat) {
          std::snprintf(yazi, sizeof(yazi), "engel %.1f m", engel_.mesafe);
          sebep = yazi;
        } else {
          sebep = nesne_sebep;
        }
      }
    }

    // Veri kesilmesi de DUR sayılır: veri geri gelince kalkis_bekleme kadar beklemeden kalkma
    if (d == Durum::VERI_YOK) {
      son_dur_ = simdi;
    }

    AracKomutu k;
    k.header.stamp = now();
    k.kaynak = "lidar_surus";
    // Direksiyon: düz + yön düzeltmesi
    const int duz = static_cast<int>(get_parameter("duz_direksiyon").as_int());
    int direksiyon = duz;
    std::string yon_yazi;
    if (yon_acik && yon_taze && aktif) {
      if (!hedef_var_) {
        hedef_yon_ = yon_;
        hedef_var_ = true;
      }
      double hata = hedef_yon_ - yon_;            // + = araç sağa sapmış, sola dönmeli
      hata = std::remainder(hata, 360.0);          // -180..180
      const int sinir = static_cast<int>(get_parameter("yon_siniri").as_int());
      const int duzeltme = std::clamp(
        static_cast<int>(std::lround(get_parameter("yon_kazanc").as_double() * hata)),
        -sinir, sinir);
      direksiyon = duz - duzeltme;
      std::snprintf(yazi, sizeof(yazi), " (yon hatasi %+.1f derece, direksiyon %d)", hata,
        direksiyon);
      yon_yazi = yazi;
    }
    k.direksiyon = static_cast<int16_t>(
      std::clamp(direksiyon, static_cast<int>(AracKomutu::DIREKSIYON_SOL),
      static_cast<int>(AracKomutu::DIREKSIYON_SAG)));
    k.geri = false;
    switch (d) {
      case Durum::GIDIYOR:
        k.gaz = true;
        k.fren = false;
        break;
      case Durum::DIKKAT:
        k.gaz = !get_parameter("dikkatte_gaz_kes").as_bool();
        k.fren = false;
        break;
      default:
        k.gaz = false;
        k.fren = true;
    }
    komut_pub_->publish(k);

    std_msgs::msg::String s;
    s.data = (sebep.empty() ? ad(d) : std::string(ad(d)) + ": " + sebep) + yon_yazi;
    durum_pub_->publish(s);

    // Durum değişince bir satır (sebebin içindeki mesafe her turda değişir, onu saymıyoruz)
    if (d != onceki_durum_) {
      RCLCPP_WARN(get_logger(), "%s  [gaz=%d fren=%d]", s.data.c_str(), k.gaz, k.fren);
      onceki_durum_ = d;
    }
  }

  rclcpp::Clock saat_;   // tazelik ve süreler: kayıt oynatırken de duvar saati
  rclcpp::Publisher<AracKomutu>::SharedPtr komut_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr durum_pub_;
  rclcpp::Subscription<EngelDurumu>::SharedPtr engel_sub_;
  rclcpp::Subscription<NesneDizisi>::SharedPtr nesne_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  EngelDurumu engel_;
  NesneDizisi nesneler_;
  rclcpp::Time engel_zamani_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time nesne_zamani_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time baslangic_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time son_dur_{0, 0, RCL_STEADY_TIME};
  bool engel_alindi_ = false;
  bool nesne_alindi_ = false;
  bool onceki_aktif_ = false;
  bool sure_doldu_ = false;
  std::string yon_kaynagi_;
  double yon_ = 0.0;          // son yön (derece)
  double hedef_yon_ = 0.0;    // sürüş başlatıldığındaki yön
  bool yon_alindi_ = false;
  bool hedef_var_ = false;
  rclcpp::Time yon_zamani_{0, 0, RCL_STEADY_TIME};
  Durum onceki_durum_ = Durum::GIDIYOR;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarSurusNode>());
  rclcpp::shutdown();
  return 0;
}
