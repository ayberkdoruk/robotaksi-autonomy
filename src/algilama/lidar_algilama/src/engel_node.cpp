// =============================================================================
// engel_node.cpp
//
// RoboSense Helios 32 LiDAR'ın nokta bulutunu (/rslidar_points) dinler,
// seçilen yöndeki (sektördeki) engellere bakarak DUR / DIKKAT / GUVENLI
// bölge durumunu belirler ve /engel/durum (robotaksi_msgs/EngelDurumu) olarak yayınlar.
// Bu bir sürüş komutu değildir; ne yapılacağına sürüş yöntemi karar verir.
//
// NOT: Bu node LiDAR'ın kendisine veya sürücüsüne HİÇBİR ayar göndermez.
//      Sadece sürücünün yayınladığı noktaları okur ve kendi içinde filtreler.
//
// Eksenler (LiDAR'a göre): x = ileri, y = sol, z = yukarı, birim metre.
// =============================================================================

#include <algorithm>   // std::max, std::nth_element
#include <cmath>       // std::sqrt, std::cos, std::sin, std::isfinite, std::ceil
#include <cstdio>      // std::snprintf
#include <limits>      // std::numeric_limits
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "robotaksi_msgs/msg/engel_durumu.hpp"

constexpr double PI = 3.14159265358979323846;
constexpr double DERECE_RADYAN = PI / 180.0;

// Histerezis (metre): Engel tam sınırdayken (ör. 2.00 m) durum DUR/DIKKAT arasında
// saniyede 10 kez gidip gelmesin diye, bir durumDAN ÇIKMAK için engelin sınırdan
// bu kadar daha uzaklaşması gerekir. Örn: DUR'dan çıkmak için 2.0 + 0.1 = 2.1 m.
constexpr double HISTEREZIS = 0.10;

// -----------------------------------------------------------------------------
// Tüm ayarlar tek bir yapıda. Parametre değişince önce bir KOPYA üzerinde
// kontrol edip, sadece geçerliyse asıl ayarların yerine koyuyoruz.
// -----------------------------------------------------------------------------
struct Ayarlar
{
  double merkez_aci = 0.0;        // derece: yelpazenin baktığı yön (0 = ön, +90 = sol)
  double gorus_genisligi = 90.0;  // derece: yelpazenin toplam açısı
  double min_mesafe = 0.3;        // metre: bundan yakın noktalar yok sayılır
  double dur_mesafesi = 2.0;      // metre: engel bundan yakınsa DUR
  double dikkat_mesafesi = 5.0;   // metre: engel bundan yakınsa DIKKAT (aynı zamanda işlenen en uzak mesafe)
  double z_min = -1.2;            // metre: bundan alçak noktalar yok sayılır (zemin)
  double z_max = 1.0;             // metre: bundan yüksek noktalar yok sayılır (tavan)
  int min_nokta_sayisi = 10;      // engel sayılması için gereken en az nokta
  bool canli_izleme = true;       // her turda terminale yaz
};

// -----------------------------------------------------------------------------
// Sektör testi için her mesajda BİR KEZ hesaplanan değerler.
// (Her nokta için tekrar tekrar cos/sin hesaplamamak için.)
// -----------------------------------------------------------------------------
struct Sektor
{
  double yon_x;      // Bakış yönünü gösteren birim vektörün x bileşeni = cos(merkez_aci)
  double yon_y;      // y bileşeni = sin(merkez_aci)
  double cos_yari;   // cos(gorus_genisligi / 2)
  bool tam_daire;    // gorus_genisligi 360 ise her nokta sektörün içindedir
};

Sektor sektorHazirla(const Ayarlar & a)
{
  Sektor s;
  s.yon_x = std::cos(a.merkez_aci * DERECE_RADYAN);
  s.yon_y = std::sin(a.merkez_aci * DERECE_RADYAN);
  s.cos_yari = std::cos(a.gorus_genisligi / 2.0 * DERECE_RADYAN);
  s.tam_daire = a.gorus_genisligi >= 360.0;
  return s;
}

// Nokta sektörün içinde mi?
//
// Fikir: Noktanın açısı ile bakış yönü arasındaki fark, yarı genişlikten küçük
// olmalı. İki vektör arasındaki açının kosinüsü = (nokta · yön) / r  (iç çarpım).
// Açı büyüdükçe kosinüs küçüldüğü için:
//     |açı farkı| <= yarı genişlik   <=>   (x*yon_x + y*yon_y) / r >= cos(yarı genişlik)
//
// Bu yöntem atan2 ve "-180..180'e normalize etme" işini gereksiz kılar; ±180°
// sınırında (ör. merkez 170°, nokta -170°) da kendiliğinden doğru çalışır.
bool sektorIcindeMi(double x, double y, double r, const Sektor & s)
{
  if (s.tam_daire) {
    return true;
  }
  return x * s.yon_x + y * s.yon_y >= r * s.cos_yari;
}

// Filtreyi geçen bir noktanın mesafesi ve konumu
struct EngelNoktasi
{
  double r;        // yatay mesafe (metre)
  double x, y, z;  // konum (metre)
};

// Yükseklik uygun mu? (zemin ve tavan dışarıda kalsın)
bool yukseklikUygunMu(double z, const Ayarlar & a)
{
  return z > a.z_min && z < a.z_max;
}

// Ayarlar birbiriyle mantıklı mı? Sorun yoksa boş metin, varsa sebebini döndürür.
std::string ayarlariKontrolEt(const Ayarlar & a)
{
  char tampon[160];
  if (a.dur_mesafesi >= a.dikkat_mesafesi) {
    std::snprintf(tampon, sizeof(tampon),
      "DUR mesafesi (%.2f) DIKKAT mesafesinden (%.2f) kucuk olmali",
      a.dur_mesafesi, a.dikkat_mesafesi);
    return tampon;
  }
  if (a.min_mesafe >= a.dur_mesafesi) {
    std::snprintf(tampon, sizeof(tampon),
      "Yakin sinir (%.2f) DUR mesafesinden (%.2f) kucuk olmali",
      a.min_mesafe, a.dur_mesafesi);
    return tampon;
  }
  if (a.z_min >= a.z_max) {
    std::snprintf(tampon, sizeof(tampon),
      "Alt yukseklik (%.2f) ust yukseklikten (%.2f) kucuk olmali", a.z_min, a.z_max);
    return tampon;
  }
  return "";
}

using EngelDurumu = robotaksi_msgs::msg::EngelDurumu;

// Başlangıçta "önceki durum" yok: ilk turda durum değişti diye terminale yazılsın.
constexpr uint8_t DURUM_YOK = 255;

const char * durumAdi(uint8_t durum)
{
  switch (durum) {
    case EngelDurumu::DUR: return "DUR";
    case EngelDurumu::DIKKAT: return "DIKKAT";
    case EngelDurumu::GUVENLI: return "GUVENLI";
    default: return "BASLANGIC";
  }
}

// Engel mesafesine ve bir önceki duruma bakarak yeni durumu belirler (histerezisli).
uint8_t durumBelirle(bool engel_var, double mesafe, const Ayarlar & a, uint8_t onceki)
{
  if (!engel_var) {
    return EngelDurumu::GUVENLI;
  }
  // Önceki durum DUR ise DUR'dan çıkmak için biraz daha uzaklaşmak gerekir
  const double dur_esik = a.dur_mesafesi + (onceki == EngelDurumu::DUR ? HISTEREZIS : 0.0);
  const double dikkat_esik = a.dikkat_mesafesi +
    ((onceki == EngelDurumu::DUR || onceki == EngelDurumu::DIKKAT) ? HISTEREZIS : 0.0);

  if (mesafe < dur_esik) {
    return EngelDurumu::DUR;
  }
  if (mesafe < dikkat_esik) {
    return EngelDurumu::DIKKAT;
  }
  return EngelDurumu::GUVENLI;
}

// -----------------------------------------------------------------------------
// Asıl node sınıfı
// -----------------------------------------------------------------------------
class EngelNode : public rclcpp::Node
{
public:
  EngelNode()
  : Node("engel_node")
  {
    parametreleriTanimla();

    std::string hata = ayarlariKontrolEt(ayarlar_);
    if (!hata.empty()) {
      RCLCPP_WARN(get_logger(), "Baslangic parametreleri mantiksiz: %s", hata.c_str());
    }

    // Çalışırken parametre değişirse çağrılacak fonksiyon
    param_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&EngelNode::parametreDegisti, this, std::placeholders::_1));

    // Durum + mesafe + durumu belirleyen nokta tek mesajda: okuyan taraf hepsini aynı turdan alır
    durum_pub_ = create_publisher<EngelDurumu>("/engel/durum", 10);
    nokta_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/engel/filtrelenmis_noktalar", 10);
    gorsel_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/engel/gorsel", 10);

    nokta_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/rslidar_points", rclcpp::SensorDataQoS(),
      std::bind(&EngelNode::noktaBulutuGeldi, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "engel_node basladi, /rslidar_points dinleniyor.");
  }

  // Node kapanırken (Ctrl+C / SIGTERM) RViz'deki yelpazeyi, yazıları ve filtrelenmiş
  // bulutu siler. RViz marker'ları silme mesajı gelene, bulutu da yenisi gelene kadar
  // ekranda tutar; bu olmazsa kapanan node'un son görüntüsü ekranda donup kalır.
  // ROS kapanmadan hemen önce, sinyal iş parçacığından çağrılır (main'e bakın).
  void ekraniTemizle()
  {
    // İşlenmekte olan tur bitsin, sonra yeni tur başlamasın: yoksa o tur silinen
    // çizimleri geri getirebilir.
    std::lock_guard<std::mutex> kilit(kapanis_mutex_);
    kapaniyor_ = true;
    if (son_bulut_sablonu_.header.frame_id.empty()) {
      return;  // hiç bulut gelmedi, ekranda bizden bir şey yok
    }

    visualization_msgs::msg::MarkerArray sil_dizi;
    visualization_msgs::msg::Marker sil;
    sil.header = son_bulut_sablonu_.header;
    sil.action = visualization_msgs::msg::Marker::DELETEALL;
    sil_dizi.markers.push_back(sil);
    gorsel_pub_->publish(sil_dizi);

    // Boş bulut, normal bulutla aynı alanlarla (RViz renk ayarı bozulmasın)
    nokta_pub_->publish(filtrelenmisBulutOlustur(son_bulut_sablonu_, {}, 0));

    // Süreç hemen kapanacak; mesajlar RViz'e ulaşmadan yok olmasın
    gorsel_pub_->wait_for_all_acked(std::chrono::milliseconds(300));
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
  }

private:
  // ---------------------------------------------------------------------------
  // Parametre tanımlama
  // ---------------------------------------------------------------------------
  double ondalikParametre(
    const std::string & ad, double varsayilan, double alt, double ust,
    const std::string & aciklama)
  {
    rcl_interfaces::msg::ParameterDescriptor tanim;
    tanim.description = aciklama;
    rcl_interfaces::msg::FloatingPointRange aralik;
    aralik.from_value = alt;
    aralik.to_value = ust;
    aralik.step = 0.0;  // 0 = adım zorunluluğu yok
    tanim.floating_point_range.push_back(aralik);
    return declare_parameter<double>(ad, varsayilan, tanim);
  }

  void parametreleriTanimla()
  {
    // --- 1) Nereye bakılsın? ---
    ayarlar_.merkez_aci = ondalikParametre("merkez_aci", 0.0, -180.0, 180.0,
      "Bakis yonu (derece). 0 = LiDAR'in onu, +90 = sol, -90 = sag, 180 = arka");
    ayarlar_.gorus_genisligi = ondalikParametre("gorus_genisligi", 90.0, 10.0, 360.0,
      "Yelpazenin toplam acisi (derece). 360 = her yon");

    // --- 2) Hangi noktalar yok sayılsın? ---
    ayarlar_.min_mesafe = ondalikParametre("min_mesafe", 0.3, 0.0, 3.0,
      "Bundan yakin noktalar yok sayilir (metre)");
    // z_min her zaman LiDAR'ın altında (<= 0), z_max her zaman üstünde (>= 0) olsun:
    // böylece yanlışlıkla "sadece 10 cm'lik şerit" gibi işe yaramaz ayarlar yapılamaz.
    ayarlar_.z_min = ondalikParametre("z_min", -1.2, -3.0, 0.0,
      "LiDAR merkezinden bu kadar asagisi yok sayilir (zemin, metre)");
    ayarlar_.z_max = ondalikParametre("z_max", 1.0, 0.0, 3.0,
      "LiDAR merkezinden bu kadar yukarisi yok sayilir (tavan, metre)");

    // --- 3) Ne zaman uyarı verilsin? ---
    ayarlar_.dur_mesafesi = ondalikParametre("dur_mesafesi", 2.0, 0.3, 10.0,
      "Engel bundan yakinsa DUR (metre)");
    ayarlar_.dikkat_mesafesi = ondalikParametre("dikkat_mesafesi", 5.0, 0.5, 20.0,
      "Engel bundan yakinsa DIKKAT (metre). Daha uzaklar hic islenmez");

    // --- 4) Gürültü filtresi ---
    // Bir "engel" sayılmak için görüş alanında en az bu kadar nokta olmalı.
    // Ayrıca engel mesafesi = en yakın N. noktanın mesafesi; yani tek tük
    // (N'den az) hatalı nokta ne kadar yakında olursa olsun kararı etkilemez.
    rcl_interfaces::msg::ParameterDescriptor nokta_tanim;
    nokta_tanim.description = "Engel sayilmasi icin gereken en az nokta (gurultu filtresi)";
    rcl_interfaces::msg::IntegerRange nokta_aralik;
    nokta_aralik.from_value = 1;
    nokta_aralik.to_value = 100;
    nokta_aralik.step = 1;
    nokta_tanim.integer_range.push_back(nokta_aralik);
    ayarlar_.min_nokta_sayisi = static_cast<int>(
      declare_parameter<int64_t>("min_nokta_sayisi", 10, nokta_tanim));

    rcl_interfaces::msg::ParameterDescriptor canli_tanim;
    canli_tanim.description = "Her turda durum ve mesafeyi terminale yaz";
    ayarlar_.canli_izleme = declare_parameter<bool>("canli_izleme", true, canli_tanim);
  }

  // Çalışırken parametre değişince ROS bu fonksiyonu çağırır
  rcl_interfaces::msg::SetParametersResult parametreDegisti(
    const std::vector<rclcpp::Parameter> & parametreler)
  {
    rcl_interfaces::msg::SetParametersResult sonuc;
    Ayarlar yeni = ayarlar_;  // Önce kopya üzerinde dene

    for (const auto & p : parametreler) {
      const std::string & ad = p.get_name();
      if (ad == "merkez_aci") {
        yeni.merkez_aci = p.as_double();
      } else if (ad == "gorus_genisligi") {
        yeni.gorus_genisligi = p.as_double();
      } else if (ad == "min_mesafe") {
        yeni.min_mesafe = p.as_double();
      } else if (ad == "z_min") {
        yeni.z_min = p.as_double();
      } else if (ad == "z_max") {
        yeni.z_max = p.as_double();
      } else if (ad == "dur_mesafesi") {
        yeni.dur_mesafesi = p.as_double();
      } else if (ad == "dikkat_mesafesi") {
        yeni.dikkat_mesafesi = p.as_double();
      } else if (ad == "min_nokta_sayisi") {
        yeni.min_nokta_sayisi = static_cast<int>(p.as_int());
      } else if (ad == "canli_izleme") {
        yeni.canli_izleme = p.as_bool();
      }
    }

    std::string hata = ayarlariKontrolEt(yeni);
    if (!hata.empty()) {
      sonuc.successful = false;
      sonuc.reason = hata;
      RCLCPP_WARN(get_logger(), "Parametre degisikligi reddedildi: %s", hata.c_str());
      return sonuc;
    }

    ayarlar_ = yeni;
    sonuc.successful = true;
    return sonuc;
  }

  // ---------------------------------------------------------------------------
  // Her nokta bulutu geldiğinde (saniyede ~10 kez) çalışan ana fonksiyon
  // ---------------------------------------------------------------------------
  void noktaBulutuGeldi(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> kilit(kapanis_mutex_);
    if (kapaniyor_) {
      return;
    }
    // Kapanışta boş bulut gönderebilmek için alanları sakla (veri kopyalanmaz)
    son_bulut_sablonu_.header = msg->header;
    son_bulut_sablonu_.fields = msg->fields;
    son_bulut_sablonu_.is_bigendian = msg->is_bigendian;
    son_bulut_sablonu_.point_step = msg->point_step;

    const Ayarlar a = ayarlar_;            // Bu tur boyunca sabit kalsın
    const Sektor s = sektorHazirla(a);     // cos/sin'i bir kez hesapla

    // Karekök almadan karşılaştırmak için mesafe sınırlarının KARELERİ.
    // Üst sınıra histerezis payını ekliyoruz ki DIKKAT'ten çıkış doğru çalışsın.
    const double ust_sinir = a.dikkat_mesafesi + HISTEREZIS;
    const double min_kare = a.min_mesafe * a.min_mesafe;
    const double max_kare = ust_sinir * ust_sinir;

    // Filtrelenmiş bulutu RViz'de izleyen yoksa hiç oluşturmayalım (boşuna kopyalama)
    const bool bulut_gerekli = nokta_pub_->get_subscription_count() > 0;

    // adaylar_ üye değişken: her turda yeniden bellek ayırmamak için tekrar kullanılıyor
    adaylar_.clear();
    std::vector<uint8_t> tutulan_veri;

    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
    const size_t nokta_boyu = msg->point_step;
    size_t indeks = 0;

    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++indeks) {
      const double x = *it_x;
      const double y = *it_y;
      const double z = *it_z;

      // 1) Geçersiz (NaN / sonsuz) noktaları at. Helios yansıma alamadığı yerlere NaN koyar.
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      // 2) Yükseklik testi (en ucuz test, en başta)
      if (!yukseklikUygunMu(z, a)) {
        continue;
      }
      // 3) Mesafe testi: karekök almadan kareleri karşılaştır
      const double r_kare = x * x + y * y;
      if (r_kare <= min_kare || r_kare >= max_kare) {
        continue;
      }
      // 4) Açı testi (sadece buraya kadar gelen noktalar için karekök alıyoruz)
      const double r = std::sqrt(r_kare);
      if (!sektorIcindeMi(x, y, r, s)) {
        continue;
      }

      // Nokta tüm filtreleri geçti
      adaylar_.push_back({r, x, y, z});
      if (bulut_gerekli) {
        const uint8_t * kaynak = &msg->data[indeks * nokta_boyu];
        tutulan_veri.insert(tutulan_veri.end(), kaynak, kaynak + nokta_boyu);
      }
    }

    // --- Engel mesafesi ---
    // En yakın TEK noktayı değil, en yakın N. noktayı (N = min_nokta_sayisi) kullanıyoruz.
    // Böylece havadaki tek bir hatalı nokta (toz, yansıma) kararı bozamaz:
    // "engel 1.4 m'de" demek için en az N noktanın 1.4 m'den yakın olması gerekir.
    const int tutulan_sayi = static_cast<int>(adaylar_.size());
    const bool engel_var = tutulan_sayi >= a.min_nokta_sayisi;
    double engel_mesafesi = -1.0;
    EngelNoktasi engel{-1.0, 0.0, 0.0, 0.0};
    if (engel_var) {
      auto n_inci = adaylar_.begin() + (a.min_nokta_sayisi - 1);
      // nth_element: diziyi tamamen sıralamadan N. en yakın noktayı yerine koyar (hızlı)
      std::nth_element(adaylar_.begin(), n_inci, adaylar_.end(),
        [](const EngelNoktasi & p, const EngelNoktasi & q) {return p.r < q.r;});
      engel = *n_inci;
      engel_mesafesi = engel.r;
    }
    // Kararı veren noktanın açısı (derece) — sadece bir nokta için atan2, ucuz
    const double engel_aci = std::atan2(engel.y, engel.x) / DERECE_RADYAN;

    const uint8_t durum = durumBelirle(engel_var, engel_mesafesi, a, onceki_durum_);

    // --- Terminale yaz ---
    if (durum != onceki_durum_) {
      RCLCPP_WARN(get_logger(), ">>> DURUM DEGISTI: %s -> %s",
        durumAdi(onceki_durum_), durumAdi(durum));
      onceki_durum_ = durum;
    }
    if (a.canli_izleme) {
      if (engel_var) {
        RCLCPP_INFO(get_logger(), "[%-7s] engel: %5.2f m | aci %+6.1f | yukseklik %+5.2f m | %5d nokta",
          durumAdi(durum), engel_mesafesi, engel_aci, engel.z, tutulan_sayi);
      } else {
        RCLCPP_INFO(get_logger(), "[%-7s] engel yok      | %5d nokta",
          durumAdi(durum), tutulan_sayi);
      }
    }

    // --- Yayınla ---
    EngelDurumu durum_msg;
    durum_msg.header = msg->header;
    durum_msg.durum = durum;
    durum_msg.engel_var = engel_var;
    // Engel yoksa +inf: "mesafe < esik" gibi bir karşılaştırma yanlışlıkla doğru çıkmasın
    durum_msg.mesafe = engel_var ? static_cast<float>(engel_mesafesi) :
      std::numeric_limits<float>::infinity();
    if (engel_var) {
      durum_msg.engel_noktasi.x = engel.x;
      durum_msg.engel_noktasi.y = engel.y;
      durum_msg.engel_noktasi.z = engel.z;
    }
    durum_pub_->publish(durum_msg);

    if (bulut_gerekli) {
      nokta_pub_->publish(filtrelenmisBulutOlustur(*msg, std::move(tutulan_veri), tutulan_sayi));
    }
    if (gorsel_pub_->get_subscription_count() > 0) {
      gorsel_pub_->publish(gorselOlustur(msg->header, a, durum, engel_var ? &engel : nullptr));
    }
  }

  // Tutulan noktalardan yeni bir PointCloud2 oluşturur (alanlar gelen mesajla aynı)
  sensor_msgs::msg::PointCloud2 filtrelenmisBulutOlustur(
    const sensor_msgs::msg::PointCloud2 & girdi, std::vector<uint8_t> && veri, int nokta_sayisi)
  {
    sensor_msgs::msg::PointCloud2 cikti;
    cikti.header = girdi.header;
    cikti.fields = girdi.fields;
    cikti.is_bigendian = girdi.is_bigendian;
    cikti.point_step = girdi.point_step;
    cikti.height = 1;
    cikti.width = static_cast<uint32_t>(nokta_sayisi);
    cikti.row_step = cikti.point_step * cikti.width;
    cikti.is_dense = true;
    cikti.data = std::move(veri);
    return cikti;
  }

  // ---------------------------------------------------------------------------
  // RViz görselleri
  // Hepsi z = 0'da, yani LiDAR'ın kendi yüksekliğinde çiziliyor. Böylece RViz'de
  // eğik açıdan bakınca da çizim ile noktalar aynı hizada görünür.
  // ---------------------------------------------------------------------------
  static std_msgs::msg::ColorRGBA renk(float r, float g, float b, float a = 1.0f)
  {
    std_msgs::msg::ColorRGBA c;
    c.r = r; c.g = g; c.b = b; c.a = a;
    return c;
  }

  static std_msgs::msg::ColorRGBA durumRengi(uint8_t durum, float a = 1.0f)
  {
    if (durum == EngelDurumu::DUR) {return renk(1.0f, 0.1f, 0.1f, a);}
    if (durum == EngelDurumu::DIKKAT) {return renk(1.0f, 0.85f, 0.0f, a);}
    return renk(0.1f, 0.9f, 0.2f, a);
  }

  static geometry_msgs::msg::Point nokta(double x, double y, double z = 0.0)
  {
    geometry_msgs::msg::Point p;
    p.x = x; p.y = y; p.z = z;
    return p;
  }

  // Sektör boyunca r yarıçaplı yayın noktaları (yaklaşık 2 derecede bir)
  static std::vector<geometry_msgs::msg::Point> yay(double r, const Ayarlar & a)
  {
    std::vector<geometry_msgs::msg::Point> noktalar;
    const int parca = std::max(4, static_cast<int>(std::ceil(a.gorus_genisligi / 2.0)));
    const double baslangic = a.merkez_aci - a.gorus_genisligi / 2.0;
    for (int i = 0; i <= parca; ++i) {
      const double aci = (baslangic + a.gorus_genisligi * i / parca) * DERECE_RADYAN;
      noktalar.push_back(nokta(r * std::cos(aci), r * std::sin(aci)));
    }
    return noktalar;
  }

  visualization_msgs::msg::Marker yeniMarker(
    const std_msgs::msg::Header & header, int id, int32_t tip)
  {
    visualization_msgs::msg::Marker m;
    m.header = header;  // aynı frame (rslidar) ve zaman damgası
    m.ns = "engel";
    m.id = id;
    m.type = tip;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    return m;
  }

  visualization_msgs::msg::MarkerArray gorselOlustur(
    const std_msgs::msg::Header & header, const Ayarlar & a,
    uint8_t durum, const EngelNoktasi * engel)
  {
    using visualization_msgs::msg::Marker;
    const double mesafe = engel ? engel->r : -1.0;
    visualization_msgs::msg::MarkerArray dizi;
    const bool tam_daire = a.gorus_genisligi >= 360.0;

    const auto ic_yay = yay(a.min_mesafe, a);        // yok sayılan yakın bölgenin sınırı
    const auto dikkat_yay = yay(a.dikkat_mesafesi, a);
    const auto dur_yay = yay(a.dur_mesafesi, a);

    // 0) İzlenen alan: min_mesafe ile dikkat_mesafesi arasındaki yarı saydam halka dilimi
    Marker alan = yeniMarker(header, 0, Marker::TRIANGLE_LIST);
    alan.scale.x = alan.scale.y = alan.scale.z = 1.0;
    alan.color = durumRengi(durum, 0.18f);
    for (size_t i = 0; i + 1 < dikkat_yay.size(); ++i) {
      // Her parça iki üçgenden oluşan bir dörtgen
      alan.points.push_back(ic_yay[i]);
      alan.points.push_back(dikkat_yay[i]);
      alan.points.push_back(dikkat_yay[i + 1]);
      alan.points.push_back(ic_yay[i]);
      alan.points.push_back(dikkat_yay[i + 1]);
      alan.points.push_back(ic_yay[i + 1]);
    }
    dizi.markers.push_back(alan);

    // 1) Dış sınır (sarı): DIKKAT mesafesi + yelpazenin kenarları
    Marker sinir = yeniMarker(header, 1, Marker::LINE_STRIP);
    sinir.scale.x = 0.03;
    sinir.color = renk(1.0f, 0.85f, 0.0f);
    sinir.points = dikkat_yay;
    if (!tam_daire) {
      // Dış yaydan iç yaya geri dön ve başlangıca kapat
      sinir.points.insert(sinir.points.end(), ic_yay.rbegin(), ic_yay.rend());
      sinir.points.push_back(dikkat_yay.front());
    }
    dizi.markers.push_back(sinir);

    // 2) DUR sınırı (kırmızı yay)
    Marker dur = yeniMarker(header, 2, Marker::LINE_STRIP);
    dur.scale.x = 0.04;
    dur.color = renk(1.0f, 0.1f, 0.1f);
    dur.points = dur_yay;
    dizi.markers.push_back(dur);

    // 3) LiDAR'ın önünü (0°, x ekseni) gösteren beyaz ok
    Marker ok = yeniMarker(header, 3, Marker::ARROW);
    ok.scale.x = 0.04;  // gövde kalınlığı
    ok.scale.y = 0.10;  // uç genişliği
    ok.scale.z = 0.15;  // uç uzunluğu
    ok.color = renk(1.0f, 1.0f, 1.0f);
    ok.points.push_back(nokta(0.0, 0.0));
    ok.points.push_back(nokta(1.0, 0.0));
    dizi.markers.push_back(ok);

    Marker on_yazi = yeniMarker(header, 4, Marker::TEXT_VIEW_FACING);
    on_yazi.pose.position = nokta(1.25, 0.0, 0.1);
    on_yazi.scale.z = 0.2;
    on_yazi.color = renk(1.0f, 1.0f, 1.0f);
    on_yazi.text = "ON (0)";
    dizi.markers.push_back(on_yazi);

    // 5) Durum yazısı
    Marker yazi = yeniMarker(header, 5, Marker::TEXT_VIEW_FACING);
    yazi.pose.position = nokta(0.0, 0.0, 1.5);
    yazi.scale.z = 0.4;
    yazi.color = durumRengi(durum);
    if (mesafe >= 0.0) {
      char tampon[32];
      std::snprintf(tampon, sizeof(tampon), "%s %.1f m", durumAdi(durum), mesafe);
      yazi.text = tampon;
    } else {
      yazi.text = durumAdi(durum);
    }
    dizi.markers.push_back(yazi);

    // 6) Kararı veren nokta: büyük bir top + oraya uzanan çizgi.
    //    "DUR diyor ama neden?" sorusunun cevabı bu top nerede duruyorsa orası.
    Marker top = yeniMarker(header, 6, Marker::SPHERE);
    Marker cizgi = yeniMarker(header, 7, Marker::LINE_LIST);
    if (engel) {
      top.pose.position = nokta(engel->x, engel->y, engel->z);
      top.scale.x = top.scale.y = top.scale.z = 0.25;
      top.color = durumRengi(durum);
      cizgi.scale.x = 0.02;
      cizgi.color = durumRengi(durum, 0.8f);
      cizgi.points.push_back(nokta(0.0, 0.0));
      cizgi.points.push_back(nokta(engel->x, engel->y, engel->z));
    } else {
      // Engel yoksa önceki topu ve çizgiyi RViz'den sil
      top.action = Marker::DELETE;
      cizgi.action = Marker::DELETE;
    }
    dizi.markers.push_back(top);
    dizi.markers.push_back(cizgi);

    return dizi;
  }

  // ---------------------------------------------------------------------------
  // Üye değişkenler
  // ---------------------------------------------------------------------------
  Ayarlar ayarlar_;
  uint8_t onceki_durum_ = DURUM_YOK;
  std::vector<EngelNoktasi> adaylar_;

  // Kapanışta ekranı temizlemek için (ekraniTemizle)
  std::mutex kapanis_mutex_;
  bool kapaniyor_ = false;
  sensor_msgs::msg::PointCloud2 son_bulut_sablonu_;  // son bulutun başlığı + alanları, verisiz

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr nokta_sub_;
  rclcpp::Publisher<EngelDurumu>::SharedPtr durum_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nokta_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr gorsel_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EngelNode>();
  // Ctrl+C / SIGTERM gelince rclcpp bunu ROS kapanmadan HEMEN ÖNCE çağırır (o an yayın
  // hâlâ mümkün): RViz'de son yelpaze/bulut asılı kalmasın.
  std::weak_ptr<EngelNode> zayif = node;
  rclcpp::contexts::get_global_default_context()->add_pre_shutdown_callback(
    [zayif]() {
      if (auto n = zayif.lock()) {n->ekraniTemizle();}
    });
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
