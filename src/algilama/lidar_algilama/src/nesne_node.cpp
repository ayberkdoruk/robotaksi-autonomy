// =============================================================================
// nesne_node.cpp
//
// LiDAR nokta bulutundan (/rslidar_points) nesneleri bulur:
//   zemin ayırma -> kümeleme -> dönük kutu + mesafe -> kaba etiket -> takip
// ve sonuçları /nesne/liste (robotaksi_msgs/NesneDizisi) ile RViz'e yayınlar.
//
// engel_node'dan (DUR / DIKKAT / GUVENLI) tamamen bağımsızdır, onun yanında çalışır.
// Sadece engel_node'un uyarı yelpazesi ayarlarını OKUR (hangi nesne yelpazenin
// içinde, göstermek için). engel_node kapalıysa bu özellik kendiliğinden kapanır.
//
// Algoritmalar src/nesne/ altında, ROS'tan bağımsız. Bu dosya sadece ROS tarafı.
// Eksenler (LiDAR'a göre): x = ileri, y = sol, z = yukarı, birim metre.
// =============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "robotaksi_msgs/msg/nesne.hpp"
#include "robotaksi_msgs/msg/nesne_dizisi.hpp"
#include "robotaksi_msgs/msg/zemin_durumu.hpp"

#include "nesne/kumeleme.hpp"
#include "nesne/kutu.hpp"
#include "nesne/takip.hpp"
#include "nesne/tipler.hpp"
#include "nesne/zemin.hpp"

namespace
{

constexpr double PI = 3.14159265358979323846;
constexpr double DERECE_RADYAN = PI / 180.0;

// Hareket etmiş (takip "hareketli" dedi) ve bundan hızlı nesnenin hızı yazılır, oku çizilir
constexpr double HAREKET_ESIGI = 0.3;  // m/s

// -----------------------------------------------------------------------------
// Tüm ayarlar tek yapıda. Parametre değişince önce KOPYA üzerinde kontrol edilir.
// -----------------------------------------------------------------------------
struct Ayarlar
{
  // Kutulama alanı (engel_node'un uyarı yelpazesinden AYRI)
  double nesne_merkez_aci = 0.0;        // derece: 0 = ön, +90 = sol
  double nesne_gorus_genisligi = 120.0; // derece: 360 = her yön
  // Ön filtre
  double menzil = 20.0;
  double govde_x_min = 0.0, govde_x_max = 0.0, govde_y_min = 0.0, govde_y_max = 0.0;
  double voxel_boyu = 0.08;
  // Zemin
  bool zemin_otomatik = true;
  double zemin_z = -0.81, zemin_bant = 0.30, zemin_esik = 0.10, zemin_max_egim = 10.0;
  int zemin_min_nokta = 30;
  double max_yukseklik = 2.5;
  // Zemin kontrolü (bulunan zemin beklenenle tutuyor mu)
  double zemin_tolerans = 0.10, egim_uyari = 3.0;
  // Kümeleme
  double kume_yatay_tol = 0.35, kume_dikey_taban = 0.25, kume_dikey_artis = 0.04;
  int kume_min_nokta = 5, kume_max_nokta = 20000;
  // Etiket
  double yaya_min_tepe = 1.0, yaya_max_tepe = 2.1, yaya_max_uzunluk = 1.2,
    yaya_max_genislik = 1.0, yapi_min_uzunluk = 2.5, yapi_max_genislik = 0.5,
    yapi_kesin_uzunluk = 6.0;
  // Takip
  double takip_kapi = 1.0, kutu_yumusatma = 0.7, hareket_mesafesi = 0.8;
  // Hiç hareket etmemiş nesneleri (uyarı alanındakiler hariç) yayınlama / çizme
  bool statik_gizle = false;
  int takip_dogrulama = 3, takip_kayip = 5;
  // Diğer
  bool canli_izleme = true;

  nesne::ZeminAyari zemin() const
  {
    nesne::ZeminAyari z;
    z.otomatik = zemin_otomatik;
    z.zemin_z = static_cast<float>(zemin_z);
    z.zemin_bant = static_cast<float>(zemin_bant);
    z.zemin_esik = static_cast<float>(zemin_esik);
    z.zemin_max_egim_derece = static_cast<float>(zemin_max_egim);
    z.max_yukseklik = static_cast<float>(max_yukseklik);
    z.zemin_min_nokta = zemin_min_nokta;
    return z;
  }
  nesne::KumeAyari kume() const
  {
    nesne::KumeAyari k;
    k.yatay_tol = static_cast<float>(kume_yatay_tol);
    k.dikey_taban = static_cast<float>(kume_dikey_taban);
    k.dikey_artis = static_cast<float>(kume_dikey_artis);
    k.min_nokta = kume_min_nokta;
    k.max_nokta = kume_max_nokta;
    return k;
  }
  nesne::EtiketAyari etiket() const
  {
    nesne::EtiketAyari e;
    e.yaya_min_tepe = static_cast<float>(yaya_min_tepe);
    e.yaya_max_tepe = static_cast<float>(yaya_max_tepe);
    e.yaya_max_uzunluk = static_cast<float>(yaya_max_uzunluk);
    e.yaya_max_genislik = static_cast<float>(yaya_max_genislik);
    e.yapi_min_uzunluk = static_cast<float>(yapi_min_uzunluk);
    e.yapi_max_genislik = static_cast<float>(yapi_max_genislik);
    e.yapi_kesin_uzunluk = static_cast<float>(yapi_kesin_uzunluk);
    return e;
  }
  nesne::TakipAyari takip() const
  {
    nesne::TakipAyari t;
    t.kapi = static_cast<float>(takip_kapi);
    t.kutu_yumusatma = static_cast<float>(kutu_yumusatma);
    t.hareket_mesafesi = static_cast<float>(hareket_mesafesi);
    t.dogrulama = takip_dogrulama;
    t.kayip = takip_kayip;
    return t;
  }
  bool govdeAcik() const {return govde_x_max > govde_x_min && govde_y_max > govde_y_min;}
};

// Parametre tabloları: ad, Ayarlar içindeki alan, varsayılan, aralık, açıklama.
// Tablo sayesinde tanımlama ve "parametre değişti" kodu her parametre için
// ayrı ayrı yazılmıyor.
struct OndalikTanim
{
  const char * ad;
  double Ayarlar::* alan;
  double alt, ust;
  const char * aciklama;
};

const OndalikTanim ONDALIKLAR[] = {
  {"nesne_merkez_aci", &Ayarlar::nesne_merkez_aci, -180.0, 180.0,
    "Kutulama alaninin baktigi yon (derece). 0 = on, +90 = sol, -90 = sag, 180 = arka"},
  {"nesne_gorus_genisligi", &Ayarlar::nesne_gorus_genisligi, 10.0, 360.0,
    "Kutulama alaninin toplam acisi (derece). 360 = her yon"},
  {"menzil", &Ayarlar::menzil, 1.0, 50.0, "Bundan uzak noktalar islenmez (yatay, metre)"},
  {"govde_x_min", &Ayarlar::govde_x_min, -10.0, 10.0,
    "Govde kutusu: icindeki noktalar yok sayilir (aracin/masanin kendisi). Hepsi 0 = kapali"},
  {"govde_x_max", &Ayarlar::govde_x_max, -10.0, 10.0, "Govde kutusu x ust siniri (metre)"},
  {"govde_y_min", &Ayarlar::govde_y_min, -10.0, 10.0, "Govde kutusu y alt siniri (metre)"},
  {"govde_y_max", &Ayarlar::govde_y_max, -10.0, 10.0, "Govde kutusu y ust siniri (metre)"},
  {"voxel_boyu", &Ayarlar::voxel_boyu, 0.0, 0.5,
    "Nokta seyreltme kup boyu (metre). 0 = kapali"},
  {"zemin_z", &Ayarlar::zemin_z, -4.0, -0.3,
    "Beklenen zemin yuksekligi, LiDAR'a gore (metre). Masada -0.81, aracta -montaj yuksekligi. "
    "zemin_otomatik acikken sadece kontrol icin (ve zemin hic bulunamazsa varsayilan)"},
  {"zemin_bant", &Ayarlar::zemin_bant, 0.05, 1.5,
    "Takipte zemin onceki duzlemin +- bu kadar yakininda aranir (otomatik kapaliysa ilk "
    "aramada da zemin_z +- bu kadar) (metre)"},
  {"zemin_esik", &Ayarlar::zemin_esik, 0.02, 0.5,
    "Zemin duzlemine bundan yakin nokta zemin sayilir (metre)"},
  {"zemin_max_egim", &Ayarlar::zemin_max_egim, 1.0, 30.0,
    "Zemin en fazla bu kadar egik olabilir (derece)"},
  {"max_yukseklik", &Ayarlar::max_yukseklik, 0.5, 6.0,
    "Zeminden bu kadar yukarisi (tavan, dal) yok sayilir (metre)"},
  {"zemin_tolerans", &Ayarlar::zemin_tolerans, 0.02, 1.0,
    "Bulunan zemin zemin_z'den bundan fazla farkliysa uyari verilir (metre)"},
  {"egim_uyari", &Ayarlar::egim_uyari, 0.5, 30.0,
    "Bulunan zemin bundan fazla egikse uyari verilir (derece)"},
  {"kume_yatay_tol", &Ayarlar::kume_yatay_tol, 0.05, 2.0,
    "Ayni nesne sayilmak icin en fazla yatay aralik (metre)"},
  {"kume_dikey_taban", &Ayarlar::kume_dikey_taban, 0.05, 2.0,
    "Dikey aralik esigi = taban + artis * mesafe (metre)"},
  {"kume_dikey_artis", &Ayarlar::kume_dikey_artis, 0.0, 0.2,
    "Dikey esigin mesafe basina artisi (metre / metre)"},
  {"yaya_min_tepe", &Ayarlar::yaya_min_tepe, 0.3, 3.0,
    "Yaya adayi: tepe noktasi zeminden en az (metre)"},
  {"yaya_max_tepe", &Ayarlar::yaya_max_tepe, 0.5, 3.0,
    "Yaya adayi: tepe noktasi zeminden en fazla (metre)"},
  {"yaya_max_uzunluk", &Ayarlar::yaya_max_uzunluk, 0.2, 3.0,
    "Yaya adayi: kutu uzunlugu en fazla (metre)"},
  {"yaya_max_genislik", &Ayarlar::yaya_max_genislik, 0.2, 3.0,
    "Yaya adayi: kutu genisligi en fazla (metre)"},
  {"yapi_min_uzunluk", &Ayarlar::yapi_min_uzunluk, 0.5, 20.0,
    "Yapi: en az bu kadar uzun ve yapi_max_genislik kadar ince (metre)"},
  {"yapi_max_genislik", &Ayarlar::yapi_max_genislik, 0.05, 3.0,
    "Yapi: en fazla bu kadar genis (metre)"},
  {"yapi_kesin_uzunluk", &Ayarlar::yapi_kesin_uzunluk, 1.0, 50.0,
    "Bundan uzun her sey yapi (metre)"},
  {"takip_kapi", &Ayarlar::takip_kapi, 0.1, 5.0,
    "Takip: tahmin ile kutu arasi en fazla bu kadar olabilir (metre)"},
  {"kutu_yumusatma", &Ayarlar::kutu_yumusatma, 0.0, 0.95,
    "Kutu titremesini azaltir: 0 = kapali (ham kutu), 0.9 = cok yumusak. Mesafe yumusatilmaz"},
  {"hareket_mesafesi", &Ayarlar::hareket_mesafesi, 0.2, 3.0,
    "Nesne ilk goruldugu / son hareket ettigi yerden bundan fazla uzaklasinca hareketli sayilir; "
    "5 sn yerinde durursa tekrar duran sayilir (metre)"},
};

struct TamsayiTanim
{
  const char * ad;
  int Ayarlar::* alan;
  int alt, ust;
  const char * aciklama;
};

const TamsayiTanim TAMSAYILAR[] = {
  {"zemin_min_nokta", &Ayarlar::zemin_min_nokta, 3, 5000,
    "Zemin duzleminin kabulu icin en az nokta"},
  {"kume_min_nokta", &Ayarlar::kume_min_nokta, 1, 1000, "Bundan az noktali kume gurultu sayilir"},
  {"kume_max_nokta", &Ayarlar::kume_max_nokta, 10, 200000, "Bundan cok noktali kume atilir"},
  {"takip_dogrulama", &Ayarlar::takip_dogrulama, 1, 20,
    "Yeni nesne bu kadar tur ust uste gorulunce numara alir"},
  {"takip_kayip", &Ayarlar::takip_kayip, 1, 50, "Nesne bu kadar tur gorulmezse silinir"},
};

// Ayarlar birbiriyle mantıklı mı? Sorun yoksa boş metin, varsa sebebi.
std::string ayarlariKontrolEt(const Ayarlar & a)
{
  // Gövde kutusu: min > max mantıksız. Eşitlik serbest (o eksen 0 genişlikte =
  // kutu kapalı); böylece değerler teker teker girilirken ara adımlar reddedilmez.
  if (a.govde_x_min > a.govde_x_max || a.govde_y_min > a.govde_y_max) {
    return "Govde kutusu: x_min <= x_max ve y_min <= y_max olmali";
  }
  if (a.kume_min_nokta > a.kume_max_nokta) {
    return "kume_min_nokta, kume_max_nokta'dan buyuk olamaz";
  }
  if (a.yaya_min_tepe >= a.yaya_max_tepe) {
    return "yaya_min_tepe, yaya_max_tepe'den kucuk olmali";
  }
  if (a.yapi_min_uzunluk >= a.yapi_kesin_uzunluk) {
    return "yapi_min_uzunluk, yapi_kesin_uzunluk'tan kucuk olmali";
  }
  return "";
}

// engel_node'un uyarı yelpazesi (okunabildiyse)
struct Yelpaze
{
  bool hazir = false;
  double merkez_aci = 0.0, gorus_genisligi = 90.0, min_mesafe = 0.3, dikkat_mesafesi = 5.0;

  // engel_node::sektorIcindeMi ile aynı formül + mesafe aralığı
  bool iceride(double x, double y) const
  {
    if (!hazir) {
      return false;
    }
    const double r = std::sqrt(x * x + y * y);
    if (r <= min_mesafe || r >= dikkat_mesafesi) {
      return false;
    }
    if (gorus_genisligi >= 360.0) {
      return true;
    }
    const double yon_x = std::cos(merkez_aci * DERECE_RADYAN);
    const double yon_y = std::sin(merkez_aci * DERECE_RADYAN);
    return x * yon_x + y * yon_y >= r * std::cos(gorus_genisligi / 2.0 * DERECE_RADYAN);
  }
};

}  // namespace

// -----------------------------------------------------------------------------
// Asıl node sınıfı
// -----------------------------------------------------------------------------
class NesneNode : public rclcpp::Node
{
public:
  NesneNode()
  : Node("nesne_node")
  {
    parametreleriTanimla();
    const std::string hata = ayarlariKontrolEt(ayarlar_);
    if (!hata.empty()) {
      RCLCPP_WARN(get_logger(), "Baslangic parametreleri mantiksiz: %s", hata.c_str());
    }
    param_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&NesneNode::parametreDegisti, this, std::placeholders::_1));

    liste_pub_ = create_publisher<robotaksi_msgs::msg::NesneDizisi>("/nesne/liste", 10);
    gorsel_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/nesne/gorsel", 10);
    kume_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/nesne/kumeler", 10);
    zemin_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/nesne/zemin", 10);
    zemin_disi_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/nesne/zemin_disi", 10);
    zemin_durumu_pub_ =
      create_publisher<robotaksi_msgs::msg::ZeminDurumu>("/nesne/zemin_durumu", 10);

    nokta_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/rslidar_points", rclcpp::SensorDataQoS(),
      std::bind(&NesneNode::noktaBulutuGeldi, this, std::placeholders::_1));

    yelpazeIzlemeyiBaslat();

    RCLCPP_INFO(get_logger(), "nesne_node basladi, /rslidar_points dinleniyor.");
  }

  // Node kapanırken (Ctrl+C / SIGTERM) RViz'deki son kutuları ve bulutları siler.
  // RViz marker'ları silme mesajı gelene, bulutları da yenisi gelene kadar ekranda tutar;
  // bu olmazsa kapanan node'un son görüntüsü ekranda donup kalır.
  // ROS kapanmadan hemen önce, sinyal iş parçacığından çağrılır (main'e bakın).
  void ekraniTemizle()
  {
    // İşlenmekte olan tur bitsin, sonra yeni tur başlamasın: yoksa o tur silinen
    // çizimleri geri getirebilir.
    std::lock_guard<std::mutex> kilit(kapanis_mutex_);
    kapaniyor_ = true;
    if (son_header_.frame_id.empty()) {
      return;  // hiç bulut gelmedi, ekranda bizden bir şey yok
    }

    visualization_msgs::msg::MarkerArray sil_dizi;
    visualization_msgs::msg::Marker sil;
    sil.header = son_header_;
    sil.action = visualization_msgs::msg::Marker::DELETEALL;
    sil_dizi.markers.push_back(sil);
    gorsel_pub_->publish(sil_dizi);

    // Boş bulutlar, normal bulutlarla aynı alanlarla (RViz renk ayarı bozulmasın)
    const nesne::Bulut bos;
    zemin_pub_->publish(bulutMesaji(bos, son_header_));
    zemin_disi_pub_->publish(bulutMesaji(bos, son_header_));
    kume_pub_->publish(kumeBulutu(bos, {}, son_header_));

    // Süreç hemen kapanacak; mesajlar RViz'e ulaşmadan yok olmasın
    gorsel_pub_->wait_for_all_acked(std::chrono::milliseconds(300));
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
  }

private:
  // ---------------------------------------------------------------------------
  // Parametreler
  // ---------------------------------------------------------------------------
  void parametreleriTanimla()
  {
    for (const auto & t : ONDALIKLAR) {
      rcl_interfaces::msg::ParameterDescriptor tanim;
      tanim.description = t.aciklama;
      rcl_interfaces::msg::FloatingPointRange aralik;
      aralik.from_value = t.alt;
      aralik.to_value = t.ust;
      aralik.step = 0.0;
      tanim.floating_point_range.push_back(aralik);
      ayarlar_.*t.alan = declare_parameter<double>(t.ad, ayarlar_.*t.alan, tanim);
    }
    for (const auto & t : TAMSAYILAR) {
      rcl_interfaces::msg::ParameterDescriptor tanim;
      tanim.description = t.aciklama;
      rcl_interfaces::msg::IntegerRange aralik;
      aralik.from_value = t.alt;
      aralik.to_value = t.ust;
      aralik.step = 1;
      tanim.integer_range.push_back(aralik);
      ayarlar_.*t.alan = static_cast<int>(
        declare_parameter<int64_t>(t.ad, ayarlar_.*t.alan, tanim));
    }
    rcl_interfaces::msg::ParameterDescriptor otomatik;
    otomatik.description =
      "true: zemin acilista ipucusuz aranir (LiDAR'in altindaki en alcak genis yatay yuzey); "
      "zemin_z sadece kontrol icin kullanilir. false: zemin sadece zemin_z +- zemin_bant icinde aranir";
    ayarlar_.zemin_otomatik = declare_parameter<bool>("zemin_otomatik", true, otomatik);

    rcl_interfaces::msg::ParameterDescriptor statik;
    statik.description =
      "true: hic hareket etmemis nesneler /nesne/liste'de ve RViz'de gosterilmez (uyari "
      "alanindakiler haric). Takip arkada surer; nesne hareket edince hemen gorunur, "
      "5 sn yerinde durunca tekrar gizlenir";
    ayarlar_.statik_gizle = declare_parameter<bool>("statik_gizle", false, statik);

    rcl_interfaces::msg::ParameterDescriptor canli;
    canli.description = "Saniyede bir ozet satiri yaz (nesne sayisi, en yakin, islem suresi)";
    ayarlar_.canli_izleme = declare_parameter<bool>("canli_izleme", true, canli);

    // Uyarı yelpazesi ayarlarının okunacağı node (çalışırken değiştirilemez)
    rcl_interfaces::msg::ParameterDescriptor yelpaze;
    yelpaze.description = "Uyari yelpazesi ayarlarinin okunacagi node";
    yelpaze.read_only = true;
    yelpaze_node_ = declare_parameter<std::string>("yelpaze_node", "engel_node", yelpaze);
  }

  rcl_interfaces::msg::SetParametersResult parametreDegisti(
    const std::vector<rclcpp::Parameter> & parametreler)
  {
    rcl_interfaces::msg::SetParametersResult sonuc;
    Ayarlar yeni = ayarlar_;
    for (const auto & p : parametreler) {
      const std::string & ad = p.get_name();
      for (const auto & t : ONDALIKLAR) {
        if (ad == t.ad) {yeni.*t.alan = p.as_double();}
      }
      for (const auto & t : TAMSAYILAR) {
        if (ad == t.ad) {yeni.*t.alan = static_cast<int>(p.as_int());}
      }
      if (ad == "canli_izleme") {yeni.canli_izleme = p.as_bool();}
      if (ad == "zemin_otomatik") {yeni.zemin_otomatik = p.as_bool();}
      if (ad == "statik_gizle") {yeni.statik_gizle = p.as_bool();}
    }
    const std::string hata = ayarlariKontrolEt(yeni);
    if (!hata.empty()) {
      sonuc.successful = false;
      sonuc.reason = hata;
      RCLCPP_WARN(get_logger(), "Parametre degisikligi reddedildi: %s", hata.c_str());
      return sonuc;
    }
    // Arama yöntemi değiştiyse eski düzlemi unut, ilk aramayı yeniden yap. Otomatikte
    // zemin_z sadece kontrol için kullanıldığından onun değişmesi aramayı etkilemez.
    if (yeni.zemin_otomatik != ayarlar_.zemin_otomatik ||
      (!yeni.zemin_otomatik && yeni.zemin_z != ayarlar_.zemin_z))
    {
      zemin_.sifirla();
    }
    ayarlar_ = yeni;
    sonuc.successful = true;
    return sonuc;
  }

  // ---------------------------------------------------------------------------
  // engel_node'un uyarı yelpazesini izleme
  //   - Açılışta (ve engel_node sonradan açılırsa) parametreleri bir kez okur
  //   - Sonra /parameter_events ile değişiklikleri dinler (panelden ayar değişince)
  // ---------------------------------------------------------------------------
  void yelpazeIzlemeyiBaslat()
  {
    param_istemci_ = std::make_shared<rclcpp::AsyncParametersClient>(this, yelpaze_node_);

    param_olay_sub_ = create_subscription<rcl_interfaces::msg::ParameterEvent>(
      "/parameter_events", rclcpp::ParameterEventsQoS(),
      [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr olay) {
        if (olay->node != "/" + yelpaze_node_) {
          return;
        }
        for (const auto * liste : {&olay->new_parameters, &olay->changed_parameters}) {
          for (const auto & p : *liste) {
            yelpazeParametresi(p.name, rclcpp::Parameter::from_parameter_msg(p));
          }
        }
      });

    // engel_node hazır olana kadar 2 sn'de bir dene
    yelpaze_zamanlayici_ = create_wall_timer(std::chrono::seconds(2), [this]() {
        if (yelpaze_.hazir || yelpaze_istek_bekliyor_ || !param_istemci_->service_is_ready()) {
          return;
        }
        yelpaze_istek_bekliyor_ = true;
        param_istemci_->get_parameters(
          {"merkez_aci", "gorus_genisligi", "min_mesafe", "dikkat_mesafesi"},
          [this](std::shared_future<std::vector<rclcpp::Parameter>> gelecek) {
            yelpaze_istek_bekliyor_ = false;
            try {
              for (const auto & p : gelecek.get()) {
                yelpazeParametresi(p.get_name(), p);
              }
              yelpaze_.hazir = true;
              RCLCPP_INFO(get_logger(),
                "Uyari yelpazesi okundu (%s): merkez %.0f, genislik %.0f, %.1f-%.1f m",
                yelpaze_node_.c_str(), yelpaze_.merkez_aci, yelpaze_.gorus_genisligi,
                yelpaze_.min_mesafe, yelpaze_.dikkat_mesafesi);
            } catch (const std::exception & e) {
              RCLCPP_WARN(get_logger(), "Uyari yelpazesi okunamadi: %s", e.what());
            }
          });
      });
  }

  void yelpazeParametresi(const std::string & ad, const rclcpp::Parameter & p)
  {
    if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
      return;
    }
    if (ad == "merkez_aci") {yelpaze_.merkez_aci = p.as_double();}
    else if (ad == "gorus_genisligi") {yelpaze_.gorus_genisligi = p.as_double();}
    else if (ad == "min_mesafe") {yelpaze_.min_mesafe = p.as_double();}
    else if (ad == "dikkat_mesafesi") {yelpaze_.dikkat_mesafesi = p.as_double();}
  }

  // ---------------------------------------------------------------------------
  // Her nokta bulutunda (saniyede ~10 kez) çalışan ana fonksiyon
  // ---------------------------------------------------------------------------
  void noktaBulutuGeldi(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> kilit(kapanis_mutex_);
    if (kapaniyor_) {
      return;
    }
    son_header_ = msg->header;

    const auto baslangic = std::chrono::steady_clock::now();
    const Ayarlar a = ayarlar_;

    // --- 1) Ön filtre: NaN, menzil, gövde kutusu ---
    nesne::Bulut::Ptr filtreli(new nesne::Bulut);
    filtreli->reserve(msg->width * msg->height);
    const double menzil_kare = a.menzil * a.menzil;
    const bool govde = a.govdeAcik();
    // Kutulama alanı testi (engel_node'daki yelpaze testiyle aynı fikir):
    // nokta ile bakış yönü arasındaki açı yarı genişlikten küçükse içeride.
    const bool tam_daire = a.nesne_gorus_genisligi >= 360.0;
    const double yon_x = std::cos(a.nesne_merkez_aci * DERECE_RADYAN);
    const double yon_y = std::sin(a.nesne_merkez_aci * DERECE_RADYAN);
    const double cos_yari = std::cos(a.nesne_gorus_genisligi / 2.0 * DERECE_RADYAN);
    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      const float x = *it_x, y = *it_y, z = *it_z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      const double r_kare = static_cast<double>(x) * x + static_cast<double>(y) * y;
      if (r_kare > menzil_kare) {
        continue;
      }
      if (!tam_daire && x * yon_x + y * yon_y < std::sqrt(r_kare) * cos_yari) {
        continue;
      }
      if (govde && x >= a.govde_x_min && x <= a.govde_x_max &&
        y >= a.govde_y_min && y <= a.govde_y_max)
      {
        continue;
      }
      filtreli->push_back(nesne::Nokta(x, y, z));
    }
    const size_t giris_sayisi = filtreli->size();

    // --- 2) Seyreltme: her voxel_boyu küpünde tek nokta kalsın ---
    nesne::Bulut::Ptr seyrek = filtreli;
    if (a.voxel_boyu > 0.0 && !filtreli->empty()) {
      seyrek.reset(new nesne::Bulut);
      pcl::VoxelGrid<nesne::Nokta> voxel;
      voxel.setInputCloud(filtreli);
      const float v = static_cast<float>(a.voxel_boyu);
      voxel.setLeafSize(v, v, v);
      voxel.filter(*seyrek);
    }

    // --- 3) Zemin ayırma ---
    const auto zemin_sonuc = zemin_.ayir(*seyrek, a.zemin());
    zeminiKontrolEt(msg->header, zemin_sonuc, a);

    // --- 4) Kümeleme ---
    const auto kumeler = nesne::kumele(zemin_sonuc.nesne, a.kume());

    // --- 5) Kutu + etiket ---
    const nesne::EtiketAyari etiket_ayari = a.etiket();
    std::vector<nesne::Kutu> kutular;
    std::vector<std::string> etiketler;
    kutular.reserve(kumeler.size());
    etiketler.reserve(kumeler.size());
    for (const auto & kume : kumeler) {
      kutular.push_back(nesne::kutuHesapla(*zemin_sonuc.nesne, kume, zemin_.duzlem()));
      etiketler.push_back(nesne::etiketVer(kutular.back(), etiket_ayari));
    }

    // --- 6) Takip ---
    const double zaman = rclcpp::Time(msg->header.stamp).seconds();
    const auto izler = takipci_.guncelle(kutular, etiketler, zaman, a.takip());

    // --- 7) Yayınla ---
    const auto liste = listeOlustur(msg->header, izler, a);
    const size_t gizlenen = izler.size() - liste.nesneler.size();
    liste_pub_->publish(liste);

    if (gorsel_pub_->get_subscription_count() > 0) {
      gorsel_pub_->publish(gorselOlustur(msg->header, liste, a));
    }
    if (zemin_pub_->get_subscription_count() > 0) {
      zemin_pub_->publish(bulutMesaji(*zemin_sonuc.zemin, msg->header));
    }
    if (zemin_disi_pub_->get_subscription_count() > 0) {
      zemin_disi_pub_->publish(bulutMesaji(*zemin_sonuc.nesne, msg->header));
    }
    if (kume_pub_->get_subscription_count() > 0) {
      kume_pub_->publish(kumeBulutu(*zemin_sonuc.nesne, kumeler, msg->header));
    }

    // --- 8) Terminal özeti (saniyede bir) ---
    const double sure_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - baslangic).count();
    if (a.canli_izleme) {
      int yaya = 0;
      const robotaksi_msgs::msg::Nesne * en_yakin = nullptr;
      for (const auto & n : liste.nesneler) {
        if (n.etiket == nesne::ETIKET_YAYA) {++yaya;}
        if (!en_yakin || n.mesafe < en_yakin->mesafe) {en_yakin = &n;}
      }
      char yakin_metin[96] = "yok";
      if (en_yakin) {
        std::snprintf(yakin_metin, sizeof(yakin_metin), "#%u %s %.2f m",
          en_yakin->id, en_yakin->etiket.c_str(), en_yakin->mesafe);
      }
      const auto & d = zemin_.duzlem();
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "[NESNE] nokta %zu->%zu | zemin %zu, nesne %zu | kume %zu | iz %zu (yaya %d, gizli %zu) | "
        "en yakin: %s | zemin z %.2f egim %.1f (%s, %s) | %.1f ms",
        giris_sayisi, seyrek->size(), zemin_sonuc.zemin->size(), zemin_sonuc.nesne->size(),
        kumeler.size(), liste.nesneler.size(), yaya, gizlenen, yakin_metin, d.merkezZ(),
        egimDerece(d), kaynakAdi(zemin_sonuc.kaynak), zemin_uyumlu_ ? "uyumlu" : "UYUMSUZ",
        sure_ms);
    }
  }

  // ---------------------------------------------------------------------------
  // Zemin kontrolü: bulunan zemin beklenenle (zemin_z, egim_uyari) tutuyor mu?
  // Sonuç her turda /nesne/zemin_durumu ile yayınlanır; durum değişince terminale yazılır.
  // LiDAR yerinden oynarsa ya da yanlış profil yüklüyse (ör. araçta masa ayarı) böylece fark edilir.
  // ---------------------------------------------------------------------------
  static double egimDerece(const nesne::Duzlem & d)
  {
    return std::acos(std::min(1.0f, d.c)) / DERECE_RADYAN;
  }

  static const char * kaynakAdi(nesne::ZeminKaynagi k)
  {
    switch (k) {
      case nesne::ZeminKaynagi::OLCULDU: return "olculdu";
      case nesne::ZeminKaynagi::ONCEKI: return "onceki";
      default: return "varsayilan";
    }
  }

  void zeminiKontrolEt(
    const std_msgs::msg::Header & header, const nesne::ZeminAyirici::Sonuc & s, const Ayarlar & a)
  {
    const nesne::Duzlem & d = zemin_.duzlem();
    const double yukseklik = d.merkezZ();
    const double egim = egimDerece(d);
    const bool varsayilan = s.kaynak == nesne::ZeminKaynagi::VARSAYILAN;
    const bool yukseklik_tutuyor = std::fabs(yukseklik - a.zemin_z) <= a.zemin_tolerans;
    const bool egim_tutuyor = egim <= a.egim_uyari;
    const bool simdi_uyumlu = !varsayilan && yukseklik_tutuyor && egim_tutuyor;

    // Otomatik ilk arama sonucu
    if (s.ilk_arama && s.kaynak == nesne::ZeminKaynagi::OLCULDU) {
      if (a.zemin_otomatik) {
        RCLCPP_INFO(get_logger(),
          "Zemin otomatik bulundu: z %.2f m, egim %.1f derece (%d yatay yuzeyin en alcagi; "
          "beklenen z %.2f)", yukseklik, egim, s.yatay_yuzey_sayisi, a.zemin_z);
      } else {
        RCLCPP_INFO(get_logger(), "Zemin zemin_z yakininda bulundu: z %.2f m, egim %.1f derece",
          yukseklik, egim);
      }
    }

    // Uyum durumu: titremesin diye yeni durum UYUM_TUR tur üst üste görülünce değişir
    if (zemin_uyum_ilk_) {
      zemin_uyumlu_ = simdi_uyumlu;
      zemin_uyum_ilk_ = false;
      zemin_uyum_sayac_ = UYUM_TUR;  // ilk durumu da aşağıda bir kez yaz
    } else if (simdi_uyumlu != zemin_uyumlu_) {
      if (++zemin_uyum_sayac_ >= UYUM_TUR) {
        zemin_uyumlu_ = simdi_uyumlu;
      }
    } else {
      zemin_uyum_sayac_ = 0;
    }
    if (zemin_uyum_sayac_ >= UYUM_TUR) {
      zemin_uyum_sayac_ = 0;
      if (zemin_uyumlu_) {
        RCLCPP_INFO(get_logger(), "Zemin beklenenle uyumlu: z %.2f m (beklenen %.2f), egim %.1f derece",
          yukseklik, a.zemin_z, egim);
      } else if (varsayilan) {
        RCLCPP_WARN(get_logger(),
          "Zemin bulunamadi; beklenen yukseklikte (zemin_z = %.2f) duz zemin varsayiliyor. "
          "LiDAR yeri goremiyor olabilir (ortam cok dolu ya da LiDAR yukari bakiyor).", a.zemin_z);
      } else if (!yukseklik_tutuyor) {
        RCLCPP_WARN(get_logger(),
          "Zemin %.2f m'de bulundu ama beklenen %.2f m (fark %.0f cm > %.0f cm). LiDAR yerinden "
          "oynamis ya da yanlis profil yuklu olabilir. Olcum dogruysa zemin_z'yi %.2f yapin.",
          yukseklik, a.zemin_z, std::fabs(yukseklik - a.zemin_z) * 100.0,
          a.zemin_tolerans * 100.0, yukseklik);
      } else {
        RCLCPP_WARN(get_logger(),
          "Zemin %.1f derece egik (sinir %.1f). LiDAR yamuk duruyor olabilir.", egim, a.egim_uyari);
      }
    }

    robotaksi_msgs::msg::ZeminDurumu m;
    m.header = header;
    m.kaynak = static_cast<uint8_t>(s.kaynak);
    m.otomatik = a.zemin_otomatik;
    m.yukseklik = static_cast<float>(yukseklik);
    m.egim = static_cast<float>(egim);
    m.duzlem = {d.a, d.b, d.c, d.d};
    m.beklenen_yukseklik = static_cast<float>(a.zemin_z);
    m.yukseklik_toleransi = static_cast<float>(a.zemin_tolerans);
    m.egim_siniri = static_cast<float>(a.egim_uyari);
    m.uyumlu = zemin_uyumlu_;
    zemin_durumu_pub_->publish(m);
  }

  // ---------------------------------------------------------------------------
  // /nesne/liste mesajı
  // ---------------------------------------------------------------------------
  // statik_gizle açıksa hiç hareket etmemiş nesneler listeye girmez; uyarı alanındakiler
  // (aracın önündeki duran engeller) her zaman girer.
  robotaksi_msgs::msg::NesneDizisi listeOlustur(
    const std_msgs::msg::Header & header, const std::vector<nesne::Takipci::Iz> & izler,
    const Ayarlar & a) const
  {
    robotaksi_msgs::msg::NesneDizisi dizi;
    dizi.header = header;
    dizi.nesneler.reserve(izler.size());
    for (const auto & iz : izler) {
      const auto & k = iz.kutu;
      const bool uyari_alaninda = yelpaze_.iceride(k.yakin_x, k.yakin_y);
      if (a.statik_gizle && !iz.hareketli && !uyari_alaninda) {
        continue;
      }
      robotaksi_msgs::msg::Nesne n;
      n.id = iz.id;
      n.etiket = iz.etiket;
      n.merkez.x = k.cx;
      n.merkez.y = k.cy;
      n.merkez.z = k.cz;
      n.boyut.x = k.uzunluk;
      n.boyut.y = k.genislik;
      n.boyut.z = k.yukseklik;
      n.yon = k.yon;
      n.en_yakin_nokta.x = k.yakin_x;
      n.en_yakin_nokta.y = k.yakin_y;
      n.en_yakin_nokta.z = k.yakin_z;
      n.mesafe = k.mesafe;
      n.tepe_yuksekligi = k.tepe_yuksekligi;
      // Yapılar (duvar vb.) hareket etmez. Görünen kısmı değiştikçe kutu merkezi
      // kaydığı için Kalman sahte bir hız hesaplar; onu yayınlamayalım.
      const bool yapi = iz.etiket == nesne::ETIKET_YAPI;
      const double vx = yapi ? 0.0 : iz.vx;
      const double vy = yapi ? 0.0 : iz.vy;
      n.hiz.x = vx;
      n.hiz.y = vy;
      n.nokta_sayisi = static_cast<uint32_t>(k.nokta_sayisi);
      n.yas = iz.yas;

      // Yaklaşma hızı: hızın LiDAR'a doğru bileşeni (+ = yaklaşıyor)
      const double r = std::sqrt(k.cx * k.cx + k.cy * k.cy);
      n.yaklasma_hizi = r > 1e-3 ? static_cast<float>(-(vx * k.cx + vy * k.cy) / r) : 0.0f;
      n.carpisma_suresi = n.yaklasma_hizi > 0.05f ? n.mesafe / n.yaklasma_hizi : -1.0f;

      n.uyari_alaninda = uyari_alaninda;
      n.hareketli = iz.hareketli;

      // Güven (kural tabanlı): etiket kuralına uyuyor 0.6, hareket etmiş yaya +0.3,
      // uzun süredir takipte +0.1
      double guven = 0.6;
      if (iz.etiket == nesne::ETIKET_YAYA && iz.hareketli) {guven += 0.3;}
      if (iz.yas >= 10) {guven += 0.1;}
      n.guven = static_cast<float>(std::min(1.0, guven));

      dizi.nesneler.push_back(n);
    }
    return dizi;
  }

  // ---------------------------------------------------------------------------
  // Hata ayıklama bulutları
  // ---------------------------------------------------------------------------
  static sensor_msgs::msg::PointCloud2 bulutMesaji(
    const nesne::Bulut & bulut, const std_msgs::msg::Header & header)
  {
    sensor_msgs::msg::PointCloud2 m;
    pcl::toROSMsg(bulut, m);
    m.header = header;
    return m;
  }

  // Her küme ayrı renkte (komşu kümeler birbirinden ayırt edilsin diye altın oran ile ton)
  static sensor_msgs::msg::PointCloud2 kumeBulutu(
    const nesne::Bulut & bulut, const std::vector<std::vector<int>> & kumeler,
    const std_msgs::msg::Header & header)
  {
    pcl::PointCloud<pcl::PointXYZRGB> renkli;
    for (size_t i = 0; i < kumeler.size(); ++i) {
      const double ton = std::fmod(i * 0.618033988749895, 1.0);
      const auto c = tondanRenk(ton);
      for (int j : kumeler[i]) {
        pcl::PointXYZRGB p;
        p.x = bulut[j].x;
        p.y = bulut[j].y;
        p.z = bulut[j].z;
        p.r = static_cast<uint8_t>(c.r * 255);
        p.g = static_cast<uint8_t>(c.g * 255);
        p.b = static_cast<uint8_t>(c.b * 255);
        renkli.push_back(p);
      }
    }
    sensor_msgs::msg::PointCloud2 m;
    pcl::toROSMsg(renkli, m);
    m.header = header;
    return m;
  }

  // ---------------------------------------------------------------------------
  // RViz görselleri
  // ---------------------------------------------------------------------------
  static std_msgs::msg::ColorRGBA renk(float r, float g, float b, float a = 1.0f)
  {
    std_msgs::msg::ColorRGBA c;
    c.r = r; c.g = g; c.b = b; c.a = a;
    return c;
  }

  // Ton (0..1) -> doygun renk (HSV, S = V = 1)
  static std_msgs::msg::ColorRGBA tondanRenk(double ton)
  {
    const double h = ton * 6.0;
    const double x = 1.0 - std::fabs(std::fmod(h, 2.0) - 1.0);
    double r = 0, g = 0, b = 0;
    switch (static_cast<int>(h) % 6) {
      case 0: r = 1; g = x; break;
      case 1: r = x; g = 1; break;
      case 2: g = 1; b = x; break;
      case 3: g = x; b = 1; break;
      case 4: r = x; b = 1; break;
      default: r = 1; b = x; break;
    }
    return renk(static_cast<float>(r), static_cast<float>(g), static_cast<float>(b));
  }

  // Etiket rengi: yaya adayı mavi, engel turuncu, yapı gri
  static std_msgs::msg::ColorRGBA etiketRengi(const std::string & etiket, float a = 1.0f)
  {
    if (etiket == nesne::ETIKET_YAYA) {return renk(0.2f, 0.55f, 1.0f, a);}
    if (etiket == nesne::ETIKET_YAPI) {return renk(0.6f, 0.6f, 0.6f, a);}
    return renk(1.0f, 0.55f, 0.0f, a);
  }

  static const char * etiketYazisi(const std::string & etiket)
  {
    if (etiket == nesne::ETIKET_YAYA) {return "YAYA?";}
    if (etiket == nesne::ETIKET_YAPI) {return "YAPI";}
    return "ENGEL";
  }

  static geometry_msgs::msg::Point nokta(double x, double y, double z)
  {
    geometry_msgs::msg::Point p;
    p.x = x; p.y = y; p.z = z;
    return p;
  }

  visualization_msgs::msg::MarkerArray gorselOlustur(
    const std_msgs::msg::Header & header, const robotaksi_msgs::msg::NesneDizisi & liste,
    const Ayarlar & a)
  {
    using visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray dizi;

    // Önce önceki turun tüm kutularını sil (kaybolan nesnelerin kutusu ekranda kalmasın)
    Marker sil;
    sil.header = header;
    sil.ns = "nesne";
    sil.action = Marker::DELETEALL;
    dizi.markers.push_back(sil);

    // Kutulama alanının sınırı: ince açık mavi çizgi, menzil yarıçapında, z = 0
    {
      Marker alan;
      alan.header = header;
      alan.ns = "nesne_alan";
      alan.id = 0;
      alan.type = Marker::LINE_STRIP;
      alan.action = Marker::ADD;
      alan.pose.orientation.w = 1.0;
      alan.scale.x = 0.03;
      alan.color = renk(0.4f, 0.75f, 1.0f, 0.6f);
      const bool tam = a.nesne_gorus_genisligi >= 360.0;
      if (!tam) {alan.points.push_back(nokta(0.0, 0.0, 0.0));}
      const int parca = std::max(8, static_cast<int>(std::ceil(a.nesne_gorus_genisligi / 2.0)));
      const double bas = a.nesne_merkez_aci - a.nesne_gorus_genisligi / 2.0;
      for (int i = 0; i <= parca; ++i) {
        const double aci = (bas + a.nesne_gorus_genisligi * i / parca) * DERECE_RADYAN;
        alan.points.push_back(nokta(a.menzil * std::cos(aci), a.menzil * std::sin(aci), 0.0));
      }
      if (!tam) {alan.points.push_back(nokta(0.0, 0.0, 0.0));}
      dizi.markers.push_back(alan);
    }

    int id = 0;
    for (const auto & n : liste.nesneler) {
      const double c = std::cos(n.yon), s = std::sin(n.yon);
      const double yl = n.boyut.x / 2.0, yg = n.boyut.y / 2.0;
      const double alt = n.merkez.z - n.boyut.z / 2.0, ust = n.merkez.z + n.boyut.z / 2.0;
      const auto renk_ = etiketRengi(n.etiket);

      auto yeni = [&](int32_t tip) {
          Marker m;
          m.header = header;
          m.ns = "nesne";
          m.id = id++;
          m.type = tip;
          m.action = Marker::ADD;
          m.pose.orientation.w = 1.0;
          return m;
        };

      // 1) Kutu: 12 kenarlı kafes
      Marker kafes = yeni(Marker::LINE_LIST);
      kafes.scale.x = n.uyari_alaninda ? 0.08 : 0.03;  // yelpaze içindeyse kalın
      kafes.color = renk_;
      geometry_msgs::msg::Point kose[8];
      const double isaret[4][2] = {{1, 1}, {-1, 1}, {-1, -1}, {1, -1}};
      for (int i = 0; i < 4; ++i) {
        const double u = isaret[i][0] * yl, v = isaret[i][1] * yg;
        const double x = n.merkez.x + c * u - s * v;
        const double y = n.merkez.y + s * u + c * v;
        kose[i] = nokta(x, y, alt);
        kose[i + 4] = nokta(x, y, ust);
      }
      for (int i = 0; i < 4; ++i) {
        const int j = (i + 1) % 4;
        kafes.points.push_back(kose[i]); kafes.points.push_back(kose[j]);          // alt
        kafes.points.push_back(kose[i + 4]); kafes.points.push_back(kose[j + 4]);  // üst
        kafes.points.push_back(kose[i]); kafes.points.push_back(kose[i + 4]);      // dikey
      }
      dizi.markers.push_back(kafes);

      // 2) Yelpaze içindeyse yarı saydam dolgu
      if (n.uyari_alaninda) {
        Marker dolgu = yeni(Marker::CUBE);
        dolgu.pose.position = n.merkez;
        dolgu.pose.orientation.z = std::sin(n.yon / 2.0);
        dolgu.pose.orientation.w = std::cos(n.yon / 2.0);
        dolgu.scale = n.boyut;
        dolgu.color = etiketRengi(n.etiket, 0.25f);
        dizi.markers.push_back(dolgu);
      }

      // 3) Yazı: "#12 YAYA? 3.4 m" (+ hareketliyse hız). Hiç hareket etmemiş nesnelerin
      // Kalman hızı sadece gürültüdür, yazılmaz.
      const double hiz = n.hareketli ? std::hypot(n.hiz.x, n.hiz.y) : 0.0;
      Marker yazi = yeni(Marker::TEXT_VIEW_FACING);
      yazi.pose.position = nokta(n.merkez.x, n.merkez.y, ust + 0.3);
      yazi.scale.z = 0.25;
      yazi.color = renk_;
      char tampon[96];
      if (hiz > HAREKET_ESIGI) {
        std::snprintf(tampon, sizeof(tampon), "#%u %s %.1f m\n%.1f m/s",
          n.id, etiketYazisi(n.etiket), n.mesafe, hiz);
      } else {
        std::snprintf(tampon, sizeof(tampon), "#%u %s %.1f m",
          n.id, etiketYazisi(n.etiket), n.mesafe);
      }
      yazi.text = tampon;
      dizi.markers.push_back(yazi);

      // 4) Hareketliyse hız oku (1 saniyede gideceği yer)
      if (hiz > HAREKET_ESIGI) {
        Marker ok = yeni(Marker::ARROW);
        ok.scale.x = 0.05;
        ok.scale.y = 0.12;
        ok.scale.z = 0.15;
        ok.color = renk_;
        ok.points.push_back(nokta(n.merkez.x, n.merkez.y, n.merkez.z));
        ok.points.push_back(nokta(n.merkez.x + n.hiz.x, n.merkez.y + n.hiz.y, n.merkez.z));
        dizi.markers.push_back(ok);
      }
    }
    return dizi;
  }

  // ---------------------------------------------------------------------------
  // Üye değişkenler
  // ---------------------------------------------------------------------------
  Ayarlar ayarlar_;
  nesne::ZeminAyirici zemin_;
  nesne::Takipci takipci_;
  // Zemin uyum durumu (zeminiKontrolEt)
  static constexpr int UYUM_TUR = 10;  // 10 Hz'de 1 sn
  bool zemin_uyumlu_ = false;
  bool zemin_uyum_ilk_ = true;
  int zemin_uyum_sayac_ = 0;

  // Kapanışta ekranı temizlemek için (ekraniTemizle)
  std::mutex kapanis_mutex_;
  bool kapaniyor_ = false;
  std_msgs::msg::Header son_header_;  // son gelen bulutun frame'i ve zamanı

  std::string yelpaze_node_;
  Yelpaze yelpaze_;
  bool yelpaze_istek_bekliyor_ = false;
  rclcpp::AsyncParametersClient::SharedPtr param_istemci_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_olay_sub_;
  rclcpp::TimerBase::SharedPtr yelpaze_zamanlayici_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr nokta_sub_;
  rclcpp::Publisher<robotaksi_msgs::msg::NesneDizisi>::SharedPtr liste_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr gorsel_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr kume_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr zemin_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr zemin_disi_pub_;
  rclcpp::Publisher<robotaksi_msgs::msg::ZeminDurumu>::SharedPtr zemin_durumu_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NesneNode>();
  // Ctrl+C / SIGTERM gelince rclcpp bunu ROS kapanmadan HEMEN ÖNCE çağırır (o an yayın
  // hâlâ mümkün): RViz'de son kutular/bulutlar asılı kalmasın.
  std::weak_ptr<NesneNode> zayif = node;
  rclcpp::contexts::get_global_default_context()->add_pre_shutdown_callback(
    [zayif]() {
      if (auto n = zayif.lock()) {n->ekraniTemizle();}
    });
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
