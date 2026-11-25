// ============================================================================
//  serial_comm_node - TEK YAZICI (2026-08-15 yeni mimari, 2026-11-25 robotaksi_ws)
//  --------------------------------------------------------------------------
//  Teensy'ye yazan TEK yer burasidir. Gamepad seri porta kendi yazmaz;
//  gamepad_node.py joystick'i okuyup /gamepad/state yayinlar, burasi ONU ve
//  surus yonteminin komutunu (/arac/komut) birlestirip Teensy'ye TEK bir
//  paket akisi gonderir. Boylece "iki program ayni porta yaziyor" sinifindaki
//  hatalar imkansiz olur.
//
//  robotaksi_ws'e tasinirken degisenler (2026-11-25):
//    - Otonom girdi artik TEK topic: /arac/komut (robotaksi_msgs/AracKomutu).
//      Eski /steering_angle_topic + speed_on/brake_on/reverse_on/gear_toggle
//      servisleri kaldirildi ("son cagiran kazanir" yarisinin kaynagiydi).
//    - KOMUT ZAMAN ASIMI: otonomda son komut komut_zaman_asimi (0.3 sn) icinde
//      gelmediyse gaz kesilir + FREN. Eskiden acilista gaz ACIKTI ve surus
//      node'u coksa bile arac son komutla gitmeye devam ediyordu.
//    - MANUELDE kumanda koparsa artik otonoma GECILMEZ, fren basilir. "Kumanda
//      koparsa otonom devam" sadece kumanda OTONOM'dayken koparsa gecerli.
//    - Port acilamazsa node kapanmaz; kimlikten (by-id) aramaya devam eder.
//    - kuru_calisma: Teensy olmadan masada deneme (porta yazmaz, sadece yayinlar).
//    - Teensy'ye giden her paket /arac/gonderilen'e, mod /arac/surus_modu'na yazilir.
//
//  HIYERARSI (kumanda en oncelikli):
//    MANUEL  (kumanda taze, otonom butonu KAPALI) -> her sey kumandadan
//    OTONOM  (kumanda taze, otonom butonu ACIK)   -> direksiyon/gaz/fren/vites
//                                                    /arac/komut'tan; kumandanin
//                                                    sadece MOD TUSU (LB) dinlenir.
//    KUMANDA HIC BAGLANMADIYSA                    -> OTONOM (arac sadece ROS ile)
//    KUMANDA OTONOMDAYKEN KOPARSA                 -> OTONOM DEVAM (2026-08-24)
//                                                    (otonomda_kumanda_zorunlu=true ise fren)
//    KUMANDA MANUELDEYKEN KOPARSA                 -> FREN
//    OTONOMDA KOMUT YOK / BAYAT                   -> FREN
//
//  PAKET (5 byte, firmware ile birebir):
//      0xAA | steer(25-165) | flags | checksum | 0x55
//      flags: bit0=gaz bit1=fren bit2=geri bit3=(TEST modu - HEP 0) bit4=ACIL
//      checksum = (steer ^ flags) & 0xFF
//  Firmware 300 ms paket alamazsa kendisi fren basar.
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <robotaksi_msgs/msg/arac_komutu.hpp>

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cmath>
#include <string>
#include <mutex>
#include <thread>
#include <chrono>
#include <dirent.h>
#include <limits.h>
#include <algorithm>

using robotaksi_msgs::msg::AracKomutu;

int serial_fd = -1;
std::mutex serial_mutex;

// ---------------- SERIAL ----------------
bool openSerial(const std::string &port)
{
    serial_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0) return false;

    struct termios tty{};
    if (tcgetattr(serial_fd, &tty) != 0) {
        close(serial_fd);
        serial_fd = -1;
        return false;
    }

    // Firmware Serial.begin(115200). Teensy USB-CDC'de baud aslinda yok
    // sayilir ama dogru degeri yazmak zararsiz ve niyeti belli eder.
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5;

    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        close(serial_fd);
        serial_fd = -1;
        return false;
    }
    return true;
}

// ---------------- TEENSY PORTUNU KIMLIKTEN BUL ----------------
// USB fisi cikip girince ttyACM NUMARASI DEGISIR (ACM0 -> ACM2). Ama
// /dev/serial/by-id altindaki isim seri numarayla sabittir:
//     usb-Teensyduino_USB_Serial_15609870-if00
// Bu yuzden numaraya degil KIMLIGE bakiyoruz. Baska cihaza (CubeOrange /
// Arduino encoder) yanlislikla yazmamak icin KOR TARAMA YAPILMAZ.
std::string teensyPortBul(const std::string &varsayilan)
{
    const char *dizin = "/dev/serial/by-id";
    DIR *d = opendir(dizin);
    if (d) {
        struct dirent *e;
        while ((e = readdir(d)) != nullptr) {
            std::string ad = e->d_name;
            if (ad == "." || ad == "..") continue;
            std::string kucuk = ad;
            std::transform(kucuk.begin(), kucuk.end(), kucuk.begin(), ::tolower);
            if (kucuk.find("teensy") == std::string::npos) continue;
            std::string tam = std::string(dizin) + "/" + ad;
            char hedef[PATH_MAX];
            ssize_t n = readlink(tam.c_str(), hedef, sizeof(hedef) - 1);
            if (n > 0) {
                hedef[n] = '\0';
                std::string yol(hedef);
                size_t k = yol.find_last_of('/');
                if (k != std::string::npos) yol = yol.substr(k + 1);
                closedir(d);
                return "/dev/" + yol;
            }
        }
        closedir(d);
    }
    // Kimlikten bulunamadi: eski yol hala duruyorsa onu dene. Baska bir
    // ACM'yi KOR KOR denemiyoruz - o port mavros/encoder olabilir.
    if (access(varsayilan.c_str(), F_OK) == 0) return varsayilan;
    return "";
}

// Portu kilit altinda kapatip yeniden acar (sendPacket ile yarismasin).
bool seriYenidenAc(const std::string &yol)
{
    std::lock_guard<std::mutex> lock(serial_mutex);
    if (serial_fd >= 0) { close(serial_fd); serial_fd = -1; }
    return openSerial(yol);
}

// 5 byte'i TEK write() ile yolla: paket bolunmesin (cerceve bozulmasin).
bool sendPacket(uint8_t steer, uint8_t flags)
{
    uint8_t pkt[5];
    pkt[0] = 0xAA;
    pkt[1] = steer;
    pkt[2] = flags;
    pkt[3] = (uint8_t)((steer ^ flags) & 0xFF);
    pkt[4] = 0x55;
    std::lock_guard<std::mutex> lock(serial_mutex);
    if (serial_fd < 0) return false;
    return write(serial_fd, pkt, 5) == 5;
}

// ---------------- QUANTIZE ----------------
// Otonom direksiyon degeri 5'lik bloklara yuvarlanir (sahada boyle ayarlandi,
// firmware'in +-3 toleransiyla uyumlu). Manuel/kumanda degeri HAM gecer.
// Ornek: 84 -> 82, 85..89 -> 87.
uint16_t quantizeSteering(int data)
{
    if (data <= 25)  return 25;
    if (data >= 165) return 165;
    int block = data / 5;
    return static_cast<uint16_t>(block * 5 + 2);
}

// Teensy'ye giden modlar (/arac/surus_modu)
enum Mod : int {
    MOD_PAKET_YOK = 0,   // hic paket gonderilmiyor (Teensy 300 ms sonra kendi frenler)
    MOD_MANUEL    = 1,   // kumanda suruyor
    MOD_OTONOM    = 2,   // /arac/komut suruyor
    MOD_FREN      = 3,   // guvenlik freni: otonomda komut yok ya da manuelde kumanda koptu
};

// ---------------- NODE ----------------
class SerialCommNode : public rclcpp::Node
{
public:
    SerialCommNode() : Node("serial_comm_node")
    {
        // Varsayilan yol; once /dev/serial/by-id altinda "Teensy" aranir.
        port_ = this->declare_parameter<std::string>("port", "/dev/ttyACM0");
        // true: kumanda taze degilse HIC paket gonderme (olu-adam, Teensy frenler).
        // false (varsayilan, 2026-08-24): kumandasiz da paket akar.
        require_gamepad_ = this->declare_parameter<bool>("require_gamepad", false);
        // OTONOMDAYKEN kumanda koparsa ne olsun (2026-08-23 kullanici karari):
        // false = surus DEVAM eder (yaris turu kumanda parazitinden olmesin)
        // true  = fren
        // MANUELDEYKEN kumanda koparsa HER ZAMAN fren.
        otonomda_kumanda_zorunlu_ =
            this->declare_parameter<bool>("otonomda_kumanda_zorunlu", false);
        // Surus yonteminin komutu bu sureden eskiyse otonomda FREN (saniye)
        komut_zaman_asimi_ = this->declare_parameter<double>("komut_zaman_asimi", 0.3);
        // true: Teensy'ye YAZMA, sadece ne gonderecegini yayinla (masada deneme)
        kuru_ = this->declare_parameter<bool>("kuru_calisma", false);

        rclcpp::QoS qos_latched(1);
        qos_latched.transient_local();
        baglanti_pub_ = this->create_publisher<std_msgs::msg::Bool>("/arac/baglanti", qos_latched);
        mode_pub_ = this->create_publisher<std_msgs::msg::Int32>("/arac/surus_modu", 10);
        gonderilen_pub_ = this->create_publisher<AracKomutu>("/arac/gonderilen", 10);

        if (kuru_) {
            RCLCPP_WARN(this->get_logger(),
                "KURU CALISMA: Teensy'ye HICBIR SEY YAZILMIYOR. Gidecek paketler "
                "/arac/gonderilen'de.");
            baglantiYayinla(false);
        } else {
            std::string yol = teensyPortBul(port_);
            if (!yol.empty() && seriYenidenAc(yol)) {
                port_ = yol;
                RCLCPP_INFO(this->get_logger(),
                    "Teensy baglandi: %s (115200, 5-byte paket)", port_.c_str());
                baglantiYayinla(true);
                std::this_thread::sleep_for(std::chrono::milliseconds(800));
            } else {
                // Kapanmiyoruz: yazma hatasi sayaci yeniden baglanmayi dener.
                RCLCPP_ERROR(this->get_logger(),
                    "Seri port acilamadi (%s). Teensy takili mi, kullanici 'dialout' "
                    "grubunda mi? Aramaya devam ediliyor...",
                    yol.empty() ? port_.c_str() : yol.c_str());
                baglantiYayinla(false);
            }
        }

        // ---- KUMANDA (tek gercek hakem) ----
        gamepad_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/gamepad/state", 10,
            std::bind(&SerialCommNode::gamepadCallback, this, std::placeholders::_1));

        // ---- OTONOM: surus yonteminin komutu ----
        komut_sub_ = this->create_subscription<AracKomutu>(
            "/arac/komut", 10,
            [this](const AracKomutu::SharedPtr m) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                komut_ = *m;
                komut_last_ = this->now();
                komut_alindi_ = true;
            });

        // 50 Hz paket akisi (firmware timeout'u 300ms; bol pay var)
        tx_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&SerialCommNode::txTimer, this));

        RCLCPP_INFO(this->get_logger(),
            "Serial Comm hazir - TEK YAZICI. Komut: /arac/komut (zaman asimi %.2f sn). "
            "Kumanda zorunlu: %s",
            komut_zaman_asimi_, require_gamepad_ ? "EVET" : "hayir");
    }

    ~SerialCommNode()
    {
        // Kapanirken son soz FREN olsun (Teensy 300 ms'de zaten frenler, bu daha hizli).
        if (!kuru_) {
            for (int i = 0; i < 3; ++i) {
                sendPacket(son_steer_, 0x02);
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        }
        std::lock_guard<std::mutex> lock(serial_mutex);
        if (serial_fd >= 0) { close(serial_fd); serial_fd = -1; }
    }

private:
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr gamepad_sub_;
    rclcpp::Subscription<AracKomutu>::SharedPtr komut_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr  baglanti_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mode_pub_;
    rclcpp::Publisher<AracKomutu>::SharedPtr gonderilen_pub_;
    rclcpp::TimerBase::SharedPtr tx_timer_;

    std::mutex data_mutex_;

    // --- kumandadan gelen ---
    int  gp_steer_ = 95;
    bool gp_gaz_ = false, gp_fren_ = false, gp_geri_ = false, gp_otonom_ = false;
    bool gp_acil_ = false;   // ACIL DURUM butonu (B) - ham durum, toggle'i Teensy yapar
    rclcpp::Time gp_last_;
    bool gp_alindi_ = false;

    // --- otonom (surus yontemi) tarafi ---
    AracKomutu komut_;
    rclcpp::Time komut_last_;
    bool komut_alindi_ = false;

    bool require_gamepad_ = false;
    bool otonomda_kumanda_zorunlu_ = false;
    double komut_zaman_asimi_ = 0.3;
    bool kuru_ = false;
    uint8_t son_steer_ = 84;      // fren paketlerinde direksiyon yerinde kalsin
    unsigned yazma_hatasi_ = 0;   // toplam basarisiz write()
    unsigned ardisik_hata_ = 0;   // ust uste basarisiz write()
    std::string port_;            // acik olan port yolu
    rclcpp::Time son_deneme_;     // son yeniden baglanma denemesi
    bool         deneme_var_ = false;
    int  son_loglanan_mod_ = -1;
    std::string son_sebep_;

    void gamepadCallback(const std_msgs::msg::Int32MultiArray::SharedPtr m)
    {
        if (m->data.size() < 5) return;
        std::lock_guard<std::mutex> lock(data_mutex_);
        gp_steer_  = m->data[0];
        gp_gaz_    = m->data[1] != 0;
        gp_fren_   = m->data[2] != 0;
        gp_geri_   = m->data[3] != 0;
        gp_otonom_ = m->data[4] != 0;
        // 6. eleman ACIL (yeni). Eski 5 elemanli yayin gelirse acil=false.
        gp_acil_   = (m->data.size() >= 6) && (m->data[5] != 0);
        gp_last_   = this->now();
        gp_alindi_ = true;
    }

    void baglantiYayinla(bool acik)
    {
        std_msgs::msg::Bool b; b.data = acik;
        baglanti_pub_->publish(b);
    }

    // Portu kimlikten bulup yeniden acar. Bulamazsa sessizce gecer,
    // bir sonraki denemede tekrar arar (cihaz henuz takilmamis olabilir).
    void yenidenBagla()
    {
        std::string yeni = teensyPortBul(port_);
        if (yeni.empty()) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Teensy bulunamadi (/dev/serial/by-id icinde 'Teensy' yok). "
                "Kablo takili mi?");
            return;
        }
        if (seriYenidenAc(yeni)) {
            RCLCPP_WARN(this->get_logger(),
                "SERI PORT ACILDI: %s  (onceki: %s)",
                yeni.c_str(), port_.c_str());
            port_ = yeni;
            ardisik_hata_ = 0;
            baglantiYayinla(true);
        } else {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Port acilamadi: %s (izin yoksa: sudo usermod -aG dialout $USER, "
                "sonra oturumu kapat-ac)", yeni.c_str());
        }
    }

    void txTimer()
    {
        uint8_t steer = son_steer_, flags = 0;
        int mod;
        std::string sebep;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            const rclcpp::Time simdi = this->now();

            bool gp_taze = gp_alindi_ && (simdi - gp_last_).seconds() < 0.5;
            bool komut_taze = komut_alindi_ &&
                              (simdi - komut_last_).seconds() < komut_zaman_asimi_;

            // ---- 1) KIM SURUYOR? ----
            if (gp_taze) {
                mod = gp_otonom_ ? MOD_OTONOM : MOD_MANUEL;
            } else if (require_gamepad_) {
                // OLU-ADAM: kumanda yoksa PAKET GONDERME. Teensy 300ms sonra
                // kendi kendine fren basar.
                mod = MOD_PAKET_YOK;
            } else if (!gp_alindi_) {
                mod = MOD_OTONOM;   // kumanda hic baglanmadi: arac sadece ROS ile
            } else if (gp_otonom_ && !otonomda_kumanda_zorunlu_) {
                mod = MOD_OTONOM;   // otonomdayken koptu: yaris turu olmesin
            } else {
                mod = MOD_FREN;
                sebep = gp_otonom_ ? "kumanda koptu (otonomda_kumanda_zorunlu)"
                                   : "kumanda koptu (manuel modda)";
            }
            // Bayat kumanda degerleri hicbir zaman kullanilmaz (kacak gaz riski yok).
            if (!gp_taze) gp_acil_ = false;   // bayat acil biti kilitli kalmasin

            // ---- 2) OTONOMDA KOMUT TAZE MI? ----
            if (mod == MOD_OTONOM && !komut_taze) {
                mod = MOD_FREN;
                sebep = komut_alindi_ ? "komut bayat (/arac/komut kesildi)"
                                      : "komut yok (/arac/komut hic gelmedi)";
            }

            // ---- 3) PAKETI KUR ----
            if (mod == MOD_OTONOM) {
                // Kumandanin direksiyon/gaz/fren/vitesi YOK SAYILIR.
                steer = (uint8_t)quantizeSteering(komut_.direksiyon);
                bool fren = komut_.fren;
                bool gaz  = komut_.gaz && !fren;   // fren > gaz
                flags = (uint8_t)((gaz ? 1 : 0) | (fren ? 2 : 0) | (komut_.geri ? 4 : 0));
            } else if (mod == MOD_MANUEL) {
                // her sey kumandadan, direksiyon HAM
                int s = std::clamp(gp_steer_, 25, 165);
                steer = (uint8_t)s;
                flags = (uint8_t)((gp_gaz_  ? 1 : 0) |
                                  (gp_fren_ ? 2 : 0) |
                                  (gp_geri_ ? 4 : 0));
            } else if (mod == MOD_FREN) {
                // gaz yok + fren; direksiyon son yerinde (ani direksiyon yok)
                flags = 0x02;
            }
            // bit3 (firmware TEST modu) ASLA set edilmez.
            // ACIL DURUM (bit4) KALDIRILDI (2026-08-24). gamepad_node zaten
            // hep 0 yolluyor; burada da KOSULSUZ set edilmiyor ki bozuk/eski
            // bir yayin bile firmware'in acilDurumModu'nu tetikleyemesin.
            //   Geri acmak: asagidaki satiri yorumdan cikar.
            // if (gp_acil_) flags |= 0x10;
            (void)gp_acil_;
        }

        modDegistiyse(mod, sebep);
        publishMode(mod);
        if (mod == MOD_PAKET_YOK) return;

        son_steer_ = steer;
        gonderileniYayinla(mod, sebep, steer, flags);

        // KURU: porta hic dokunma
        if (kuru_) {
            durumSatiri(mod, steer, flags, "KURU");
            return;
        }

        // ---- TESHIS: her paketin sonucunu izle ----
        // USB dususe write() bassiz olur; basarisizlik ANINDA terminale duser.
        bool yazildi = sendPacket(steer, flags);

        if (!yazildi) {
            yazma_hatasi_++;
            ardisik_hata_++;
            if (ardisik_hata_ == 1) baglantiYayinla(false);
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "!!! SERI YAZMA HATASI (toplam %u) - Teensy'ye paket GITMIYOR. "
                "Yeniden baglanmaya calisiliyor...", yazma_hatasi_);

            // 5 ardisik hata (~100 ms) sonra portu yeniden ac. Fis cikip
            // girince ya da hub resetleyince ttyACM NUMARASI DEGISIR; kimlikten
            // (Teensyduino seri no) yeni yolu buluruz. Denemeler 500 ms'de bir.
            if (ardisik_hata_ >= 5) {
                auto simdi = this->now();
                if (!deneme_var_ || (simdi - son_deneme_).seconds() > 0.5) {
                    son_deneme_ = simdi;
                    deneme_var_ = true;
                    yenidenBagla();
                }
            }
        } else {
            if (ardisik_hata_ > 0) {
                RCLCPP_WARN(this->get_logger(),
                    "Seri hat DUZELDI (%u hatadan sonra) -> %s",
                    ardisik_hata_, port_.c_str());
                baglantiYayinla(true);
            }
            ardisik_hata_ = 0;
        }

        durumSatiri(mod, steer, flags, yazildi ? "OK" : "HATA");
    }

    static const char *modAdi(int mod)
    {
        switch (mod) {
            case MOD_MANUEL: return "MANUEL";
            case MOD_OTONOM: return "OTONOM";
            case MOD_FREN:   return "FREN";
            default:         return "PAKET_YOK";
        }
    }

    void modDegistiyse(int mod, const std::string &sebep)
    {
        if (son_loglanan_mod_ == mod && son_sebep_ == sebep) return;
        son_loglanan_mod_ = mod;
        son_sebep_ = sebep;
        switch (mod) {
            case MOD_MANUEL:
                RCLCPP_WARN(this->get_logger(), "SURUS MODU -> MANUEL (kumanda suruyor)");
                break;
            case MOD_OTONOM:
                RCLCPP_WARN(this->get_logger(), "SURUS MODU -> OTONOM (/arac/komut suruyor)");
                break;
            case MOD_FREN:
                RCLCPP_WARN(this->get_logger(), "SURUS MODU -> GUVENLIK FRENI: %s", sebep.c_str());
                break;
            default:
                RCLCPP_WARN(this->get_logger(),
                    "KUMANDA PAKETI YOK -> gonderim durduruldu (Teensy fren basacak)");
        }
    }

    // 500 ms'de bir durum satiri. Eskiden her byte basiliyordu (60 satir/sn)
    // ve terminal cizimi gonderimi geciktiriyordu - bu yuzden kisitli.
    void durumSatiri(int mod, uint8_t steer, uint8_t flags, const char *yazma)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "[TX] %-6s gaz=%d fren=%d geri=%d steer=%3d | yazma=%s",
            modAdi(mod),
            (flags & 1) ? 1 : 0, (flags & 2) ? 1 : 0, (flags & 4) ? 1 : 0,
            (int)steer, yazma);
    }

    void gonderileniYayinla(int mod, const std::string &sebep, uint8_t steer, uint8_t flags)
    {
        AracKomutu g;
        g.header.stamp = this->now();
        g.kaynak = mod == MOD_MANUEL ? "manuel"
                 : mod == MOD_OTONOM ? "otonom"
                 : "fren:" + sebep;
        g.direksiyon = steer;
        g.gaz  = flags & 1;
        g.fren = flags & 2;
        g.geri = flags & 4;
        gonderilen_pub_->publish(g);
    }

    void publishMode(int m)
    {
        std_msgs::msg::Int32 msg; msg.data = m;
        mode_pub_->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialCommNode>();
    rclcpp::spin(node);
    node.reset();   // yikici fren paketlerini gondersin
    rclcpp::shutdown();
    return 0;
}
