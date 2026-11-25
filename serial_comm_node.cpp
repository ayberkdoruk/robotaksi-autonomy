// ============================================================================
//  serial_comm_node - TEK YAZICI (2026-08-15 yeni mimari)
//  --------------------------------------------------------------------------
//  Teensy'ye yazan TEK yer burasidir. Gamepad artik seri porta kendi yazmaz;
//  gamepad_node.py joystick'i okuyup /gamepad/state yayinlar, burasi ONU ve
//  otonom komutlari birlestirip Teensy'ye TEK bir paket akisi gonderir.
//  Boylece "iki program ayni porta yaziyor" sinifindaki hatalar imkansiz olur.
//
//  HIYERARSI (kumanda en oncelikli):
//    MANUEL  (gamepad otonom butonu KAPALI) -> her sey kumandadan
//    OTONOM  (gamepad otonom butonu ACIK)   -> direksiyon/gaz/fren/vites
//                                              ROS'tan; kumandanin sadece
//                                              MOD TUSU (LB) dinlenir.
//    KUMANDA KOPARSA/HIC YOKSA -> OTONOM DEVAM EDER (2026-08-24). Paket
//    akisi kesilmez, arac durmaz. Kumanda sadece TAZE iken soz sahibidir.
//
//  PAKET (5 byte, firmware ile birebir):
//      0xAA | steer(25-165) | flags | checksum | 0x55
//      flags: bit0=gaz bit1=fren bit2=geri bit3=(TEST modu - HEP 0) bit4=ACIL
//      checksum = (steer ^ flags) & 0xFF
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

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
uint16_t quantizeSteering(int data)
{
    data = (int)std::ceil((double)data);
    if (data <= 25)  return 25;
    if (data >= 165) return 165;
    int block = data / 5;
    return static_cast<uint16_t>(block * 5 + 2);
}

// ---------------- NODE ----------------
class SerialCommNode : public rclcpp::Node
{
public:
    SerialCommNode() : Node("serial_comm_node")
    {
        this->declare_parameter<std::string>("port", "/dev/ttyACM0");
        // 2026-08-24: VARSAYILAN false. Kumanda olmasa BILE paket akar.
        // true yaparsan eski davranis (kumandasiz gonderim durur).
        this->declare_parameter<bool>("require_gamepad", false);
        // OTONOMA GECILDIKTEN SONRA kumanda kopsa bile FREN YAPMA (2026-08-23).
        // Kullanici karari: yaris turu kumanda parazitinden dolayi olmesin.
        // false = otonomda kumanda kopunca surus DEVAM eder (olu-adam KAPALI)
        // true  = eski davranis (kumanda kopunca her durumda fren)
        // MANUEL modda koruma HER ZAMAN acik kalir - orada kumanda tek kontrol.
        // DIKKAT: otonomda kumanda giderse ACIL DURUM butonu da gider ->
        //         acil fren = kodunyo'dan Teensy'yi Durdur (paket kesilir,
        //         Teensy 300ms timeout ile frenler).
        this->declare_parameter<bool>("otonomda_kumanda_zorunlu", false);
        otonomda_kumanda_zorunlu_ = this->get_parameter("otonomda_kumanda_zorunlu").as_bool();
        port_ = this->get_parameter("port").as_string();
        const std::string &port = port_;
        require_gamepad_ = this->get_parameter("require_gamepad").as_bool();

        if (!openSerial(port)) {
            RCLCPP_FATAL(this->get_logger(), "Seri port acilamadi: %s", port.c_str());
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Teensy baglandi: %s (115200, 5-byte paket)", port.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(800));

        // ---- KUMANDA (tek gercek hakem) ----
        gamepad_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/gamepad/state", 10,
            std::bind(&SerialCommNode::gamepadCallback, this, std::placeholders::_1));

        // ---- OTONOM direksiyon (mux cikisi) ----
        steering_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/steering_angle_topic", 10,
            [this](const std_msgs::msg::Int32::SharedPtr m) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                oto_steer_ = m->data;
            });

        // ---- OTONOM gaz/fren/vites servisleri (pure_pursuit kullanir) ----
        speed_srv_ = this->create_service<std_srvs::srv::Trigger>("speed_on",
            std::bind(&SerialCommNode::speedService, this,
                      std::placeholders::_1, std::placeholders::_2));
        brake_srv_ = this->create_service<std_srvs::srv::Trigger>("brake_on",
            std::bind(&SerialCommNode::brakeService, this,
                      std::placeholders::_1, std::placeholders::_2));
        reverse_srv_ = this->create_service<std_srvs::srv::Trigger>("reverse_on",
            std::bind(&SerialCommNode::reverseService, this,
                      std::placeholders::_1, std::placeholders::_2));
        gear_srv_ = this->create_service<std_srvs::srv::Trigger>("gear_toggle",
            std::bind(&SerialCommNode::gearToggleService, this,
                      std::placeholders::_1, std::placeholders::_2));

        // 50 Hz paket akisi (firmware timeout'u 300ms; bol pay var)
        tx_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&SerialCommNode::txTimer, this));

        rclcpp::QoS qos_latched(1);
        qos_latched.transient_local();
        ready_pub_ = this->create_publisher<std_msgs::msg::Bool>("/serial_comm/ready", qos_latched);
        std_msgs::msg::Bool ready; ready.data = true;
        ready_pub_->publish(ready);

        // Arayuz icin: su an hangi kaynak araci suruyor (0=paket yok, 1=manuel, 2=otonom)
        mode_pub_ = this->create_publisher<std_msgs::msg::Int32>("/surus_modu", 10);

        RCLCPP_INFO(this->get_logger(),
            "Serial Comm hazir - TEK YAZICI. Kumanda zorunlu: %s",
            require_gamepad_ ? "EVET" : "hayir");
    }

    ~SerialCommNode()
    {
        if (serial_fd >= 0) { close(serial_fd); serial_fd = -1; }
    }

private:
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr gamepad_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr steering_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr speed_srv_, brake_srv_,
                                                       reverse_srv_, gear_srv_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr  ready_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mode_pub_;
    rclcpp::TimerBase::SharedPtr tx_timer_;

    std::mutex data_mutex_;

    // --- kumandadan gelen ---
    int  gp_steer_ = 95;
    bool gp_gaz_ = false, gp_fren_ = false, gp_geri_ = false, gp_otonom_ = false;
    bool gp_acil_ = false;   // ACIL DURUM butonu (B) - ham durum, toggle'i Teensy yapar
    rclcpp::Time gp_last_;
    bool gp_alindi_ = false;

    // --- otonom (ROS) tarafi ---
    int  oto_steer_ = 84;   // fiziksel DUZ (pure_pursuit STEER_CENTER ile ayni)
    // GAZ VARSAYILAN ACIK: eski mimaride serial_comm 201 (gaz) ile basliyordu;
    // SERIT TAKIBI fazinda gazi veren buydu (serit node'u sadece direksiyon
    // verir, speed_on cagirmaz). Varsayilan kapali olsaydi serit fazinda arac
    // hic hareket etmezdi. brake_on cagrilinca kapanir, speed_on ile geri acilir.
    // NOT: bu yuzden OTONOMA GECER GECMEZ arac gaz alir - LB'ye basmadan once
    // aracin onunun bos oldugundan emin ol.
    bool oto_gaz_ = true, oto_fren_ = false, oto_geri_ = false;

    bool require_gamepad_ = true;
    bool otonomda_kumanda_zorunlu_ = false;
    unsigned yazma_hatasi_ = 0;   // toplam basarisiz write()
    unsigned ardisik_hata_ = 0;   // ust uste basarisiz write()
    std::string port_;            // acik olan port yolu
    rclcpp::Time son_deneme_;     // son yeniden baglanma denemesi
    bool         deneme_var_ = false;
    int  son_loglanan_mod_ = -1;

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
                "SERI PORT YENIDEN ACILDI: %s  (eski: %s)",
                yeni.c_str(), port_.c_str());
            port_ = yeni;
            ardisik_hata_ = 0;
        } else {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Port acilamadi: %s", yeni.c_str());
        }
    }

    void txTimer()
    {
        uint8_t steer, flags;
        int mod;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);

            bool gp_taze = gp_alindi_ &&
                           (this->now() - gp_last_).seconds() < 0.5;

            // Taze kumanda varken otonoma gecildiyse KILITLE. Kumanda sonradan
            // koparsa bu bayrak sayesinde otonom surus kesilmez.
            // KUMANDA UMURSANMAZ (2026-08-24 kullanici karari):
            //   taze paket VARSA  -> kumanda karar verir (manuel / otonom)
            //   taze paket YOKSA  -> OTONOM devam. Kumanda node'u hic
            //                        acilmamis olsa bile arac ROS ile gider.
            // Boylece "kumanda node'u kapali" ya da "kumanda koptu" durumu
            // otonom surusu HIC etkilemez. Bayat kumanda degerleri de
            // kullanilmaz (kacak gaz riski yok) - oto_ degerleri gecerlidir.
            bool oto_devam = !gp_taze && !otonomda_kumanda_zorunlu_;
            if (oto_devam) {
                gp_acil_ = false;   // bayat acil biti kilitli kalmasin
                if (son_loglanan_mod_ != 3) {
                    son_loglanan_mod_ = 3;
                    RCLCPP_WARN(this->get_logger(),
                        "KUMANDA YOK -> OTONOM DEVAM EDIYOR (fren yok). "
                        "Acil durdurma: kumandada B ya da gucu kes.");
                }
            }

            if (require_gamepad_ && !gp_taze && !oto_devam) {
                // OLU-ADAM: kumanda yoksa PAKET GONDERME. Teensy 300ms sonra
                // kendi kendine fren basar. ROS/PC/kumanda coktugunde arac
                // kacmaz - eski RF alicinin sagladigi guvenligin karsiligi.
                mod = 0;
                if (son_loglanan_mod_ != mod) {
                    son_loglanan_mod_ = mod;
                    RCLCPP_WARN(this->get_logger(),
                        "KUMANDA PAKETI YOK -> gonderim durduruldu (Teensy fren basacak)");
                }
                publishMode(0);
                return;
            }

            if (gp_otonom_ || oto_devam) {
                // ---- OTONOM ----
                // Kumandanin direksiyon/gaz/fren/vitesi YOK SAYILIR.
                // Kumandanin tek etkisi MOD TUSU'dur (gp_otonom_); kapatildigi
                // an bir sonraki pakette (20ms) manuele donulur.
                steer = (uint8_t)quantizeSteering(oto_steer_);
                flags = (uint8_t)((oto_gaz_  ? 1 : 0) |
                                  (oto_fren_ ? 2 : 0) |
                                  (oto_geri_ ? 4 : 0));
                mod = 2;
            } else {
                // ---- MANUEL ---- her sey kumandadan
                int s = gp_steer_;
                if (s < 25) s = 25;
                if (s > 165) s = 165;
                steer = (uint8_t)s;
                flags = (uint8_t)((gp_gaz_  ? 1 : 0) |
                                  (gp_fren_ ? 2 : 0) |
                                  (gp_geri_ ? 4 : 0));
                mod = 1;
            }
            // bit3 (firmware TEST modu) ASLA set edilmez.
            // bit4 ACIL DURUM: OTONOMDA DA gecerli. Otonomda kumandanin
            // direksiyon/gaz/freni yok sayilir AMA acil durdurma her zaman
            // gecmeli - yoksa otonomdayken araci durduramazdik. Firmware
            // yukselen kenarda toggle eder: gaz kes + fren + dortlu flasor.
            // ACIL DURUM (bit4) KALDIRILDI (2026-08-24). gamepad_node zaten
            // hep 0 yolluyor; burada da KOSULSUZ set edilmiyor ki bozuk/eski
            // bir yayin bile firmware'in acilDurumModu'nu tetikleyemesin.
            //   Geri acmak: asagidaki satiri yorumdan cikar.
            // if (gp_acil_) flags |= 0x10;
            (void)gp_acil_;
        }

        // ---- TESHIS: her paketin sonucunu izle ----
        // sendPacket'in donusu ESKIDEN ATILIYORDU: USB dususe write() bassiz
        // olur, node hicbir sey demez, Teensy 300ms'de FREN BASIP KILITLENIR.
        // Artik basarisizlik ANINDA terminale dusuyor.
        bool yazildi = sendPacket(steer, flags);
        publishMode(mod);

        if (!yazildi) {
            yazma_hatasi_++;
            ardisik_hata_++;
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
            }
            ardisik_hata_ = 0;
        }

        // 500 ms'de bir durum satiri. Eskiden her byte basiliyordu (60 satir/sn)
        // ve terminal cizimi gonderimi geciktiriyordu - bu yuzden kisitli.
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "[TX] %s gaz=%d fren=%d geri=%d steer=%3d | yazma=%s%s",
            mod == 2 ? "OTONOM" : "MANUEL",
            (flags & 1) ? 1 : 0, (flags & 2) ? 1 : 0, (flags & 4) ? 1 : 0,
            (int)steer,
            yazildi ? "OK" : "HATA",
            (flags & 0x10) ? "  [ACIL!]" : "");

        if (son_loglanan_mod_ != mod) {
            son_loglanan_mod_ = mod;
            RCLCPP_WARN(this->get_logger(), "SURUS MODU -> %s",
                        mod == 2 ? "OTONOM (ROS suruyor)" : "MANUEL (kumanda suruyor)");
        }
    }

    void publishMode(int m)
    {
        std_msgs::msg::Int32 msg; msg.data = m;
        mode_pub_->publish(msg);
    }

    // ---------------- OTONOM SERVISLERI ----------------
    void speedService(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> r)
    {
        { std::lock_guard<std::mutex> lock(data_mutex_); oto_gaz_ = true; oto_fren_ = false; }
        r->success = true; r->message = "gaz";
    }

    void brakeService(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> r)
    {
        { std::lock_guard<std::mutex> lock(data_mutex_); oto_fren_ = true; oto_gaz_ = false; }
        r->success = true; r->message = "fren";
    }

    // Geri vites: bayrak set edilir, firmware kendi GUVENLI vites sekansini
    // uygular (gaz kes -> bekle -> role -> bekle). Toggle DEGIL, durum.
    void reverseService(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> r)
    {
        { std::lock_guard<std::mutex> lock(data_mutex_); oto_geri_ = true; }
        RCLCPP_WARN(this->get_logger(), "OTONOM VITES -> GERI (firmware sekansi calisacak)");
        r->success = true; r->message = "geri";
    }

    void gearToggleService(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> r)
    {
        bool yeni;
        { std::lock_guard<std::mutex> lock(data_mutex_); oto_geri_ = !oto_geri_; yeni = oto_geri_; }
        RCLCPP_WARN(this->get_logger(), "OTONOM VITES -> %s", yeni ? "GERI" : "ILERI");
        r->success = true; r->message = yeni ? "geri" : "ileri";
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialCommNode>());
    rclcpp::shutdown();
    return 0;
}
