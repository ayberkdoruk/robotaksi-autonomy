"""
Arayüzün ROS bağlantısı.

Tek bir rclpy node'u arka plandaki bir iş parçacığında döner; abonelik geri
çağrıları sadece en son değeri saklar. Qt tarafı bu değerleri kendi zamanlayıcısıyla
okur. Böylece saniyede onlarca mesaj gelse de pencere takılmaz.

Hiçbir şey YAYINLAMAZ: LiDAR'a veya sürücüye ayar göndermez; sadece engel_node /
nesne_node parametrelerini değiştirir ve kayıt oynatıcısını (duraklat, hız, atla) yönetir.
"""

import collections
import struct
import threading
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, String

from .ayar_tanim import ENGEL, NESNE

try:
    from engel_algilama_msgs.msg import NesneDizisi
except ImportError:  # mesaj paketi derlenmemişse nesne tablosu boş kalır
    NesneDizisi = None

OYNATICI = "/rosbag2_player"


class HizSayaci:
    """Bir topic'in saniyedeki mesaj sayısı, son mesajın yaşı ve veri kopmaları."""

    def __init__(self):
        self.zamanlar = collections.deque(maxlen=100)
        self.kopmalar = collections.deque(maxlen=500)
        self.son = 0.0
        self.toplam = 0

    def isaretle(self):
        t = time.monotonic()
        if self.son and t - self.son > 0.35:  # 10 Hz'de 3 tur kaçtı
            self.kopmalar.append(t)
        self.son = t
        self.zamanlar.append(t)
        self.toplam += 1

    def hz(self):
        t = time.monotonic()
        z = [x for x in list(self.zamanlar) if t - x < 2.0]
        if len(z) < 2 or z[-1] <= z[0]:
            return 0.0
        return (len(z) - 1) / (z[-1] - z[0])

    def yas(self):
        """Son mesajdan bu yana geçen süre (s). Hiç gelmediyse sonsuz."""
        return time.monotonic() - self.son if self.son else float("inf")

    def kopma_sayisi(self, sure=60.0):
        t = time.monotonic()
        return sum(1 for x in list(self.kopmalar) if t - x < sure)


def bulut_boyutu(veri):
    """
    Ham (serileştirilmiş) PointCloud2 mesajından nokta sayısını okur.
    1.5 MB'lık mesajı Python'da açmamak için sadece başlığın arkasındaki
    height/width alanlarına bakılır (CDR: 4 bayt başlık, stamp, frame_id, height, width).
    """
    try:
        isaret = "<" if veri[1] == 1 else ">"
        uzunluk = struct.unpack_from(isaret + "I", veri, 12)[0]
        konum = 16 + uzunluk
        konum += (-(konum - 4)) % 4
        yukseklik, genislik = struct.unpack_from(isaret + "II", veri, konum)
        return yukseklik * genislik
    except (struct.error, IndexError, TypeError):
        return 0


class RosBaglanti:
    def __init__(self, node_adi="lidar_kontrol", args=None, engel_hedef=None, nesne_hedef=None):
        if not rclpy.ok():
            # Ctrl+C'yi rclpy değil pencere karşılasın: önce arka plandaki programlar düzgün
            # kapatılsın (kayıt dosyası yarım kalmasın), sonra ROS bağlantısı.
            from rclpy.signals import SignalHandlerOptions
            rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        self.node = n = Node(node_adi)
        if engel_hedef is None:
            engel_hedef = n.declare_parameter("hedef_node", "engel_node").value
        if nesne_hedef is None:
            nesne_hedef = n.declare_parameter("nesne_node", "nesne_node").value
        self.hedefler = {ENGEL: engel_hedef, NESNE: nesne_hedef}
        self.param = {ENGEL: AsyncParameterClient(n, engel_hedef),
                      NESNE: AsyncParameterClient(n, nesne_hedef)}

        # En son gelen değerler (geri çağrılar yazar, Qt okur)
        self.karar, self.mesafe = None, -1.0
        self.engel_noktasi, self.nokta_zamani = None, 0.0
        self.nesneler, self.nesne_zamani = [], 0.0
        self.nokta_sayisi = 0
        self.saat_ns, self.saat_zamani = None, 0.0
        self.karar_sayac = HizSayaci()
        self.nesne_sayac = HizSayaci()
        self.lidar_sayac = HizSayaci()

        n.create_subscription(String, "/engel/karar", self._karar, 10)
        n.create_subscription(Float32, "/engel/mesafe", self._mesafe, 10)
        n.create_subscription(PointStamped, "/engel/engel_noktasi", self._nokta, 10)
        if NesneDizisi is not None:
            n.create_subscription(NesneDizisi, "/nesne/liste", self._nesneler, 10)
        # raw=True: mesaj Python nesnesine çevrilmez (1.5 MB x 10 Hz), sadece sayılır
        n.create_subscription(PointCloud2, "/rslidar_points", self._bulut,
                              qos_profile_sensor_data, raw=True)
        n.create_subscription(Clock, "/clock", self._saat, qos_profile_sensor_data)

        self._oynatici_istemcileri = None
        self._calisiyor = True
        self.yurutucu = SingleThreadedExecutor()
        self.yurutucu.add_node(n)
        self._is = threading.Thread(target=self._dongu, name="ros", daemon=True)
        self._is.start()

    # ---------------- arka plan ----------------
    def _dongu(self):
        while self._calisiyor and rclpy.ok():
            try:
                self.yurutucu.spin_once(timeout_sec=0.05)
            except Exception as e:  # kapanırken gelen hatalar
                if self._calisiyor and rclpy.ok():
                    print(f"[lidar_kontrol] ROS döngüsünde hata: {e}", flush=True)
                    time.sleep(0.1)

    def kapat(self):
        self._calisiyor = False
        self._is.join(timeout=1.0)
        try:
            self.yurutucu.remove_node(self.node)
            self.node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()

    # ---------------- geri çağrılar ----------------
    def _karar(self, msg):
        self.karar = msg.data
        self.karar_sayac.isaretle()

    def _mesafe(self, msg):
        self.mesafe = msg.data

    def _nokta(self, msg):
        self.engel_noktasi, self.nokta_zamani = msg.point, time.monotonic()

    def _nesneler(self, msg):
        self.nesneler, self.nesne_zamani = list(msg.nesneler), time.monotonic()
        self.nesne_sayac.isaretle()

    def _bulut(self, veri):
        self.nokta_sayisi = bulut_boyutu(veri)
        self.lidar_sayac.isaretle()

    def _saat(self, msg):
        self.saat_ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        self.saat_zamani = time.monotonic()

    # ---------------- kolay erişim ----------------
    def karar_taze(self, sure=1.5):
        return self.karar is not None and self.karar_sayac.yas() < sure

    def nokta_taze(self, sure=0.5):
        return self.engel_noktasi is not None and time.monotonic() - self.nokta_zamani < sure

    def nesneler_taze(self, sure=1.0):
        return time.monotonic() - self.nesne_zamani < sure

    def node_var_mi(self, ad):
        """Bu adla çalışan bir ROS node'u var mı? (ör. 'engel_node')"""
        try:
            return any(n == ad for n, _ in self.node.get_node_names_and_namespaces())
        except Exception:
            return False

    # ---------------- kayıt oynatıcısı (ros2 bag play) ----------------
    def _oynatici(self):
        if self._oynatici_istemcileri is None:
            from rosbag2_interfaces.srv import Pause, PlayNext, Resume, Seek, SetRate
            n = self.node
            self._oynatici_istemcileri = {
                "duraklat": n.create_client(Pause, OYNATICI + "/pause"),
                "devam": n.create_client(Resume, OYNATICI + "/resume"),
                "hiz": n.create_client(SetRate, OYNATICI + "/set_rate"),
                "sonraki": n.create_client(PlayNext, OYNATICI + "/play_next"),
                "atla": n.create_client(Seek, OYNATICI + "/seek"),
            }
        return self._oynatici_istemcileri

    def oynatici_hazir(self):
        return self._oynatici()["duraklat"].service_is_ready()

    def oynatici(self, komut, deger=None):
        """Oynatıcıya komut gönderir. Hazır değilse False."""
        istemci = self._oynatici()[komut]
        if not istemci.service_is_ready():
            return False
        istek = istemci.srv_type.Request()
        if komut == "hiz":
            istek.rate = float(deger)
        elif komut == "atla":
            istek.time.sec = int(deger // 1_000_000_000)
            istek.time.nanosec = int(deger % 1_000_000_000)
        istemci.call_async(istek)
        return True
