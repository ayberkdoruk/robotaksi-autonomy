"""
Tek başına çalışan engel ayar paneli (kontrol_merkezi lidar_sistem.launch.py açar).

Kontrol Merkezi'ndeki Engel Algılama sayfasının aynı parçası: canlı karar,
yukarıdan bakış çizimi, nesne tablosu ve tüm ayarlar. Program başlatıp kapatmaz;
launch dosyasının açtığı node'lara bağlanır.
"""

import signal
import sys

from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from rclpy.utilities import remove_ros_args

from . import ikonlar, tema
from .algilama import AlgilamaBolumu
from .ayar_tanim import GenelAyarlar
from .ros_baglanti import RosBaglanti


class PanelPenceresi(QMainWindow):
    def __init__(self, ros):
        super().__init__()
        self.ros = ros
        self.genel = GenelAyarlar()
        tema.uygula(QApplication.instance(), self.genel["tema"])
        self.setWindowTitle("Engel Algılama — Ayar Paneli")
        self.setWindowIcon(ikonlar.uygulama_ikonu())
        self.algilama = AlgilamaBolumu(ros, self.genel)
        ic = QWidget()
        ic.setObjectName("sayfa")
        duzen = QVBoxLayout(ic)
        duzen.setContentsMargins(12, 12, 12, 12)
        duzen.addWidget(self.algilama)
        self.setCentralWidget(ic)
        self.resize(1180, 780)
        self.tik = QTimer(self)
        self.tik.timeout.connect(self.algilama.guncelle)
        self.tik.start(100)


def main():
    from .uygulama import hata_yakalayici
    sys.excepthook = hata_yakalayici
    uygulama = QApplication(remove_ros_args(sys.argv))
    uygulama.setApplicationName("Engel Ayar Paneli")
    ros = RosBaglanti("ayar_paneli", args=sys.argv)  # hedef_node / nesne_node parametreleri okunur
    try:
        pencere = PanelPenceresi(ros)
        pencere.show()
        for s in (signal.SIGINT, signal.SIGTERM):
            signal.signal(s, lambda *_: QTimer.singleShot(0, pencere.close))
        nabiz = QTimer()
        nabiz.timeout.connect(lambda: None)
        nabiz.start(250)
        return uygulama.exec_()
    finally:
        ros.kapat()
