"""
Robotaksi Kontrol Merkezi: ana pencere.

Tek pencere, hiç terminal yok. Ana ekrandan bir bölüm seçilince o bölümün gerektirdiği
programlar (LiDAR sürücüsü, engel_node, nesne_node, RViz, kayıt oynatıcı) arkada açılır.
Bölüm değişince sadece fark kapatılır/açılır; örneğin LiDAR'ı İzle → Engel Algılama
geçişinde sürücü açık kalır. Bir programın ayarı değişince (RViz düzeni, oynatıcının
döngü ayarı, nesne algılama aç/kapat) o program kendiliğinden yeniden başlatılır.
"""

import signal
import sys
import time
import traceback

from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import QApplication, QMainWindow, QMessageBox, QStackedWidget
from rclpy.utilities import remove_ros_args

from . import ikonlar, tema
from .algilama import AlgilamaBolumu
from .ana_ekran import AnaEkran, GenelAyarlarPenceresi
from .ayar_tanim import GenelAyarlar
from .bolumler import EngelSayfasi, IzleSayfasi, KayitSayfasi, OynatSayfasi
from .kayitlar import KayitYapici
from .ros_baglanti import RosBaglanti
from .surecler import DURUYOR, SurecYonetici, disaridaki_surecler, pid_kapat

# Bu programla aynı anda başka yerden açılmışsa veriyi karıştıracak programlar
CATISMALAR = {
    "surucu": ["surucu", "oynatici", "launch"],
    "oynatici": ["surucu", "oynatici", "launch"],
    "engel": ["engel", "launch"],
    "nesne": ["nesne", "launch"],
}


class AnaPencere(QMainWindow):
    def __init__(self, ros):
        super().__init__()
        self.ros = ros
        self.genel = GenelAyarlar()
        tema.uygula(QApplication.instance(), self.genel["tema"])
        self.setWindowTitle("Robotaksi Kontrol Merkezi")
        self.setWindowIcon(ikonlar.uygulama_ikonu())

        self.yonetici = SurecYonetici(self)
        self.yonetici.degisti.connect(self._surec_degisti)
        self.kayit = KayitYapici(self.yonetici, ros, self.genel, self)
        # Engel Algılama ve Kayıt Oynat aynı algılama görünümünü kullanır (sayfa değişince taşınır)
        self.algilama = AlgilamaBolumu(ros, self.genel)

        self.yigin = QStackedWidget()
        self.setCentralWidget(self.yigin)
        self.sayfalar = {"ana": AnaEkran(self), "engel": EngelSayfasi(self), "izle": IzleSayfasi(self),
                         "kayit": KayitSayfasi(self), "oynat": OynatSayfasi(self)}
        for s in self.sayfalar.values():
            self.yigin.addWidget(s)
        self.aktif = self.sayfalar["ana"]
        self.yigin.setCurrentWidget(self.aktif)

        self._bekleyen = {}           # başlatılmayı bekleyen programlar: anahtar → Gerek
        self._kapaniyor = False
        self._kapanabilir = False
        self._kapanma_baslangici = 0.0

        self.tik = QTimer(self)
        self.tik.timeout.connect(self._tik)
        self.tik.start(100)

        alan = (self.screen() or QApplication.primaryScreen()).availableGeometry()
        if alan.width() <= 1440 or alan.height() <= 860:
            self.showMaximized()
        else:
            self.resize(1320, 860)
            self.move(alan.center().x() - 660, alan.center().y() - 430)
        self.aktif.girildi()

    # ---------------- bölüm değiştirme ----------------
    def bolum_ac(self, ad, **secenek):
        yeni = self.sayfalar[ad]
        if yeni is self.aktif:
            return
        # Kayıt sadece sürücüye bağlı: yeni bölümde de sürücü açık kalacaksa kayıt sürer
        if self.kayit.kayitta:
            if "surucu" not in self._gereken(yeni):
                cevap = QMessageBox.question(
                    self, "Kayıt sürüyor",
                    "Kayıt hâlâ sürüyor. Bu bölüme geçmek için kayıt durdurulacak.\n\nKayıt durdurulup "
                    "geçilsin mi?")
                if cevap != QMessageBox.Yes:
                    return
                self.kayit.durdur("Bölüm değiştirildi.")
        self.aktif.ayrildi()
        self.aktif = yeni
        yeni.hazirla(**secenek)
        self.yigin.setCurrentWidget(yeni)
        yeni.girildi()
        self.esitle()

    @staticmethod
    def _gereken(sayfa):
        return sayfa.gereken_surecler() if hasattr(sayfa, "gereken_surecler") else {}

    def esitle(self):
        """Çalışan programları aktif bölümün ihtiyacına göre kapatır / açar / yeniden başlatır."""
        gereken = self._gereken(self.aktif)
        for anahtar, s in self.yonetici.surecler.items():
            if anahtar == "kayit":  # kayıt kullanıcının düğmesiyle yönetilir
                continue
            if s.acik and (anahtar not in gereken or gereken[anahtar].kimlik != s.kimlik):
                s.durdur()
        baslatilacak = {}
        for anahtar, g in gereken.items():
            s = self.yonetici.surecler.get(anahtar)
            if s and s.acik and s.kimlik == g.kimlik and s.durum != DURUYOR:
                continue
            baslatilacak[anahtar] = g
        if baslatilacak and not self._catismalari_coz(baslatilacak):
            self._bekleyen = {}
            return
        self._bekleyen = baslatilacak
        self._bekleyenleri_baslat()

    def _catismalari_coz(self, baslatilacak):
        """Başka bir yerden açılmış, veriyi karıştıracak programlar varsa kapatmayı önerir."""
        turler = sorted({t for k in baslatilacak for t in CATISMALAR.get(k, [])})
        if not turler:
            return True
        dis = disaridaki_surecler(turler, self.yonetici.bizim_pidler())
        if not dis:
            return True
        liste = "\n".join(f"  • {ad} (pid {pid})" for pid, ad, _ in dis)
        cevap = QMessageBox.question(
            self, "Başka yerden açılmış programlar var",
            "Bu bilgisayarda başka bir yerden (eski kısayol ya da terminal) açılmış programlar çalışıyor:\n\n"
            f"{liste}\n\nBunlar açıkken veriler karışır (iki sürücü aynı LiDAR portunu kullanamaz).\n"
            "Kapatılıp devam edilsin mi?")
        if cevap != QMessageBox.Yes:
            return False
        pid_kapat([pid for pid, _, _ in dis], olay_isle=QApplication.processEvents)
        return True

    def _bekleyenleri_baslat(self):
        if not self._bekleyen:
            return
        # Önce kapanan programların (ör. eski RViz, sürücü) tamamen kapanmasını bekle
        if any(s.durum == DURUYOR for k, s in self.yonetici.surecler.items() if k != "kayit"):
            return
        for anahtar, g in list(self._bekleyen.items()):
            s = self.yonetici.surec(anahtar, g.ad)
            if not s.acik:
                komut = g.komut_fn()
                if not komut or not komut[0]:
                    s._satir_ekle(f"Program bulunamadı: {g.ad} (paket derlendi mi?)")
                    s.durum = "hata"
                    continue
                s.baslat(g.kimlik, komut)
        self._bekleyen = {}

    def bekleyen_var(self):
        return bool(self._bekleyen)

    def yeniden_baslat(self, anahtar):
        gereken = self._gereken(self.aktif)
        if anahtar not in gereken:
            return
        s = self.yonetici.surecler.get(anahtar)
        if s and s.acik:
            s.durdur()
        self._bekleyen[anahtar] = gereken[anahtar]
        self._bekleyenleri_baslat()

    def _surec_degisti(self, anahtar):
        if hasattr(self.aktif, "surec_degisti"):
            self.aktif.surec_degisti(anahtar)

    # ---------------- pencere yerleşimi ----------------
    def rviz_geometrisi(self):
        """'Yan yana' açıksa bu pencereyi ekranın soluna alır, RViz'e sağ yarıyı verir."""
        if not self.genel["yan_yana"]:
            return None
        alan = (self.screen() or QApplication.primaryScreen()).availableGeometry()
        if self.isMaximized() or self.isFullScreen():
            self.showNormal()
        baslik = max(0, self.frameGeometry().height() - self.geometry().height()) or 32
        sol = int(alan.width() * 0.5)
        self.move(alan.x(), alan.y())
        self.resize(sol, alan.height() - baslik)
        return (alan.x() + sol, alan.y(), alan.width() - sol, alan.height() - baslik)

    # ---------------- tema / ayarlar ----------------
    def tema_uygula(self, ad):
        tema.uygula(QApplication.instance(), ad)
        self.genel["tema"] = ad
        ikonlar.yenile()
        self.sayfalar["ana"].tema_guncelle()
        for w in QApplication.instance().allWidgets():
            w.update()

    def tema_degistir(self):
        self.tema_uygula("acik" if tema.aktif_ad == "koyu" else "koyu")

    def genel_ayarlar_ac(self):
        GenelAyarlarPenceresi(self).exec_()

    # ---------------- zamanlayıcı ----------------
    def _tik(self):
        if self._kapaniyor:
            if self.yonetici.hepsi_kapali() or time.monotonic() - self._kapanma_baslangici > 30:
                self._kapanabilir = True
                self.close()
            return
        self._bekleyenleri_baslat()
        self.aktif.yenile()

    # ---------------- kapanış ----------------
    def closeEvent(self, olay):
        if self._kapanabilir:
            olay.accept()
            return
        if self._kapaniyor:
            olay.ignore()
            return
        if self.kayit.kayitta and QMessageBox.question(
                self, "Kayıt sürüyor", "Kayıt hâlâ sürüyor. Kayıt düzgünce kapatılıp çıkılsın mı?") \
                != QMessageBox.Yes:
            olay.ignore()
            return
        self.aktif.ayrildi()
        if self.yonetici.hepsi_kapali():
            olay.accept()
            return
        # Programları düzgün kapat (kayıt dosyası yarım kalmasın), bitince pencereyi kapat
        olay.ignore()
        self._kapaniyor = True
        self._kapanma_baslangici = time.monotonic()
        self._bekleyen = {}
        self.yonetici.hepsini_durdur()
        self.setEnabled(False)
        self.setWindowTitle("Robotaksi Kontrol Merkezi — programlar kapatılıyor…")


def hata_yakalayici(tur, deger, iz):
    """PyQt5, zamanlayıcı/düğme içindeki yakalanmamış hatada programı kapatır. Bunun yerine
    hatayı günlüğe yaz ve devam et (arka plandaki programlar sahipsiz kalmasın)."""
    print("[kontrol_merkezi] Beklenmedik hata:", flush=True)
    traceback.print_exception(tur, deger, iz)
    sys.stdout.flush()


def main():
    sys.excepthook = hata_yakalayici
    uygulama = QApplication(remove_ros_args(sys.argv))
    uygulama.setApplicationName("Robotaksi Kontrol Merkezi")
    uygulama.setDesktopFileName("kontrol-merkezi")
    ros = RosBaglanti("kontrol_merkezi", args=sys.argv, engel_hedef="engel_node", nesne_hedef="nesne_node")
    try:
        pencere = AnaPencere(ros)
        pencere.show()

        # Terminalden açıldıysa Ctrl+C de pencereyi düzgün kapatır
        for s in (signal.SIGINT, signal.SIGTERM):
            signal.signal(s, lambda *_: QTimer.singleShot(0, pencere.close))
        nabiz = QTimer()
        nabiz.timeout.connect(lambda: None)  # Python'un sinyalleri işleyebilmesi için
        nabiz.start(250)
        return uygulama.exec_()
    finally:
        ros.kapat()
