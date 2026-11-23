"""
rosbag kayıtları: listeleme, yeniden adlandırma, çöp kutusuna atma ve kayıt alma.

Kayıtlar <kayit_klasoru>/TARIH_SAAT_isim/ klasörlerine MCAP biçiminde gider
(lidar_kayit.sh ile aynı düzen, iki yöntemle alınan kayıtlar birlikte listelenir).
"""

import datetime
import os
import re
import subprocess
import time
from dataclasses import dataclass, field

import yaml
from python_qt_binding.QtCore import QObject, QTimer, Signal

from . import lidar_bilgi

TARIHLI_AD = re.compile(r"^(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})_(.*)$")

# Algılama sonuçlarını da kaydet seçeneğinde eklenen topic'ler
ALGILAMA_KONULARI = ["/engel/karar", "/engel/mesafe", "/engel/engel_noktasi", "/engel/gorsel",
                     "/nesne/liste", "/nesne/gorsel"]

SIKISTIRMALAR = [("zstd_fast", "Hızlı sıkıştırma (önerilen, ~yarı boyut)"),
                 ("fastwrite", "Sıkıştırmasız (bilgisayar kasarsa)")]


@dataclass
class Kayit:
    yol: str
    ad: str
    tarih: datetime.datetime = None
    sure_s: float = 0.0
    baslangic_ns: int = 0
    boyut: int = 0
    mesaj: int = 0
    konular: list = field(default_factory=list)

    @property
    def klasor_adi(self):
        return os.path.basename(self.yol)

    @property
    def algilama_var(self):
        return any(k.startswith(("/engel/", "/nesne/")) for k in self.konular)


def klasor_boyutu(yol):
    toplam = 0
    try:
        with os.scandir(yol) as it:
            for g in it:
                if g.is_file(follow_symlinks=False):
                    toplam += g.stat().st_size
    except OSError:
        pass
    return toplam


def kayit_oku(yol):
    """Bir kayıt klasörünün bilgileri. Geçerli bir rosbag değilse None."""
    meta = os.path.join(yol, "metadata.yaml")
    if not os.path.isfile(meta):
        return None
    try:
        with open(meta, encoding="utf-8") as f:
            bilgi = (yaml.safe_load(f) or {}).get("rosbag2_bagfile_information", {})
    except (OSError, yaml.YAMLError, AttributeError):
        return None
    klasor = os.path.basename(yol.rstrip("/"))
    eslesme = TARIHLI_AD.match(klasor)
    ad = eslesme.group(2) if eslesme and eslesme.group(2) else klasor
    baslangic = int((bilgi.get("starting_time") or {}).get("nanoseconds_since_epoch", 0) or 0)
    tarih = datetime.datetime.fromtimestamp(baslangic / 1e9) if baslangic else None
    if tarih is None and eslesme:
        try:
            tarih = datetime.datetime.strptime(eslesme.group(1), "%Y-%m-%d_%H-%M-%S")
        except ValueError:
            pass
    konular = []
    for k in bilgi.get("topics_with_message_count", []) or []:
        try:
            konular.append(k["topic_metadata"]["name"])
        except (KeyError, TypeError):
            pass
    return Kayit(
        yol=yol, ad=ad, tarih=tarih,
        sure_s=int((bilgi.get("duration") or {}).get("nanoseconds", 0) or 0) / 1e9,
        baslangic_ns=baslangic, boyut=klasor_boyutu(yol),
        mesaj=int(bilgi.get("message_count", 0) or 0), konular=konular)


def kayitlari_listele(klasor):
    """Klasördeki kayıtlar, en yenisi başta."""
    sonuc = []
    try:
        with os.scandir(klasor) as it:
            for g in it:
                if g.is_dir():
                    k = kayit_oku(g.path)
                    if k:
                        sonuc.append(k)
    except OSError:
        return []
    sonuc.sort(key=lambda k: (k.tarih or datetime.datetime.min), reverse=True)
    return sonuc


def isim_temizle(isim):
    """Dosya adında sorun çıkaracak karakterleri temizler (Türkçe harfler kalır)."""
    isim = re.sub(r"[^\w\-]+", "_", (isim or "").strip()).strip("_")
    return isim[:60]


def yeniden_adlandir(kayit, yeni_isim):
    """Klasörün tarih kısmını korur, isim kısmını değiştirir. Yeni yolu döndürür."""
    yeni_isim = isim_temizle(yeni_isim)
    if not yeni_isim:
        raise ValueError("Geçerli bir isim girin.")
    eslesme = TARIHLI_AD.match(kayit.klasor_adi)
    yeni_klasor = f"{eslesme.group(1)}_{yeni_isim}" if eslesme else yeni_isim
    yeni_yol = os.path.join(os.path.dirname(kayit.yol), yeni_klasor)
    if os.path.exists(yeni_yol):
        raise ValueError("Bu isimde bir kayıt zaten var.")
    os.rename(kayit.yol, yeni_yol)
    return yeni_yol


def cop_kutusuna_at(yol):
    """Kaydı kalıcı olarak silmez, çöp kutusuna taşır (geri alınabilir)."""
    try:
        sonuc = subprocess.run(["gio", "trash", yol], capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.SubprocessError) as e:
        raise OSError(f"Çöp kutusuna taşınamadı: {e}")
    if sonuc.returncode != 0:
        raise OSError(sonuc.stderr.strip() or "Çöp kutusuna taşınamadı.")


# -----------------------------------------------------------------------------
# Kayıt alma
# -----------------------------------------------------------------------------
class KayitYapici(QObject):
    """
    'ros2 bag record' sürecini yönetir. Hem Kayıt Al sayfası hem de Engel Algılama /
    LiDAR'ı İzle sayfalarındaki hızlı kayıt çubuğu bunu kullanır (aynı anda tek kayıt).
    """

    degisti = Signal()
    bitti = Signal(str, bool, str)  # yol, başarılı mı, mesaj

    MIN_BOS_GB = 1.0   # bunun altına inince kayıt kendiliğinden durur
    UYARI_GB = 5.0

    def __init__(self, yonetici, ros, genel, parent=None):
        super().__init__(parent)
        self.ros, self.genel = ros, genel
        self.surec = yonetici.surec("kayit", "Kayıt")
        self.surec.durdurma_suresi = 20.0  # büyük kayıt dosyasının kapanması zaman alabilir
        self.yol = None
        self.sure_s = 0
        self.profil = "zstd_fast"
        self.boyut = 0
        self.bos = None
        self.tur_baslangic = 0
        self.durma_nedeni = ""
        self._zamanlayici = QTimer(self)
        self._zamanlayici.timeout.connect(self._yokla)
        self._zamanlayici.start(1000)
        yonetici.degisti.connect(self._surec_degisti)

    @property
    def kayitta(self):
        return self.surec.acik

    def gecen(self):
        return self.surec.calisma_suresi()

    def tur_sayisi(self):
        return max(0, self.ros.lidar_sayac.toplam - self.tur_baslangic)

    def baslatilabilir_mi(self):
        """(evet/hayır, neden)"""
        if self.kayitta:
            return False, "Kayıt zaten sürüyor."
        if self.ros.lidar_sayac.yas() > 2.0:
            return False, "LiDAR'dan veri gelmiyor (/rslidar_points)."
        bos, _ = lidar_bilgi.disk_durumu(self.genel.kayit_klasoru)
        if bos is not None and bos < self.MIN_BOS_GB * 1024 ** 3:
            return False, f"Diskte {self.MIN_BOS_GB:.0f} GB'tan az yer var."
        return True, ""

    def baslat(self, isim, sure_s=0, profil="zstd_fast", algilama=False):
        tamam, neden = self.baslatilabilir_mi()
        if not tamam:
            return neden
        klasor = self.genel.kayit_klasoru
        try:
            os.makedirs(klasor, exist_ok=True)
        except OSError as e:
            return f"Kayıt klasörü oluşturulamadı: {e}"
        isim = isim_temizle(isim) or "kayit"
        self.yol = os.path.join(klasor, f"{time.strftime('%Y-%m-%d_%H-%M-%S')}_{isim}")
        self.sure_s, self.profil, self.boyut, self.durma_nedeni = int(sure_s), profil, 0, ""
        konular = ["/rslidar_points"] + (ALGILAMA_KONULARI if algilama else [])
        komut = ["ros2", "bag", "record", *konular, "-o", self.yol, "-s", "mcap",
                 "--storage-preset-profile", profil]
        self.tur_baslangic = self.ros.lidar_sayac.toplam
        self.surec.baslat(("kayit", self.yol), komut)
        self.degisti.emit()
        return None

    def durdur(self, neden=""):
        if self.kayitta:
            self.durma_nedeni = neden
            self.surec.durdur()
            self.degisti.emit()

    def _yokla(self):
        if not self.kayitta or self.surec.durum == "duruyor":
            return
        self.boyut = klasor_boyutu(self.yol) if self.yol and os.path.isdir(self.yol) else 0
        self.bos, _ = lidar_bilgi.disk_durumu(self.genel.kayit_klasoru)
        if self.sure_s and self.gecen() >= self.sure_s:
            self.durdur("Seçilen süre doldu.")
        elif self.bos is not None and self.bos < self.MIN_BOS_GB * 1024 ** 3:
            self.durdur("Disk dolmak üzereydi, kayıt güvenle kapatıldı.")
        self.degisti.emit()

    def _surec_degisti(self, anahtar):
        if anahtar != "kayit":
            return
        self.degisti.emit()
        if self.surec.acik:
            return
        basarili = bool(self.yol) and os.path.isfile(os.path.join(self.yol, "metadata.yaml"))
        if basarili:
            mesaj = self.durma_nedeni or "Kayıt tamamlandı."
        elif self.surec.durum == "hata":
            mesaj = "Kayıt programı hata verdi; ayrıntı 'Program çıktıları'nda."
        else:
            mesaj = "Kayıt dosyası oluşmadı ya da eksik kaldı."
        self.bitti.emit(self.yol or "", basarili, mesaj)
