"""
Arka planda çalışan programlar (LiDAR sürücüsü, engel_node, nesne_node, RViz,
kayıt, oynatıcı).

Her program kendi süreç grubunda açılır; terminal çıktısı yakalanıp arayüzdeki
"Program çıktıları" bölümünde gösterilir. Kapatırken önce Ctrl+C (SIGINT) gönderilir,
böylece kayıt dosyası düzgün kapanır; kapanmazsa SIGTERM, en son SIGKILL.

Arayüz beklenmedik şekilde kapanırsa (çökme) programlar da kapanır: her biri
"setpriv --pdeathsig TERM" ile, ebeveyni ölünce kendine SIGTERM gelecek şekilde başlatılır.
"""

import collections
import os
import re
import shutil
import signal
import subprocess
import threading
import time

from python_qt_binding.QtCore import QObject, QTimer, Signal

SETPRIV = shutil.which("setpriv")
RENK_KODU = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")  # sürücünün renkli yazıları
STDBUF = shutil.which("stdbuf")

# Durumlar
KAPALI, BASLIYOR, CALISIYOR, DURUYOR, HATA, BITTI = (
    "kapali", "basliyor", "calisiyor", "duruyor", "hata", "bitti")
DURUM_ADLARI = {KAPALI: "kapalı", BASLIYOR: "açılıyor…", CALISIYOR: "çalışıyor",
                DURUYOR: "kapanıyor…", HATA: "hata verdi", BITTI: "kapandı"}
DURUM_RENKLERI = {KAPALI: "silik", BASLIYOR: "uyari", CALISIYOR: "basari",
                  DURUYOR: "uyari", HATA: "hata", BITTI: "silik"}


class Surec:
    def __init__(self, anahtar, ad):
        self.anahtar, self.ad = anahtar, ad
        self.proc = None
        self.durum = KAPALI
        self.kimlik = None          # hangi ayarla açıldı (değişirse yeniden başlatılır)
        self.komut = None
        self.cikis_kodu = None
        self.baslama = 0.0
        self.durdurma_suresi = 6.0  # SIGINT'ten sonra bu kadar beklenir
        self._durdur_istendi = False
        self._durdurma_zamani = 0.0
        self._asama = 0
        self._kilit = threading.Lock()
        self.satirlar = collections.deque(maxlen=4000)
        self.satir_no = 0           # şimdiye kadar gelen toplam satır

    # ---------------- çıktı ----------------
    def _satir_ekle(self, s):
        with self._kilit:
            self.satirlar.append(s)
            self.satir_no += 1

    def yeni_satirlar(self, son_no):
        """son_no'dan sonra gelen satırlar ve yeni son_no."""
        with self._kilit:
            fark = min(self.satir_no - son_no, len(self.satirlar))
            if fark <= 0:
                return [], self.satir_no
            return list(self.satirlar)[-fark:], self.satir_no

    def son_satirlar(self, n=8):
        with self._kilit:
            return list(self.satirlar)[-n:]

    def temizle(self):
        with self._kilit:
            self.satirlar.clear()

    def _oku(self, proc):
        try:
            for satir in iter(proc.stdout.readline, b""):
                self._satir_ekle(RENK_KODU.sub("", satir.decode("utf-8", "replace").rstrip("\r\n")))
        except (OSError, ValueError):
            pass
        finally:
            try:
                proc.stdout.close()
            except OSError:
                pass

    # ---------------- başlat / durdur ----------------
    @property
    def acik(self):
        """Süreç hâlâ yaşıyor mu (çalışıyor ya da kapanıyor)?"""
        return self.proc is not None

    @property
    def pid(self):
        return self.proc.pid if self.proc else None

    def baslat(self, kimlik, komut):
        if self.acik:
            return
        self.kimlik, self.komut = kimlik, list(komut)
        tam = list(komut)
        if STDBUF:   # çıktı borudan satır satır gelsin (yoksa C++ programları blok blok yazar)
            tam = [STDBUF, "-oL", "-eL"] + tam
        if SETPRIV:  # arayüz çökerse program da kapansın
            tam = [SETPRIV, "--pdeathsig", "TERM", "--"] + tam
        ortam = dict(os.environ, RCUTILS_COLORIZED_OUTPUT="0", PYTHONUNBUFFERED="1")
        self._satir_ekle("")
        self._satir_ekle(f"──── {time.strftime('%H:%M:%S')}  başlatılıyor: {' '.join(komut)}")
        self._durdur_istendi, self._asama, self.cikis_kodu = False, 0, None
        try:
            self.proc = subprocess.Popen(
                tam, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                start_new_session=True, env=ortam)
        except OSError as e:
            self.proc = None
            self.durum = HATA
            self._satir_ekle(f"Başlatılamadı: {e}")
            return
        self.durum = BASLIYOR
        self.baslama = time.monotonic()
        threading.Thread(target=self._oku, args=(self.proc,), daemon=True).start()

    def durdur(self):
        if not self.acik or self._durdur_istendi:
            return
        self._durdur_istendi = True
        self.durum = DURUYOR
        self._durdurma_zamani = time.monotonic()
        self._sinyal(signal.SIGINT)

    def _sinyal(self, s):
        try:
            os.killpg(self.proc.pid, s)
        except (ProcessLookupError, PermissionError, OSError):
            try:
                self.proc.send_signal(s)
            except (ProcessLookupError, OSError):
                pass

    def calisma_suresi(self):
        return time.monotonic() - self.baslama if self.acik else 0.0

    def guncelle(self):
        """Zamanlayıcıdan çağrılır. Durum değiştiyse True."""
        if self.proc is None:
            return False
        eski = self.durum
        kod = self.proc.poll()
        if kod is not None:
            self.cikis_kodu = kod
            self.proc = None
            if self._durdur_istendi:
                self.durum = KAPALI
                self._satir_ekle(f"──── {time.strftime('%H:%M:%S')}  kapatıldı")
            elif kod == 0:
                self.durum = BITTI
                self._satir_ekle(f"──── {time.strftime('%H:%M:%S')}  kendiliğinden kapandı")
            else:
                self.durum = HATA
                self._satir_ekle(f"──── {time.strftime('%H:%M:%S')}  beklenmedik şekilde kapandı "
                                 f"(çıkış kodu {kod})")
        elif self.durum == BASLIYOR and time.monotonic() - self.baslama > 1.5:
            self.durum = CALISIYOR
        elif self.durum == DURUYOR:
            gecen = time.monotonic() - self._durdurma_zamani
            if gecen > self.durdurma_suresi and self._asama == 0:
                self._asama = 1
                self._satir_ekle("(Ctrl+C ile kapanmadı, SIGTERM gönderiliyor)")
                self._sinyal(signal.SIGTERM)
            elif gecen > self.durdurma_suresi + 3.0 and self._asama == 1:
                self._asama = 2
                self._satir_ekle("(SIGTERM ile de kapanmadı, zorla kapatılıyor)")
                self._sinyal(signal.SIGKILL)
        return self.durum != eski


class SurecYonetici(QObject):
    """Tüm programları tutar, 200 ms'de bir durumlarına bakar."""

    degisti = Signal(str)  # durumu değişen programın anahtarı

    def __init__(self, parent=None):
        super().__init__(parent)
        self.surecler = {}
        self._zamanlayici = QTimer(self)
        self._zamanlayici.timeout.connect(self._yokla)
        self._zamanlayici.start(200)

    def surec(self, anahtar, ad=None):
        if anahtar not in self.surecler:
            self.surecler[anahtar] = Surec(anahtar, ad or anahtar)
        elif ad:
            self.surecler[anahtar].ad = ad
        return self.surecler[anahtar]

    def _yokla(self):
        for s in list(self.surecler.values()):
            if s.guncelle():
                self.degisti.emit(s.anahtar)

    def hepsini_durdur(self):
        for s in self.surecler.values():
            s.durdur()

    def hepsi_kapali(self):
        return not any(s.acik for s in self.surecler.values())

    def bizim_pidler(self):
        return {s.pid for s in self.surecler.values() if s.pid}


# -----------------------------------------------------------------------------
# Başka bir yerden (eski kısayollar, terminal) açılmış programlar
# -----------------------------------------------------------------------------
def _ros2_alt_komut(arg, *beklenen):
    """['/usr/bin/python3', '/opt/ros/jazzy/bin/ros2', 'bag', 'play', ...] → ros2'den sonrası beklenen mi?"""
    for i, a in enumerate(arg[:3]):
        if os.path.basename(a) == "ros2":
            return tuple(arg[i + 1:i + 1 + len(beklenen)]) == beklenen
    return False


# tür → (görünen ad, pgrep ön eleme deseni, programın kendisi mi? (argv listesi → bool))
DIS_DESENLER = {
    "surucu": ("LiDAR sürücüsü", "rslidar_sdk_node",
               lambda arg: os.path.basename(arg[0]) == "rslidar_sdk_node"),
    "engel": ("engel_node", "engel_node",
              lambda arg: arg[0].endswith("lidar_algilama/engel_node")),
    "nesne": ("nesne_node", "nesne_node",
              lambda arg: arg[0].endswith("lidar_algilama/nesne_node")),
    "oynatici": ("kayıt oynatıcı (ros2 bag play)", "bag play",
                 lambda arg: _ros2_alt_komut(arg, "bag", "play")),
    "launch": ("ros2 launch (lidar_algilama / kontrol_merkezi)", "launch (lidar_algilama|kontrol_merkezi)",
               lambda arg: (_ros2_alt_komut(arg, "launch", "lidar_algilama")
                            or _ros2_alt_komut(arg, "launch", "kontrol_merkezi"))),
}


def _argv(pid):
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            return [a.decode("utf-8", "replace") for a in f.read().split(b"\0") if a]
    except OSError:
        return []


def disaridaki_surecler(anahtarlar, haric=()):
    """
    Verilen türlerde, bu arayüzün açmadığı çalışan programlar: [(pid, ad, komut), ...]
    Sadece programın KENDİSİNE bakılır (komut satırında adı geçen bir grep/terminal sayılmaz).
    """
    haric = set(haric) | {os.getpid()}
    bulunan, gorulen = [], set()
    for anahtar in anahtarlar:
        ad, desen, kendisi_mi = DIS_DESENLER[anahtar]
        try:
            cikti = subprocess.run(["pgrep", "-f", desen], capture_output=True, text=True,
                                   timeout=2).stdout
        except (OSError, subprocess.SubprocessError):
            continue
        for satir in cikti.split():
            try:
                pid = int(satir)
            except ValueError:
                continue
            if pid in haric or pid in gorulen:
                continue
            arg = _argv(pid)
            if not arg or not kendisi_mi(arg):
                continue
            gorulen.add(pid)
            bulunan.append((pid, ad, " ".join(arg)))
    return bulunan


def pid_kapat(pidler, bekle=6.0, olay_isle=None):
    """Programları önce Ctrl+C ile kapatmayı dener, kapanmayanı zorla kapatır."""
    def yasiyor(pid):
        try:
            os.kill(pid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            return True

    for pid in pidler:
        try:
            os.kill(pid, signal.SIGINT)
        except (ProcessLookupError, PermissionError):
            pass
    son = time.monotonic() + bekle
    while time.monotonic() < son and any(yasiyor(p) for p in pidler):
        if olay_isle:
            olay_isle()
        time.sleep(0.1)
    for pid in pidler:
        if yasiyor(pid):
            try:
                os.kill(pid, signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
    return not any(yasiyor(p) for p in pidler)
