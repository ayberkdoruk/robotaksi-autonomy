"""
LiDAR ile ilgili bilgiler: sürücünün ayar dosyası (sadece okunur), kablo üzerinden
paket gelip gelmediği ve disk durumu.

NOT: Sürücü ayarları burada DEĞİŞTİRİLMEZ. Yanlış model/port seçilirse LiDAR'dan hiç
veri gelmez ve araçta bunu fark etmek zordur; bu yüzden sadece gösterilir.
"""

import errno
import os
import shutil
import socket
import time

import yaml

MODEL_ADLARI = {
    "RSHELIOS": "Helios 32", "RSHELIOS_16P": "Helios 16P", "RS16": "RS-LiDAR-16",
    "RS32": "RS-LiDAR-32", "RSBP": "Bpearl", "RS128": "Ruby 128", "RS80": "Ruby Lite 80",
    "RS48": "Ruby 48", "RSP128": "Ruby Plus 128", "RSP80": "Ruby Plus 80", "RSP48": "Ruby Plus 48",
    "RSM1": "M1", "RSM2": "M2", "RSM3": "M3", "RSE1": "E1", "RSAIRY": "Airy",
}
VERI_KAYNAKLARI = {0: "LiDAR kullanılmıyor", 1: "canlı LiDAR", 2: "ROS paketlerinden", 3: "pcap dosyasından"}

# Kayıt boyutu tahmini (gerçek kayıtlardan: zstd ile dakikada ~0.5, sıkıştırmasız ~0.9 GB)
DAKIKADA_GB = {"zstd_fast": 0.55, "fastwrite": 0.9}


def surucu_ayar_dosyasi():
    """rslidar_sdk'nın kullandığı config.yaml (sürücü kaynak klasöründekini okur)."""
    adaylar = [os.path.expanduser("~/lidar_ws/src/rslidar_sdk/config/config.yaml")]
    try:
        from ament_index_python.packages import get_package_prefix, get_package_share_directory
        adaylar.append(os.path.join(get_package_share_directory("rslidar_sdk"), "config", "config.yaml"))
        # colcon --symlink-install: kaynak klasör, kurulum klasörünün yanındaki src/ altında
        onek = get_package_prefix("rslidar_sdk")
        adaylar.append(os.path.join(os.path.dirname(os.path.dirname(onek)), "src", "rslidar_sdk",
                                    "config", "config.yaml"))
    except Exception:
        pass
    for yol in adaylar:
        if os.path.isfile(yol):
            return yol
    return None


def surucu_ayarlari():
    """Sürücü ayarlarının özeti. Okunamazsa {'hata': ...}."""
    yol = surucu_ayar_dosyasi()
    if not yol:
        return {"hata": "rslidar_sdk/config/config.yaml bulunamadı"}
    try:
        with open(yol, encoding="utf-8") as f:
            veri = yaml.safe_load(f) or {}
        ortak = veri.get("common", {}) or {}
        lidar = (veri.get("lidar") or [{}])[0] or {}
        surucu, ros = lidar.get("driver", {}) or {}, lidar.get("ros", {}) or {}
    except (OSError, yaml.YAMLError, AttributeError, IndexError) as e:
        return {"hata": f"Sürücü ayar dosyası okunamadı: {e}", "dosya": yol}
    tur = str(surucu.get("lidar_type", "?"))
    return {
        "dosya": yol,
        "model": tur,
        "model_adi": MODEL_ADLARI.get(tur, tur),
        "msop": surucu.get("msop_port", 6699),
        "difop": surucu.get("difop_port", 7788),
        "min_mesafe": surucu.get("min_distance"),
        "max_mesafe": surucu.get("max_distance"),
        "kaynak": VERI_KAYNAKLARI.get(ortak.get("msg_source"), str(ortak.get("msg_source"))),
        "topic": ros.get("ros_send_point_cloud_topic", "/rslidar_points"),
        "frame": ros.get("ros_frame_id", "rslidar"),
    }


class PaketDinleyici:
    """
    Sürücü KAPALIYKEN LiDAR'ın veri portunu dinler: kablo ve güç tamam mı?
    Sürücü açılmadan önce kapatılmalı (aynı portu iki program dinleyemez).
    """

    def __init__(self):
        self.soket = None
        self.port = None
        self.durum = "kapali"     # kapali | bekliyor | geliyor | gelmiyor | mesgul | hata
        self.kaynak = None        # paketin geldiği IP
        self.paket_hizi = 0.0
        self._son_paket = 0.0
        self._baslama = 0.0
        self._son_yoklama = 0.0
        self.hata = ""

    def baslat(self, port):
        self.durdur()
        self.port = int(port)
        self._baslama = time.monotonic()
        self._son_yoklama = self._baslama
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setblocking(False)
            s.bind(("", self.port))
            self.soket = s
            self.durum = "bekliyor"
        except OSError as e:
            self.soket = None
            self.durum = "mesgul" if e.errno == errno.EADDRINUSE else "hata"
            self.hata = str(e)

    def durdur(self):
        if self.soket is not None:
            try:
                self.soket.close()
            except OSError:
                pass
        self.soket = None
        if self.durum != "mesgul":
            self.durum = "kapali"

    def yokla(self):
        """Biriken paketleri okuyup atar, durumu günceller (saniyede birkaç kez çağrılmalı)."""
        if self.soket is None:
            return self.durum
        sayi = 0
        try:
            while sayi < 20000:
                _, adres = self.soket.recvfrom(2048)
                self.kaynak = adres[0]
                sayi += 1
        except BlockingIOError:
            pass
        except OSError:
            pass
        simdi = time.monotonic()
        gecen = max(simdi - self._son_yoklama, 1e-3)
        self._son_yoklama = simdi
        self.paket_hizi = 0.7 * self.paket_hizi + 0.3 * (sayi / gecen)
        if sayi:
            self._son_paket = simdi
        if simdi - self._son_paket < 1.5:
            self.durum = "geliyor"
        elif simdi - self._baslama > 2.0:
            self.durum = "gelmiyor"
        return self.durum


def disk_durumu(klasor):
    """(boş bayt, toplam bayt). Klasör yoksa en yakın var olan üst klasöre bakılır."""
    yol = os.path.abspath(klasor)
    while not os.path.exists(yol) and os.path.dirname(yol) != yol:
        yol = os.path.dirname(yol)
    try:
        d = shutil.disk_usage(yol)
        return d.free, d.total
    except OSError:
        return None, None


def kayit_suresi_tahmini(bos_bayt, sikistirma="zstd_fast"):
    """Boş yer kaç dakikalık kayda yeter."""
    if not bos_bayt:
        return 0.0
    return bos_bayt / 1024 ** 3 / DAKIKADA_GB.get(sikistirma, 0.6)
