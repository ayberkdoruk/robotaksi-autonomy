"""
Arka plandaki programların komut satırları.

Programlar 'ros2 run' yerine doğrudan çalıştırılır (araya fazladan süreç girmesin,
kapatma sinyali doğrudan programa gitsin). Ayarlar launch dosyalarıyla AYNI:
  - sürücü: rslidar_sdk/launch/start.py gibi, kendi config.yaml'ı ile
  - engel_node / nesne_node: config/*.yaml + ~/.ros/engel_algilama/son_ayarlar.yaml
"""

import copy
import os

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory

from .ayar_tanim import KAYIT_KOKU, SON_AYARLAR, varsayilan_dosyasi

RVIZ_KLASORU = os.path.join(KAYIT_KOKU, "rviz")


def paket_programi(paket, program):
    try:
        yol = os.path.join(get_package_prefix(paket), "lib", paket, program)
    except PackageNotFoundError:
        return None
    return yol if os.path.exists(yol) else None


def eksik_paketler():
    """Kurulu olmayan programlar (ör. sürücü derlenmemiş): [açıklama, ...]"""
    eksik = []
    for paket, program, ad in (("rslidar_sdk", "rslidar_sdk_node", "LiDAR sürücüsü (rslidar_sdk)"),
                               ("engel_algilama", "engel_node", "engel_node"),
                               ("engel_algilama", "nesne_node", "nesne_node"),
                               ("rviz2", "rviz2", "RViz")):
        if paket_programi(paket, program) is None:
            eksik.append(ad)
    return eksik


def _params(node):
    dosyalar = [varsayilan_dosyasi(node)]
    if os.path.exists(SON_AYARLAR):
        dosyalar.append(SON_AYARLAR)  # panelde en son uygulanan ayarlar varsayılanların üstüne
    arg = []
    for d in dosyalar:
        if d:
            arg += ["--params-file", d]
    return arg


def surucu():
    # rslidar_sdk/launch/start.py ile aynı: namespace rslidar_sdk, config_path boş (kendi config.yaml'ı)
    return [paket_programi("rslidar_sdk", "rslidar_sdk_node"), "--ros-args", "-r", "__ns:=/rslidar_sdk"]


def engel_node():
    return [paket_programi("engel_algilama", "engel_node"), "--ros-args"] + _params("engel_node")


def nesne_node():
    return [paket_programi("engel_algilama", "nesne_node"), "--ros-args"] + _params("nesne_node")


def oynatici(kayit_yolu, hiz=1.0, dongu=True, baslangic_s=0.0):
    komut = ["ros2", "bag", "play", kayit_yolu,
             "--topics", "/rslidar_points",   # kayıtta algılama sonuçları varsa onlar oynatılmaz
             "--clock", "20",                 # ilerleme çubuğu için
             "--rate", f"{hiz:g}",
             "--disable-keyboard-controls",
             "--delay", "1"]                  # node'lar açılsın, ilk turlar kaçmasın
    if dongu:
        komut.append("--loop")
    if baslangic_s > 0.05:
        komut += ["--start-offset", f"{baslangic_s:.2f}"]
    return komut


def rviz(tur="tam", geometri=None):
    """
    tur: "tam" = tüm algılama görselleri, "lidar" = sadece ham nokta bulutu.
    geometri: (x, y, genişlik, yükseklik) verilirse RViz penceresi oraya açılır.
    """
    return [paket_programi("rviz2", "rviz2"), "-d", rviz_ayar_dosyasi(tur, geometri)]


def rviz_ayar_dosyasi(tur, geometri=None):
    kaynak = os.path.join(get_package_share_directory("engel_algilama"), "rviz", "engel.rviz")
    if tur == "tam" and geometri is None:
        return kaynak
    try:
        with open(kaynak, encoding="utf-8") as f:
            ayar = yaml.safe_load(f)
        ayar = copy.deepcopy(ayar)
        if tur == "lidar":
            gosterimler = ayar["Visualization Manager"]["Displays"]
            ayar["Visualization Manager"]["Displays"] = [
                g for g in gosterimler
                if g.get("Class", "").endswith("/Grid")
                or (g.get("Topic", {}) or {}).get("Value") == "/rslidar_points"]
        if geometri is not None:
            x, y, w, h = (int(v) for v in geometri)
            pencere = ayar.setdefault("Window Geometry", {})
            pencere.update({"X": x, "Y": y, "Width": w, "Height": h})
        os.makedirs(RVIZ_KLASORU, exist_ok=True)
        hedef = os.path.join(RVIZ_KLASORU, f"{tur}.rviz")
        with open(hedef, "w", encoding="utf-8") as f:
            yaml.safe_dump(ayar, f, allow_unicode=True, sort_keys=False)
        return hedef
    except (OSError, yaml.YAMLError, KeyError, TypeError):
        return kaynak
