"""LiDAR algılama düğümlerini (engel_node + nesne_node) başlatan launch dosyası.

Kullanım:
  ros2 launch lidar_algilama lidar_algilama.launch.py              # iki düğüm
  ros2 launch lidar_algilama lidar_algilama.launch.py nesne:=false # nesne_node'u açma
  ros2 launch lidar_algilama lidar_algilama.launch.py arac_olculeri:=<yaml>
                                                                   # araç ölçülerinden hesapla

Ayar paneli bu pakette değil: sürücü + RViz + düğümler + panel için
  ros2 launch kontrol_merkezi lidar_sistem.launch.py

Parametreler şu sırayla yüklenir (sonraki öncekinin üstüne yazar):
  1) config/engel_params.yaml / config/nesne_params.yaml -> paketin varsayılan değerleri
  2) ~/.ros/kontrol_merkezi/son_ayarlar.yaml -> panelde en son uygulanan ayarlar (varsa)
     (dosyada "engel_node:" ve "nesne_node:" bölümleri ayrı ayrı durur)
  3) arac_olculeri verildiyse: o ölçülerden hesaplanan geometri (bakış yönü, yelpaze,
     zemin, gövde kutusu, koridor, DUR / DİKKAT mesafeleri). Panelin geri kalan ayarları
     (kümeleme, hareket, takip...) olduğu gibi kalır.
"""

import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SON_AYARLAR = os.path.expanduser('~/.ros/kontrol_merkezi/son_ayarlar.yaml')

OLCULER = ('lidar_yukseklik', 'lidar_on_tampon', 'lidar_arka_tampon',
           'lidar_sol_kenar', 'lidar_sag_kenar')


def olculeri_oku(yol):
    """arac_olculeri.yaml'ı okur. Ölçüler doldurulmamışsa (hepsi 0) None döner."""
    with open(yol) as f:
        o = yaml.safe_load(f) or {}
    if all(float(o.get(ad, 0.0)) == 0.0 for ad in OLCULER):
        return None
    eksik = [ad for ad in OLCULER if ad not in o]
    if eksik:
        raise RuntimeError(f'{yol}: eksik olcu: {", ".join(eksik)}')
    o = {ad: float(deger) for ad, deger in o.items()}
    if o['lidar_yukseklik'] <= o.get('zemin_payi', 0.15) + 0.1:
        raise RuntimeError(f'{yol}: lidar_yukseklik ({o["lidar_yukseklik"]}) cok kucuk')
    for ad in OLCULER[1:]:
        if o[ad] < 0.0:
            raise RuntimeError(f'{yol}: {ad} negatif olamaz (mesafe yazin, yon degil)')
    return o


def olculerden_parametreler(o):
    """Araç ölçülerinden (engel_node, nesne_node) parametre sözlüklerini hesaplar."""
    h = o['lidar_yukseklik']
    on = o['lidar_on_tampon']
    yarim = max(o['lidar_sol_kenar'], o['lidar_sag_kenar'])  # koridor LiDAR'a göre simetrik
    yan_pay = o.get('yan_pay', 0.35)
    zemin_payi = o.get('zemin_payi', 0.15)
    govde_payi = o.get('govde_payi', 0.10)

    dur = on + o.get('durma_boslugu', 2.0)
    dikkat = max(on + o.get('dikkat_boslugu', 4.0), dur + 0.5)
    koridor_yari = yarim + yan_pay
    # Yelpaze, DUR mesafesinde koridor genişliğini kapsayacak kadar açık. Pasta dilimi
    # olduğu için daha yakında daralır: tampon önündeki köşeleri yol koridoru (nesne_node)
    # yakalar.
    aci = 2.0 * math.degrees(math.atan2(koridor_yari, dur))
    aci = min(max(aci, 10.0), 180.0)

    engel = {
        'merkez_aci': 0.0,
        'gorus_genisligi': round(aci, 1),
        'min_mesafe': round(min(on + govde_payi, 3.0), 2),   # aracın kendi kaputu
        'z_min': round(-(h - zemin_payi), 2),
        'dur_mesafesi': round(dur, 2),
        'dikkat_mesafesi': round(dikkat, 2),
    }
    nesne = {
        'nesne_merkez_aci': 0.0,
        'nesne_gorus_genisligi': 180.0,
        'menzil': 20.0,
        'voxel_boyu': 0.08,
        'zemin_z': round(-h, 2),
        'govde_x_min': round(-(o['lidar_arka_tampon'] + govde_payi), 2),
        'govde_x_max': round(on + govde_payi, 2),
        'govde_y_min': round(-(o['lidar_sag_kenar'] + govde_payi), 2),
        'govde_y_max': round(o['lidar_sol_kenar'] + govde_payi, 2),
        'koridor_genislik': round(2.0 * koridor_yari, 2),
        'koridor_uzunluk': round(max(8.0, dikkat + 1.0), 1),
    }
    return engel, nesne


def _kur(context):
    paket = get_package_share_directory('lidar_algilama')
    engel_parametreler = [os.path.join(paket, 'config', 'engel_params.yaml')]
    nesne_parametreler = [os.path.join(paket, 'config', 'nesne_params.yaml')]
    eylemler = []

    if os.path.exists(SON_AYARLAR):
        engel_parametreler.append(SON_AYARLAR)
        nesne_parametreler.append(SON_AYARLAR)  # dosyadaki "nesne_node:" bölümü kullanılır
        eylemler.append(LogInfo(msg=f'Kayitli ayarlar yukleniyor: {SON_AYARLAR}'))
    else:
        eylemler.append(LogInfo(msg='Kayitli ayar yok, varsayilan degerler kullaniliyor.'))

    olcu_yolu = LaunchConfiguration('arac_olculeri').perform(context)
    if olcu_yolu:
        o = olculeri_oku(olcu_yolu)
        if o is None:
            eylemler.append(LogInfo(
                msg=f'UYARI: {olcu_yolu} doldurulmamis, arac olculeri UYGULANMADI.'))
        else:
            engel, nesne = olculerden_parametreler(o)
            engel_parametreler.append(engel)
            nesne_parametreler.append(nesne)
            satirlar = [f'Arac olculeri uygulandi ({olcu_yolu}):']
            satirlar += [f'  engel_node.{k} = {v}' for k, v in engel.items()]
            satirlar += [f'  nesne_node.{k} = {v}' for k, v in nesne.items()]
            eylemler.append(LogInfo(msg='\n'.join(satirlar)))

    eylemler.append(Node(
        package='lidar_algilama',
        executable='engel_node',
        name='engel_node',          # yaml'daki en üst anahtarla aynı olmalı
        output='screen',            # terminalde canlı satırlar görünsün
        parameters=engel_parametreler,
    ))
    if LaunchConfiguration('nesne').perform(context).lower() == 'true':
        eylemler.append(Node(
            package='lidar_algilama',
            executable='nesne_node',
            name='nesne_node',      # nesne_params.yaml'daki en üst anahtarla aynı olmalı
            output='screen',
            parameters=nesne_parametreler,
        ))
    return eylemler


def generate_launch_description():
    return LaunchDescription([
        # "nesne:=false" yazılırsa nesne algılama (kutular) açılmaz
        DeclareLaunchArgument(
            'nesne', default_value='true',
            description='nesne_node (kutu/mesafe/takip) ac (true/false)'),
        DeclareLaunchArgument(
            'arac_olculeri', default_value='',
            description='Arac olculeri dosyasi (robotaksi_baslat/config/arac_olculeri.yaml); '
                        'bos = uygulama'),
        OpaqueFunction(function=_kur),
    ])
