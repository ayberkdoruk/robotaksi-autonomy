"""Aracı LiDAR ile sürmek için her şey: LiDAR sürücüsü + engel_node + nesne_node +
araç katmanı (serial_comm_node + gamepad_node) + lidar_surus.

  ros2 launch lidar_surus lidar_surus.launch.py                  # araçta, tam sistem
  ros2 launch lidar_surus lidar_surus.launch.py kuru:=true       # Teensy'ye YAZMA (masada)
  ros2 launch lidar_surus lidar_surus.launch.py kumanda:=false   # kumandasız
  ros2 launch lidar_surus lidar_surus.launch.py surucu:=false    # kayıt (rosbag) oynatırken
  ros2 launch lidar_surus lidar_surus.launch.py algilama:=false  # sürücü + algılama zaten açık
                                                                 # (Kontrol Merkezi / lidar_sistem)

Araç ölçüleri robotaksi_baslat/config/arac_olculeri.yaml'dan okunur; algılama geometrisi ve
DUR mesafeleri onlardan hesaplanır. Doldurulmamışsa araçta (kuru:=false) launch açılmaz.

Araç açılışta FRENDEDİR. Kalkmak için:   ros2 param set /lidar_surus surus_aktif true
Durdurmak için:                          ros2 param set /lidar_surus surus_aktif false
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

OLCULER = ('lidar_yukseklik', 'lidar_on_tampon', 'lidar_arka_tampon',
           'lidar_sol_kenar', 'lidar_sag_kenar')


def _evet(context, ad):
    return LaunchConfiguration(ad).perform(context).lower() == 'true'


def _kur(context):
    surus = get_package_share_directory('lidar_surus')
    algilama = get_package_share_directory('lidar_algilama')
    arac = get_package_share_directory('arac_haberlesme')
    eylemler = []

    # --- Araç ölçüleri ---
    olcu_yolu = LaunchConfiguration('olculer').perform(context)
    with open(olcu_yolu) as f:
        o = yaml.safe_load(f) or {}
    dolu = not all(float(o.get(ad, 0.0)) == 0.0 for ad in OLCULER)
    kuru = _evet(context, 'kuru')
    surus_parametreleri = {}
    if dolu:
        on = float(o['lidar_on_tampon'])
        dur = on + float(o.get('durma_boslugu', 2.0))
        # nesne mesafesi LiDAR'dan ölçülür; boşluklar ön tampondan
        surus_parametreleri = {
            'nesne_dur_mesafesi': round(dur, 2),
            'nesne_dikkat_mesafesi': round(max(on + float(o.get('dikkat_boslugu', 4.0)),
                                               dur + 0.5), 2),
        }
        eylemler.append(LogInfo(msg=f'Arac olculeri: {olcu_yolu}  -> lidar_surus '
                                    f'{surus_parametreleri}'))
    elif kuru:
        eylemler.append(LogInfo(
            msg=f'UYARI: {olcu_yolu} doldurulmamis. Masa (kuru) denemesi: kayitli ayarlarla devam.'))
    else:
        raise RuntimeError(
            f'\n\n  ARAC OLCULERI GIRILMEMIS: {olcu_yolu}\n'
            '  Aracta surmeden once LiDAR yuksekligi ve tampon / kenar mesafelerini yazin.\n'
            '  (Masada denemek icin: kuru:=true)\n')

    # --- LiDAR sürücüsü + algılama ---
    if _evet(context, 'algilama'):
        if _evet(context, 'surucu'):
            # kontrol_merkezi/lidar_sistem.launch.py ile aynı (sürücünün kendi config.yaml'ı)
            eylemler.append(Node(
                namespace='rslidar_sdk',
                package='rslidar_sdk',
                executable='rslidar_sdk_node',
                output='screen',
                parameters=[{'config_path': ''}],
            ))
        eylemler.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(algilama, 'launch', 'lidar_algilama.launch.py')),
            launch_arguments={'arac_olculeri': olcu_yolu if dolu else ''}.items(),
        ))
    elif dolu:
        eylemler.append(LogInfo(
            msg='UYARI: algilama:=false -> arac olculeri engel_node / nesne_node\'a '
                'UYGULANMADI (onlari acan program kendi ayarlarini kullanir).'))

    # --- Araç katmanı ---
    eylemler.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(arac, 'launch', 'arac.launch.py')),
        launch_arguments={
            'kumanda': LaunchConfiguration('kumanda').perform(context),
            'kuru': LaunchConfiguration('kuru').perform(context),
        }.items(),
    ))

    # --- Sürüş ---
    eylemler.append(Node(
        package='lidar_surus',
        executable='lidar_surus_node',
        name='lidar_surus',           # yaml'daki en üst anahtarla aynı olmalı
        output='screen',
        parameters=[os.path.join(surus, 'config', 'lidar_surus_params.yaml'),
                    surus_parametreleri],
    ))
    return eylemler


def generate_launch_description():
    varsayilan_olculer = os.path.join(
        get_package_share_directory('robotaksi_baslat'), 'config', 'arac_olculeri.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('surucu', default_value='true',
                              description='LiDAR surucusunu ac (rosbag oynatirken false)'),
        DeclareLaunchArgument('algilama', default_value='true',
                              description='Surucu + engel_node + nesne_node ac (zaten aciksa false)'),
        DeclareLaunchArgument('kumanda', default_value='true',
                              description='gamepad_node ac'),
        DeclareLaunchArgument('kuru', default_value='false',
                              description="true: Teensy'ye yazma (masada deneme)"),
        DeclareLaunchArgument('olculer', default_value=varsayilan_olculer,
                              description='Arac olculeri dosyasi'),
        OpaqueFunction(function=_kur),
    ])
