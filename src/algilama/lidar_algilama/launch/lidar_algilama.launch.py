"""LiDAR algılama düğümlerini (engel_node + nesne_node) başlatan launch dosyası.

Kullanım:
  ros2 launch lidar_algilama lidar_algilama.launch.py              # iki düğüm
  ros2 launch lidar_algilama lidar_algilama.launch.py nesne:=false # nesne_node'u açma

Ayar paneli bu pakette değil: sürücü + RViz + düğümler + panel için
  ros2 launch kontrol_merkezi lidar_sistem.launch.py

Parametreler şu sırayla yüklenir (sonraki öncekinin üstüne yazar):
  1) config/engel_params.yaml / config/nesne_params.yaml -> paketin varsayılan değerleri
  2) ~/.ros/kontrol_merkezi/son_ayarlar.yaml -> panelde en son uygulanan ayarlar (varsa)
     (dosyada "engel_node:" ve "nesne_node:" bölümleri ayrı ayrı durur)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SON_AYARLAR = os.path.expanduser('~/.ros/kontrol_merkezi/son_ayarlar.yaml')


def generate_launch_description():
    params_dosyasi = os.path.join(
        get_package_share_directory('lidar_algilama'), 'config', 'engel_params.yaml')

    parametreler = [params_dosyasi]
    if os.path.exists(SON_AYARLAR):
        parametreler.append(SON_AYARLAR)
        bilgi = LogInfo(msg=f'Kayitli ayarlar yukleniyor: {SON_AYARLAR}')
    else:
        bilgi = LogInfo(msg='Kayitli ayar yok, varsayilan degerler kullaniliyor.')

    nesne_parametreler = [os.path.join(
        get_package_share_directory('lidar_algilama'), 'config', 'nesne_params.yaml')]
    if os.path.exists(SON_AYARLAR):
        nesne_parametreler.append(SON_AYARLAR)  # dosyadaki "nesne_node:" bölümü kullanılır

    # "nesne:=false" yazılırsa nesne algılama (kutular) açılmaz
    nesne_arg = DeclareLaunchArgument(
        'nesne', default_value='true', description='nesne_node (kutu/mesafe/takip) ac (true/false)')

    nesne_node = Node(
        package='lidar_algilama',
        executable='nesne_node',
        name='nesne_node',          # nesne_params.yaml'daki en üst anahtarla aynı olmalı
        output='screen',
        # Panelde uygulanan son ayarlar (kutulama alanı) varsa varsayılanların üstüne yazar
        parameters=nesne_parametreler,
        condition=IfCondition(LaunchConfiguration('nesne')),
    )

    engel_node = Node(
        package='lidar_algilama',
        executable='engel_node',
        name='engel_node',          # yaml'daki en üst anahtarla aynı olmalı
        output='screen',            # terminalde canlı satırlar görünsün
        parameters=parametreler,
    )

    return LaunchDescription([nesne_arg, bilgi, engel_node, nesne_node])
