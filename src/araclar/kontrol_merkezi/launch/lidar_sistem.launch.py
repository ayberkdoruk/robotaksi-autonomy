"""LiDAR sistemini tek komutla başlatır: LiDAR sürücüsü + RViz + engel_node + nesne_node + ayar paneli.

  ros2 launch kontrol_merkezi lidar_sistem.launch.py                 # canlı LiDAR
  ros2 launch kontrol_merkezi lidar_sistem.launch.py surucu:=false   # rosbag oynatırken
  ros2 launch kontrol_merkezi lidar_sistem.launch.py panel:=false    # ayar paneli olmadan

Masaüstündeki "Engel Algılama" kısayolu (engel_sistemi_baslat.sh) da bunu çalıştırır.

NOT: LiDAR sürücüsü, rslidar_sdk'nın kendi start.py dosyasındakiyle AYNI şekilde
başlatılır (kendi config.yaml dosyasını kullanır). Sürücünün hiçbir ayarı
burada değiştirilmez. Sadece RViz, bizim hazır ayarlı ekranımızla açılır.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    paket = get_package_share_directory('kontrol_merkezi')
    algilama = get_package_share_directory('lidar_algilama')

    # Kayıt (rosbag) oynatırken sürücü açılmamalı: ikisi aynı topic'e yayın yapar
    surucu_arg = DeclareLaunchArgument(
        'surucu', default_value='true',
        description='LiDAR surucusunu ac (rosbag oynatirken false)')

    panel_arg = DeclareLaunchArgument(
        'panel', default_value='true', description='Ayar panelini de ac (true/false)')

    # 1) LiDAR sürücüsü — rslidar_sdk/launch/start.py ile birebir aynı ayarlar
    surucu = Node(
        namespace='rslidar_sdk',
        package='rslidar_sdk',
        executable='rslidar_sdk_node',
        output='screen',
        parameters=[{'config_path': ''}],  # boş = sürücünün kendi config.yaml'ı
        condition=IfCondition(LaunchConfiguration('surucu')),
    )

    # 2) RViz — ham bulut + filtrelenmiş noktalar + yelpaze/karar görselleri hazır ekli
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(paket, 'rviz', 'lidar.rviz')],
    )

    # 3) engel_node + nesne_node (lidar_algilama paketinden)
    dugumler = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(algilama, 'launch', 'lidar_algilama.launch.py')))

    # 4) Ayar paneli
    ayar_paneli = Node(
        package='kontrol_merkezi',
        executable='ayar_paneli',
        name='ayar_paneli',
        parameters=[{'hedef_node': 'engel_node'}],
        condition=IfCondition(LaunchConfiguration('panel')),
    )

    return LaunchDescription([surucu_arg, panel_arg, surucu, rviz, dugumler, ayar_paneli])
