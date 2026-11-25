"""Araç haberleşme katmanını başlatır: serial_comm_node (+ istenirse gamepad_node).

  ros2 launch arac_haberlesme arac.launch.py                 # Teensy + kumanda
  ros2 launch arac_haberlesme arac.launch.py kumanda:=false  # kumandasız
  ros2 launch arac_haberlesme arac.launch.py kuru:=true      # Teensy'ye YAZMA (masada deneme)

Sürüş yöntemleri araca /arac/komut (robotaksi_msgs/AracKomutu) ile konuşur.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('arac_haberlesme'), 'config', 'arac_params.yaml')

    kumanda_arg = DeclareLaunchArgument(
        'kumanda', default_value='true',
        description='gamepad_node ac (Logitech F710, LB = manuel/otonom)')
    kuru_arg = DeclareLaunchArgument(
        'kuru', default_value='false',
        description="true: Teensy'ye yazma, sadece /arac/gonderilen'e yayinla")

    serial = Node(
        package='arac_haberlesme',
        executable='serial_comm_node',
        name='serial_comm_node',
        output='screen',
        # 'true'/'false' metni ROS'ta bool olarak okunur
        parameters=[params, {'kuru_calisma': LaunchConfiguration('kuru')}],
    )

    gamepad = Node(
        package='arac_haberlesme',
        executable='gamepad_node',
        name='gamepad_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('kumanda')),
    )

    return LaunchDescription([kumanda_arg, kuru_arg, serial, gamepad])
