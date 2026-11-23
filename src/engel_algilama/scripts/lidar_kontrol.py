#!/usr/bin/env python3
"""
LiDAR Kontrol Merkezi — her şey tek pencerede.

Ana ekrandan bir bölüm seçin: Engel Algılama, LiDAR'ı İzle, Kayıt Al, Kayıt Oynat.
Gereken programlar (LiDAR sürücüsü, engel_node, nesne_node, RViz, kayıt) arkada
kendiliğinden açılır; terminal açmanıza ya da komut yazmanıza gerek yoktur.

  ros2 run engel_algilama lidar_kontrol

Masaüstü simgesi lidar_kontrol_baslat.sh üzerinden bunu açar.
Kodun kendisi lidar_arayuz/ paketindedir.
"""

import os
import sys

# lidar_arayuz paketi bu dosyanın yanında (kurulumda lib/engel_algilama/, kaynakta scripts/)
for _yol in (os.path.dirname(os.path.abspath(__file__)), os.path.dirname(os.path.realpath(__file__))):
    if _yol not in sys.path:
        sys.path.insert(0, _yol)

from lidar_arayuz.uygulama import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
