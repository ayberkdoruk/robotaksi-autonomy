#!/usr/bin/env python3
"""
engel_ayar_paneli — engel_node ve nesne_node ayarlarını değiştirmek için Türkçe,
açıklamalı pencere (rqt_reconfigure yerine). engel.launch.py ve sistem.launch.py açar.

Solda canlı karar, yukarıdan bakış çizimi ve nesne tablosu; sağda tüm ayarlar sekmelerde.
LiDAR Kontrol Merkezi'ndeki Engel Algılama sayfasıyla aynı parçadır (lidar_arayuz/panel.py).

Bu panel LiDAR'a veya sürücüsüne HİÇBİR ayar göndermez; sadece engel_node ve nesne_node
parametrelerini değiştirir.
"""

import os
import sys

for _yol in (os.path.dirname(os.path.abspath(__file__)), os.path.dirname(os.path.realpath(__file__))):
    if _yol not in sys.path:
        sys.path.insert(0, _yol)

from lidar_arayuz.panel import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
