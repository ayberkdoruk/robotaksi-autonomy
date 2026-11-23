#!/usr/bin/env python3
"""
ayar_paneli — engel_node ve nesne_node ayarlarını değiştirmek için Türkçe,
açıklamalı pencere (rqt_reconfigure yerine). lidar_sistem.launch.py açar.

Solda canlı karar, yukarıdan bakış çizimi ve nesne tablosu; sağda tüm ayarlar sekmelerde.
Kontrol Merkezi'ndeki Engel Algılama sayfasıyla aynı parçadır (arayuz/panel.py).

Bu panel LiDAR'a veya sürücüsüne HİÇBİR ayar göndermez; sadece engel_node ve nesne_node
parametrelerini değiştirir.
"""

import os
import sys

for _yol in (os.path.dirname(os.path.abspath(__file__)), os.path.dirname(os.path.realpath(__file__))):
    if _yol not in sys.path:
        sys.path.insert(0, _yol)

from arayuz.panel import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
