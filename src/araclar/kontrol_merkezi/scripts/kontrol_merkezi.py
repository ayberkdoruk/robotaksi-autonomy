#!/usr/bin/env python3
"""
Robotaksi Kontrol Merkezi — her şey tek pencerede.

Ana ekrandan bir bölüm seçin: Engel Algılama, LiDAR'ı İzle, Kayıt Al, Kayıt Oynat.
Gereken programlar (LiDAR sürücüsü, engel_node, nesne_node, RViz, kayıt) arkada
kendiliğinden açılır; terminal açmanıza ya da komut yazmanıza gerek yoktur.

  ros2 run kontrol_merkezi kontrol_merkezi

Masaüstü simgesi kontrol_merkezi_baslat.sh üzerinden bunu açar.
Kodun kendisi arayuz/ paketindedir.
"""

import os
import sys

# arayuz paketi bu dosyanın yanında (kurulumda lib/kontrol_merkezi/, kaynakta scripts/)
for _yol in (os.path.dirname(os.path.abspath(__file__)), os.path.dirname(os.path.realpath(__file__))):
    if _yol not in sys.path:
        sys.path.insert(0, _yol)

from arayuz.uygulama import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
