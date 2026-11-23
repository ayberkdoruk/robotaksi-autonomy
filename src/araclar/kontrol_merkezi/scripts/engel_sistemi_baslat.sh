#!/bin/bash
# =============================================================================
# Engel Algılama sistemini tek tıkla başlatır:
#   LiDAR sürücüsü + RViz + engel_node + ayar paneli
#
# Masaüstündeki "Engel Algılama" simgesi bu betiği bir terminal penceresinde
# çalıştırır. Kapatmak için: o terminalde Ctrl+C veya terminal penceresini kapatın.
# =============================================================================

# Ortak ROS ortamı (robotaksi_baslat paketinde). Çalışma alanı kökü: bu betikten yukarı
# çıkıp install/setup.bash bulunan ilk klasör (çalışma alanının yeri sabit değil).
k="$(dirname "$(readlink -f "$0")")"
while [ "$k" != / ] && [ ! -f "$k/install/setup.bash" ]; do k="$(dirname "$k")"; done
source "$k/install/robotaksi_baslat/share/robotaksi_baslat/scripts/ros_ortami.sh"

echo "=============================================="
echo "   ENGEL ALGILAMA SISTEMI BASLATILIYOR"
echo "=============================================="

# Sistem zaten açıksa ikinci kez açma: iki sürücü aynı LiDAR portunu kullanamaz
if surucu_calisiyor_mu; then
    echo
    echo "UYARI: LiDAR surucusu zaten calisiyor (sistem acik olabilir)."
    echo "Once acik olan pencereyi/terminali kapatin, sonra tekrar deneyin."
    bekle
    exit 1
fi

ros2 launch kontrol_merkezi lidar_sistem.launch.py

bekle "Sistem kapandi. Pencereyi kapatmak icin Enter'a basin..."
