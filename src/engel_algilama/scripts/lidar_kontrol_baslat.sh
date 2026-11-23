#!/bin/bash
# =============================================================================
# LiDAR Kontrol Merkezi'ni açar. Masaüstündeki "LiDAR Kontrol" simgesi bunu
# çalıştırır; terminal penceresi açılmaz.
#
# Programın kendi çıktısı ~/.ros/engel_algilama/lidar_kontrol.log dosyasına yazılır.
# Program açılırken hata verirse son satırlar bir pencerede gösterilir.
# =============================================================================

# Ortak ROS ortamı (robotaksi_baslat paketinde). Çalışma alanı kökü: bu betikten yukarı
# çıkıp install/setup.bash bulunan ilk klasör (çalışma alanının yeri sabit değil).
k="$(dirname "$(readlink -f "$0")")"
while [ "$k" != / ] && [ ! -f "$k/install/setup.bash" ]; do k="$(dirname "$k")"; done
source "$k/install/robotaksi_baslat/share/robotaksi_baslat/scripts/ros_ortami.sh"

GUNLUK="$HOME/.ros/engel_algilama/lidar_kontrol.log"
mkdir -p "$(dirname "$GUNLUK")"

ros2 run engel_algilama lidar_kontrol "$@" > "$GUNLUK" 2>&1
kod=$?

if [ $kod -ne 0 ] && [ -n "$DISPLAY" ] && command -v zenity > /dev/null; then
    son=$(tail -n 15 "$GUNLUK" | sed 's/&/\&amp;/g; s/</\&lt;/g; s/>/\&gt;/g')
    zenity --error --title="LiDAR Kontrol" --width=640 \
        --text="LiDAR Kontrol beklenmedik şekilde kapandı (çıkış kodu $kod).\n\nSon satırlar:\n\n<tt>$son</tt>\n\nTamamı: $GUNLUK" \
        2> /dev/null
fi
exit $kod
