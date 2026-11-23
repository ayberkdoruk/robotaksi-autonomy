#!/bin/bash
# =============================================================================
# Robotaksi Kontrol Merkezi'ni açar. Masaüstündeki "Kontrol Merkezi" simgesi bunu
# çalıştırır; terminal penceresi açılmaz.
#
# Programın kendi çıktısı ~/.ros/kontrol_merkezi/kontrol_merkezi.log dosyasına yazılır.
# Program açılırken hata verirse son satırlar bir pencerede gösterilir.
# =============================================================================

# Ortak ROS ortamı (robotaksi_baslat paketinde). Çalışma alanı kökü: bu betikten yukarı
# çıkıp install/setup.bash bulunan ilk klasör (çalışma alanının yeri sabit değil).
k="$(dirname "$(readlink -f "$0")")"
while [ "$k" != / ] && [ ! -f "$k/install/setup.bash" ]; do k="$(dirname "$k")"; done
source "$k/install/robotaksi_baslat/share/robotaksi_baslat/scripts/ros_ortami.sh"

GUNLUK="$HOME/.ros/kontrol_merkezi/kontrol_merkezi.log"
mkdir -p "$(dirname "$GUNLUK")"

ros2 run kontrol_merkezi kontrol_merkezi "$@" > "$GUNLUK" 2>&1
kod=$?

if [ $kod -ne 0 ] && [ -n "$DISPLAY" ] && command -v zenity > /dev/null; then
    son=$(tail -n 15 "$GUNLUK" | sed 's/&/\&amp;/g; s/</\&lt;/g; s/>/\&gt;/g')
    zenity --error --title="Kontrol Merkezi" --width=640 \
        --text="Kontrol Merkezi beklenmedik şekilde kapandı (çıkış kodu $kod).\n\nSon satırlar:\n\n<tt>$son</tt>\n\nTamamı: $GUNLUK" \
        2> /dev/null
fi
exit $kod
