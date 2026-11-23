#!/bin/bash
# =============================================================================
# Kaydedilmiş LiDAR verisini oynatır: RViz + engel_node + ayar paneli açılır,
# LiDAR bağlıymış gibi çalışır. LiDAR'ın takılı olmasına gerek YOKTUR.
#
# Kullanım:
#   lidar_oynat.sh                       -> kayıt seçme penceresi açılır
#   lidar_oynat.sh ~/lidar_kayitlar/XXX  -> o kaydı oynatır
#   lidar_oynat.sh ~/lidar_kayitlar/XXX 0.5  -> yarı hızda oynatır
#
# Kayıt sonsuz döngüde oynar. Durdurmak için bu terminalde Ctrl+C.
# =============================================================================

# Ortak ROS ortamı (robotaksi_baslat paketinde). Çalışma alanı kökü: bu betikten yukarı
# çıkıp install/setup.bash bulunan ilk klasör (çalışma alanının yeri sabit değil).
k="$(dirname "$(readlink -f "$0")")"
while [ "$k" != / ] && [ ! -f "$k/install/setup.bash" ]; do k="$(dirname "$k")"; done
source "$k/install/robotaksi_baslat/share/robotaksi_baslat/scripts/ros_ortami.sh"

echo "=============================================="
echo "   LiDAR KAYDI OYNATMA"
echo "=============================================="

# --- 1) Sürücü açıksa oynatma: ikisi aynı topic'e yayın yapıp veriyi karıştırır ---
if surucu_calisiyor_mu; then
    echo
    echo "HATA: LiDAR surucusu su an acik."
    echo "Kayit ve canli LiDAR ayni anda /rslidar_points'e yayin yapar, veri karisir."
    echo "Once canli sistemin terminalini kapatin, sonra tekrar deneyin."
    bekle
    exit 1
fi

# --- 2) Hangi kayıt? ---
kayit="$1"
if [ -z "$kayit" ]; then
    mkdir -p "$KAYIT_KLASORU"
    if [ -n "$DISPLAY" ] && command -v zenity > /dev/null; then
        kayit=$(zenity --file-selection --directory --title="Oynatilacak kaydi (klasor) secin" \
                --filename="$KAYIT_KLASORU/" 2>/dev/null)
        [ $? -ne 0 ] && { echo "Iptal edildi."; exit 0; }
    else
        echo "Kayitlar:"; ls -1 "$KAYIT_KLASORU"
        read -r -p "Kayit klasoru: " kayit
    fi
fi
if [ ! -f "$kayit/metadata.yaml" ]; then
    echo
    echo "HATA: '$kayit' gecerli bir rosbag kaydi degil (icinde metadata.yaml yok)."
    echo "~/lidar_kayitlar/ icindeki TARIH_SAAT_isim klasorlerinden birini secin."
    bekle
    exit 1
fi

hiz="${2:-1.0}"
echo
ros2 bag info "$kayit" 2>/dev/null | grep -E "Duration|Messages|Bag size"
echo
echo "Kayit : $kayit"
echo "Hiz   : ${hiz}x (sonsuz dongu)"
echo "DURDURMAK ICIN: Ctrl+C"
echo

# --- 3) RViz + engel_node + panel (sürücü olmadan) ---
ros2 launch engel_algilama sistem.launch.py surucu:=false &
ekran_pid=$!

temizle() {
    echo
    echo "Kapatiliyor..."
    kill -INT "$ekran_pid" 2> /dev/null
    wait "$ekran_pid" 2> /dev/null
}
trap 'true' INT

sleep 3  # RViz ve node'lar açılsın

# --- 4) Kaydı oynat ---
ros2 bag play "$kayit" --loop --rate "$hiz"

trap - INT
temizle
bekle "Oynatma bitti. Pencereyi kapatmak icin Enter'a basin..."
