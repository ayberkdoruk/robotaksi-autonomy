# shellcheck shell=bash
# =============================================================================
# Ortak ROS ortamı: masaüstü kısayollarıyla açılan tüm betikler bunu kullanır.
# (Kısayollar ~/.bashrc'yi okumadığı için ayarları burada tekrar veriyoruz.)
#
# Kullanım (başka bir paketteki betikten; çalışma alanının yeri sabit değildir):
#   k="$(dirname "$(readlink -f "$0")")"
#   while [ "$k" != / ] && [ ! -f "$k/install/setup.bash" ]; do k="$(dirname "$k")"; done
#   source "$k/install/robotaksi_baslat/share/robotaksi_baslat/scripts/ros_ortami.sh"
# =============================================================================

# Çalışma alanı kökü: bu dosyadan yukarı çıkıp install/setup.bash bulunan ilk klasör
# (dosya src/ altından da install/ altından da çağrılabilir).
CALISMA_ALANI="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
while [ "$CALISMA_ALANI" != / ] && [ ! -f "$CALISMA_ALANI/install/setup.bash" ]; do
    CALISMA_ALANI="$(dirname "$CALISMA_ALANI")"
done

source /opt/ros/jazzy/setup.bash
source "$CALISMA_ALANI/install/setup.bash"

# ~/.bashrc'deki ağ ayarlarının aynısı. setup.bash'lerden SONRA verilmeli,
# çünkü ROS'un setup.bash dosyası ROS_AUTOMATIC_DISCOVERY_RANGE'i eziyor.
export ROS_DOMAIN_ID=42                                   # Wi-Fi'deki diğer ROS bilgisayarlarından ayrı
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST            # sadece bu bilgisayar
export FASTRTPS_DEFAULT_PROFILES_FILE="$CALISMA_ALANI/install/robotaksi_baslat/share/robotaksi_baslat/config/fastdds_buyuk_mesaj.xml"  # 1.5 MB'lık mesajlar için

KAYIT_KLASORU="$HOME/robotaksi_kayitlar"

# LiDAR sürücüsü çalışıyor mu?
surucu_calisiyor_mu() {
    pgrep -f "rslidar_sdk/lib/rslidar_sdk/rslidar_sdk_node" > /dev/null
}

# Hata/bitiş mesajından sonra terminal hemen kapanmasın
bekle() {
    local mesaj="$1"
    [ -z "$mesaj" ] && mesaj="Pencereyi kapatmak icin Enter'a basin..."
    echo
    read -r -p "$mesaj"
}
