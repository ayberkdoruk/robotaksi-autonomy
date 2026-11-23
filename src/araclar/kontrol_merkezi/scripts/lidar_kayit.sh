#!/bin/bash
# =============================================================================
# LiDAR verisini (rosbag) kaydeder.
#
# Kullanım:
#   lidar_kayit.sh                    -> isim sorar, Ctrl+C'ye kadar kaydeder
#   lidar_kayit.sh oda_yurume         -> verilen isimle kaydeder
#   lidar_kayit.sh oda_yurume 60      -> 60 saniye kaydedip kendisi durur
#   SIKISTIRMA=yok lidar_kayit.sh ... -> sıkıştırmasız (bilgisayar kasarsa)
#
# Kayıtlar ~/robotaksi_kayitlar/ klasörüne "TARIH_SAAT_isim" adıyla gider.
# Önce LiDAR sürücüsü açık olmalı (masaüstündeki "Engel Algılama" simgesi).
# =============================================================================

# Ortak ROS ortamı (robotaksi_baslat paketinde). Çalışma alanı kökü: bu betikten yukarı
# çıkıp install/setup.bash bulunan ilk klasör (çalışma alanının yeri sabit değil).
k="$(dirname "$(readlink -f "$0")")"
while [ "$k" != / ] && [ ! -f "$k/install/setup.bash" ]; do k="$(dirname "$k")"; done
source "$k/install/robotaksi_baslat/share/robotaksi_baslat/scripts/ros_ortami.sh"

echo "=============================================="
echo "   LiDAR KAYIT"
echo "=============================================="

# --- 1) Sürücü açık mı? ---
if ! surucu_calisiyor_mu; then
    echo
    echo "HATA: LiDAR surucusu calismiyor."
    echo "Once masaustundeki 'Engel Algilama' simgesiyle sistemi acin,"
    echo "RViz'de noktalari gorunce bu kaydi tekrar baslatin."
    bekle
    exit 1
fi

# --- 2) Gerçekten veri geliyor mu? (kablo takılı mı, LiDAR açık mı) ---
echo "LiDAR verisi kontrol ediliyor..."
nokta=$(timeout 10 ros2 topic echo --once /rslidar_points --field width \
        --qos-reliability best_effort 2>/dev/null | head -1)
if [ -z "$nokta" ]; then
    echo
    echo "HATA: /rslidar_points topic'inden 10 saniyede veri gelmedi."
    echo "LiDAR'in gucunu ve ethernet kablosunu kontrol edin."
    bekle
    exit 1
fi
echo "Veri geliyor ✓"

# --- 3) Diskte yer var mı? (dakikada yaklaşık 0.5-0.9 GB) ---
mkdir -p "$KAYIT_KLASORU"
bos_gb=$(df --output=avail -BG "$KAYIT_KLASORU" | tail -1 | tr -dc '0-9')
echo "Diskte bos yer: ${bos_gb} GB  (1 dakikalik kayit ~0.5-0.9 GB)"
if [ "$bos_gb" -lt 5 ]; then
    echo "UYARI: Diskte 5 GB'tan az yer var, kayit yarida kalabilir!"
fi

# --- 4) Kayıt adı ---
isim="$1"
if [ -z "$isim" ]; then
    if [ -n "$DISPLAY" ] && command -v zenity > /dev/null; then
        isim=$(zenity --entry --title="LiDAR Kayit" \
               --text="Kayda bir isim verin (ne kaydettiginizi anlatsin):" \
               --entry-text="oda_yurume" 2>/dev/null)
        [ $? -ne 0 ] && { echo "Iptal edildi."; exit 0; }
    else
        read -r -p "Kayit adi (or. oda_yurume): " isim
    fi
fi
# Dosya adında sorun çıkaracak karakterleri temizle
isim=$(echo "${isim:-kayit}" | tr ' ' '_' | tr -cd '[:alnum:]_-')
yol="$KAYIT_KLASORU/$(date +%Y-%m-%d_%H-%M-%S)_${isim:-kayit}"

# --- 5) Sıkıştırma seçimi ---
# zstd_fast: hızlı sıkıştırma (dosya ~yarı boyutta, işlemciyi az yorar)
# fastwrite: sıkıştırmasız (en hafif; bilgisayar kasarsa SIKISTIRMA=yok ile kullanın)
if [ "$SIKISTIRMA" = "yok" ]; then
    profil="fastwrite"
else
    profil="zstd_fast"
fi

sure="$2"
echo
echo "----------------------------------------------"
echo " Kayit yeri : $yol"
echo " Kaydedilen : /rslidar_points (her turda ${nokta} sutun)"
echo " Sikistirma : $profil"
if [ -n "$sure" ]; then
    echo " Sure       : $sure saniye (sonra kendisi durur)"
else
    echo " DURDURMAK ICIN: Ctrl+C"
fi
echo "----------------------------------------------"
echo

# Ctrl+C'ye basınca betik de kapanmasın: kayıt düzgün bitsin, sonra özet gösterelim
trap 'true' INT

if [ -n "$sure" ]; then
    timeout -s INT "$sure" ros2 bag record /rslidar_points -o "$yol" \
        -s mcap --storage-preset-profile "$profil"
else
    ros2 bag record /rslidar_points -o "$yol" -s mcap --storage-preset-profile "$profil"
fi

trap - INT

# --- 6) Özet ---
echo
if [ -f "$yol/metadata.yaml" ]; then
    echo "=============================================="
    echo " KAYIT TAMAM ✓"
    echo "=============================================="
    ros2 bag info "$yol" 2>/dev/null | grep -E "Duration|Messages|Bag size"
    echo
    echo "Oynatmak icin masaustundeki 'LiDAR Kaydi Oynat' simgesini kullanin."
else
    echo "UYARI: Kayit dosyasi olusmadi ya da eksik kaldi: $yol"
fi
bekle
