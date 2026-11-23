#!/bin/bash
# =============================================================================
# Dış depoları (sensör sürücüleri) çeker ve bizim yamalarımızı uygular.
# README → Kurulum → 2. adım. TEKRAR ÇALIŞTIRMAK GÜVENLİDİR:
#   - Yama zaten uygulanmışsa önce geri alınır, depo robotaksi.repos'taki sürüme
#     getirilir, sonra yama yeniden uygulanır. Böylece sabitlenen sürüm değişince
#     de bu betiği bir kez daha çalıştırmak yeterlidir.
#   - Dosyalarda elle değişiklik varsa (yama ne uygulanabiliyor ne geri alınabiliyorsa)
#     o yama atlanır ve uyarı verilir; hiçbir şey silinmez.
#
# Kullanım (çalışma alanının herhangi bir yerinden):
#   scripts/dis_depolari_cek.sh
# =============================================================================
set -euo pipefail

WS="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
cd "$WS"

# depo klasörü (src/ altına göre) : yama dosyası (src/surucular/yamalar/ altında)
YAMALAR=(
  "surucular/rslidar_sdk:rslidar_sdk_helios.patch"
)

if ! command -v vcs > /dev/null; then
  echo "HATA: 'vcs' bulunamadı. Kurun: sudo apt install python3-vcstool" >&2
  exit 1
fi

yama_uygulanmis_mi() {  # $1 = depo yolu, $2 = yama yolu
  git -C "$1" apply --reverse --check "$2" 2> /dev/null
}

# --- 1) Uygulanmış yamaları geri al (depo temiz sürüme dönebilsin) ---
for kayit in "${YAMALAR[@]}"; do
  depo="src/${kayit%%:*}"
  yama="$WS/src/surucular/yamalar/${kayit##*:}"
  if [ -d "$depo/.git" ] && yama_uygulanmis_mi "$depo" "$yama"; then
    echo ">>> Yama geri alınıyor: ${kayit##*:}"
    git -C "$depo" apply --reverse "$yama"
  fi
done

# --- 2) Depoları çek / sabitlenen sürüme getir ---
echo ">>> Dış depolar çekiliyor (robotaksi.repos)"
vcs import src < robotaksi.repos

echo ">>> rslidar_sdk alt modülleri"
git -C src/surucular/rslidar_sdk submodule update --init

# --- 3) Yamaları uygula ---
uyari=0
for kayit in "${YAMALAR[@]}"; do
  depo="src/${kayit%%:*}"
  yama="$WS/src/surucular/yamalar/${kayit##*:}"
  if yama_uygulanmis_mi "$depo" "$yama"; then
    echo ">>> Yama zaten uygulanmış: ${kayit##*:}"
  elif git -C "$depo" apply --check "$yama" 2> /dev/null; then
    echo ">>> Yama uygulanıyor: ${kayit##*:}"
    git -C "$depo" apply "$yama"
  else
    echo "UYARI: ${kayit##*:} uygulanamadı: $depo içinde elle değişiklik var." >&2
    echo "       Bakmak için: git -C $depo diff" >&2
    uyari=1
  fi
done

echo
echo "Tamam. Sıradaki adım: derleme (README → Kurulum → 3)."
exit $uyari
