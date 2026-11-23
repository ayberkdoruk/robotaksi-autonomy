"""Ekranda gösterilen metinler için küçük yardımcılar (açı, süre, boyut, tarih)."""

import math

AYLAR = ["Oca", "Şub", "Mar", "Nis", "May", "Haz", "Tem", "Ağu", "Eyl", "Eki", "Kas", "Ara"]

KARAR_ADLARI = {"DUR": "DUR", "DIKKAT": "DİKKAT", "GUVENLI": "GÜVENLİ"}
ETIKET_ADLARI = {"yaya_adayi": "yaya adayı", "yapi": "yapı", "engel": "engel"}


def aciyi_normalize_et(aci):
    """Açıyı -180..180 aralığına getirir."""
    return (aci + 180.0) % 360.0 - 180.0


def aciyi_anlat(aci):
    """-45 → 'sağ 45°', 30 → 'sol 30°', 0 → 'tam ön' gibi okunur metin."""
    a = aciyi_normalize_et(aci)
    if abs(a) < 0.5:
        return "tam ön"
    if abs(abs(a) - 180.0) < 0.5:
        return "tam arka"
    return f"sol {a:.0f}°" if a > 0 else f"sağ {-a:.0f}°"


def noktanin_acisi(x, y):
    """LiDAR'a göre (x ileri, y sol) noktanın yatay açısı, derece."""
    return math.degrees(math.atan2(y, x))


def saat_metni(saniye):
    """75 → '01:15', 3700 → '1:01:40'."""
    saniye = max(0, int(saniye))
    s, d = saniye % 60, (saniye // 60) % 60
    sa = saniye // 3600
    return f"{sa}:{d:02d}:{s:02d}" if sa else f"{d:02d}:{s:02d}"


def sure_metni(saniye):
    """72.4 → '1 dk 12 s', 8 → '8 s', 3900 → '1 sa 5 dk'."""
    saniye = max(0, int(round(saniye)))
    if saniye < 60:
        return f"{saniye} s"
    if saniye < 3600:
        d, s = divmod(saniye, 60)
        return f"{d} dk {s} s" if s else f"{d} dk"
    sa, kalan = divmod(saniye, 3600)
    return f"{sa} sa {kalan // 60} dk"


def boyut_metni(bayt):
    """Bayt → '640 MB' / '1.9 GB'."""
    if bayt is None:
        return "—"
    if bayt < 1024 ** 2:
        return f"{bayt / 1024:.0f} KB"
    if bayt < 1024 ** 3:
        return f"{bayt / 1024 ** 2:.0f} MB"
    return f"{bayt / 1024 ** 3:.1f} GB"


def tarih_metni(tarih):
    """datetime → '20 Kas 18:34'."""
    if tarih is None:
        return "—"
    return f"{tarih.day} {AYLAR[tarih.month - 1]} {tarih:%H:%M}"


def bin_metni(sayi):
    """43200 → '43 bin', 950 → '950'."""
    if sayi >= 1000:
        return f"{sayi / 1000:.0f} bin"
    return str(sayi)
