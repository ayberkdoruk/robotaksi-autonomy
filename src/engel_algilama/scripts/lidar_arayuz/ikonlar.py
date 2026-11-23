"""
Kodla çizilen simgeler.

Yazı tipindeki emoji/simge karakterleri her bilgisayarda aynı görünmüyor; bu yüzden
düğme simgeleri burada basit şekillerle çiziliyor ve temanın rengini alıyor.
ata() ile verilen simgeler tema değişince yenile() ile yeniden çizilir.
"""

import math
import weakref

from python_qt_binding.QtCore import QPointF, QRectF, Qt
from python_qt_binding.QtGui import QBrush, QColor, QIcon, QPainter, QPainterPath, QPen, QPixmap, QPolygonF

from . import tema

_atananlar = []  # (zayıf_referans(düğme), ad, renk_adı, boyut)


def _kalem(c, w):
    k = QPen(c, w)
    k.setCapStyle(Qt.RoundCap)
    k.setJoinStyle(Qt.RoundJoin)
    return k


def ciz(p, ad, s, c):
    """s x s karelik alana 'ad' simgesini c rengiyle çizer."""
    w = max(1.6, s / 10.0)
    p.setBrush(Qt.NoBrush)
    p.setPen(_kalem(c, w))
    if ad == "ev":
        yol = QPainterPath()
        yol.moveTo(s * .14, s * .50)
        yol.lineTo(s * .50, s * .17)
        yol.lineTo(s * .86, s * .50)
        p.drawPath(yol)
        p.drawPolyline(QPolygonF([QPointF(s * .25, s * .42), QPointF(s * .25, s * .84),
                                  QPointF(s * .75, s * .84), QPointF(s * .75, s * .42)]))
    elif ad == "oynat":
        p.setPen(Qt.NoPen)
        p.setBrush(c)
        p.drawPolygon(QPolygonF([QPointF(s * .30, s * .18), QPointF(s * .82, s * .50), QPointF(s * .30, s * .82)]))
    elif ad == "duraklat":
        p.setPen(Qt.NoPen)
        p.setBrush(c)
        p.drawRoundedRect(QRectF(s * .25, s * .2, s * .17, s * .6), 1.5, 1.5)
        p.drawRoundedRect(QRectF(s * .58, s * .2, s * .17, s * .6), 1.5, 1.5)
    elif ad == "durdur":
        p.setPen(Qt.NoPen)
        p.setBrush(c)
        p.drawRoundedRect(QRectF(s * .24, s * .24, s * .52, s * .52), 2, 2)
    elif ad == "kayit":
        p.setPen(Qt.NoPen)
        p.setBrush(c)
        p.drawEllipse(QPointF(s * .5, s * .5), s * .28, s * .28)
    elif ad == "ileri":
        p.setPen(Qt.NoPen)
        p.setBrush(c)
        p.drawPolygon(QPolygonF([QPointF(s * .20, s * .20), QPointF(s * .62, s * .50), QPointF(s * .20, s * .80)]))
        p.drawRoundedRect(QRectF(s * .66, s * .20, s * .14, s * .60), 1.5, 1.5)
    elif ad == "ayar":
        p.drawEllipse(QPointF(s * .5, s * .5), s * .17, s * .17)
        for i in range(8):
            a = i * math.pi / 4
            p.drawLine(QPointF(s * .5 + math.cos(a) * s * .27, s * .5 + math.sin(a) * s * .27),
                       QPointF(s * .5 + math.cos(a) * s * .40, s * .5 + math.sin(a) * s * .40))
    elif ad == "klasor":
        yol = QPainterPath()
        yol.moveTo(s * .14, s * .30)
        yol.lineTo(s * .40, s * .30)
        yol.lineTo(s * .48, s * .38)
        yol.lineTo(s * .86, s * .38)
        yol.lineTo(s * .86, s * .80)
        yol.lineTo(s * .14, s * .80)
        yol.closeSubpath()
        p.drawPath(yol)
    elif ad == "sil":
        p.drawLine(QPointF(s * .2, s * .28), QPointF(s * .8, s * .28))
        p.drawLine(QPointF(s * .42, s * .18), QPointF(s * .58, s * .18))
        p.drawPolyline(QPolygonF([QPointF(s * .28, s * .32), QPointF(s * .33, s * .84),
                                  QPointF(s * .67, s * .84), QPointF(s * .72, s * .32)]))
    elif ad == "duzenle":
        p.drawLine(QPointF(s * .24, s * .76), QPointF(s * .72, s * .28))
        p.drawLine(QPointF(s * .62, s * .20), QPointF(s * .80, s * .38))
        p.drawLine(QPointF(s * .18, s * .84), QPointF(s * .30, s * .80))
    elif ad == "yenile":
        p.drawArc(QRectF(s * .2, s * .2, s * .6, s * .6), 60 * 16, 280 * 16)
        uc = QPointF(s * .5 + math.cos(math.radians(60)) * s * .3, s * .5 - math.sin(math.radians(60)) * s * .3)
        p.drawLine(uc, uc + QPointF(s * .02, s * .18))
        p.drawLine(uc, uc + QPointF(-s * .17, s * .03))
    elif ad == "ay":
        yol = QPainterPath()
        yol.addEllipse(QPointF(s * .5, s * .5), s * .32, s * .32)
        kes = QPainterPath()
        kes.addEllipse(QPointF(s * .66, s * .36), s * .28, s * .28)
        p.setPen(Qt.NoPen)
        p.setBrush(c)
        p.drawPath(yol.subtracted(kes))
    elif ad == "gunes":
        p.drawEllipse(QPointF(s * .5, s * .5), s * .16, s * .16)
        for i in range(8):
            a = i * math.pi / 4
            p.drawLine(QPointF(s * .5 + math.cos(a) * s * .28, s * .5 + math.sin(a) * s * .28),
                       QPointF(s * .5 + math.cos(a) * s * .38, s * .5 + math.sin(a) * s * .38))
    elif ad == "goz":
        yol = QPainterPath()
        yol.moveTo(s * .1, s * .5)
        yol.quadTo(s * .5, s * .12, s * .9, s * .5)
        yol.quadTo(s * .5, s * .88, s * .1, s * .5)
        p.drawPath(yol)
        p.setBrush(c)
        p.drawEllipse(QPointF(s * .5, s * .5), s * .11, s * .11)
    elif ad == "kaydet":
        p.drawRoundedRect(QRectF(s * .2, s * .2, s * .6, s * .6), 2, 2)
        p.drawLine(QPointF(s * .35, s * .2), QPointF(s * .35, s * .38))
        p.drawLine(QPointF(s * .35, s * .38), QPointF(s * .62, s * .38))
        p.drawLine(QPointF(s * .62, s * .38), QPointF(s * .62, s * .2))
    elif ad == "yukle":
        p.drawLine(QPointF(s * .5, s * .16), QPointF(s * .5, s * .62))
        p.drawPolyline(QPolygonF([QPointF(s * .32, s * .45), QPointF(s * .5, s * .63), QPointF(s * .68, s * .45)]))
        p.drawPolyline(QPolygonF([QPointF(s * .2, s * .66), QPointF(s * .2, s * .82),
                                  QPointF(s * .8, s * .82), QPointF(s * .8, s * .66)]))
    elif ad == "menu":
        p.setPen(Qt.NoPen)
        p.setBrush(c)
        for x in (.25, .5, .75):
            p.drawEllipse(QPointF(s * x, s * .5), s * .07, s * .07)
    elif ad == "kapat":
        p.drawLine(QPointF(s * .28, s * .28), QPointF(s * .72, s * .72))
        p.drawLine(QPointF(s * .72, s * .28), QPointF(s * .28, s * .72))
    elif ad == "cikti":
        p.drawRoundedRect(QRectF(s * .14, s * .2, s * .72, s * .6), 2, 2)
        p.drawPolyline(QPolygonF([QPointF(s * .28, s * .38), QPointF(s * .40, s * .50), QPointF(s * .28, s * .62)]))
        p.drawLine(QPointF(s * .48, s * .62), QPointF(s * .66, s * .62))
    elif ad == "yukari":
        p.drawPolyline(QPolygonF([QPointF(s * .25, s * .62), QPointF(s * .5, s * .38), QPointF(s * .75, s * .62)]))
    elif ad == "asagi":
        p.drawPolyline(QPolygonF([QPointF(s * .25, s * .38), QPointF(s * .5, s * .62), QPointF(s * .75, s * .38)]))
    elif ad == "uyari":
        p.drawPolygon(QPolygonF([QPointF(s * .5, s * .14), QPointF(s * .88, s * .82), QPointF(s * .12, s * .82)]))
        p.drawLine(QPointF(s * .5, s * .40), QPointF(s * .5, s * .58))
        p.drawPoint(QPointF(s * .5, s * .70))
    elif ad == "bilgi":
        p.drawEllipse(QPointF(s * .5, s * .5), s * .36, s * .36)
        p.drawLine(QPointF(s * .5, s * .46), QPointF(s * .5, s * .68))
        p.drawPoint(QPointF(s * .5, s * .33))
    elif ad == "tamam":
        p.drawPolyline(QPolygonF([QPointF(s * .22, s * .52), QPointF(s * .42, s * .72), QPointF(s * .78, s * .30)]))
    elif ad == "kutu":
        p.drawRect(QRectF(s * .22, s * .22, s * .56, s * .56))
        p.drawLine(QPointF(s * .22, s * .22), QPointF(s * .36, s * .12))
        p.drawLine(QPointF(s * .78, s * .22), QPointF(s * .9, s * .12))
        p.drawLine(QPointF(s * .36, s * .12), QPointF(s * .9, s * .12))


def ikon(ad, renk_adi="yazi", boyut=18):
    dpr = 2
    pm = QPixmap(boyut * dpr, boyut * dpr)
    pm.fill(Qt.transparent)
    pm.setDevicePixelRatio(dpr)
    p = QPainter(pm)
    p.setRenderHint(QPainter.Antialiasing)
    ciz(p, ad, boyut, tema.renk(renk_adi))
    p.end()
    return QIcon(pm)


_nokta_onbellek = {}


def nokta(renk_adi, boyut=10):
    """Durum noktası (yeşil = çalışıyor, kırmızı = hata ...)."""
    anahtar = (tema.hex_(renk_adi), boyut)
    if anahtar not in _nokta_onbellek:
        _nokta_onbellek[anahtar] = _nokta_ciz(renk_adi, boyut)
    return _nokta_onbellek[anahtar]


def _nokta_ciz(renk_adi, boyut):
    dpr = 2
    pm = QPixmap(boyut * dpr, boyut * dpr)
    pm.fill(Qt.transparent)
    pm.setDevicePixelRatio(dpr)
    p = QPainter(pm)
    p.setRenderHint(QPainter.Antialiasing)
    p.setPen(Qt.NoPen)
    p.setBrush(tema.renk(renk_adi))
    p.drawEllipse(QRectF(1, 1, boyut - 2, boyut - 2))
    p.end()
    return QIcon(pm)


def ata(dugme, ad, renk_adi="yazi", boyut=18):
    """Düğmeye simge verir; tema değişince yenile() bu simgeyi yeniden çizer."""
    anahtar = (ad, renk_adi, boyut, tema.aktif_ad)
    if getattr(dugme, "_ikon_anahtari", None) == anahtar:
        return  # aynı simge zaten verilmiş (sayfalar bunu saniyede 10 kez çağırabilir)
    dugme._ikon_anahtari = anahtar
    dugme.setIcon(ikon(ad, renk_adi, boyut))
    for i, (ref, *_ ) in enumerate(_atananlar):
        if ref() is dugme:
            _atananlar[i] = (ref, ad, renk_adi, boyut)
            return
    _atananlar.append((weakref.ref(dugme), ad, renk_adi, boyut))


def yenile():
    canli = []
    for ref, ad, renk_adi, boyut in _atananlar:
        d = ref()
        if d is None:
            continue
        try:
            d.setIcon(ikon(ad, renk_adi, boyut))
            d._ikon_anahtari = (ad, renk_adi, boyut, tema.aktif_ad)
        except RuntimeError:  # Qt nesnesi silinmiş
            continue
        canli.append((ref, ad, renk_adi, boyut))
    _atananlar[:] = canli


# -----------------------------------------------------------------------------
# Ana ekran kartlarının büyük simgeleri
# -----------------------------------------------------------------------------
def kart_ikonu_ciz(p, r, tur):
    """r (QRectF) içine bölüm simgesini çizer: engel / izle / kayit / oynat."""
    cx, cy = r.center().x(), r.center().y()
    s = min(r.width(), r.height())
    p.save()
    p.setRenderHint(QPainter.Antialiasing)
    if tur == "engel":
        alt = QPointF(cx, cy + s * .30)
        R1, R2 = s * .62, s * .32
        kutu1 = QRectF(alt.x() - R1, alt.y() - R1, 2 * R1, 2 * R1)
        kutu2 = QRectF(alt.x() - R2, alt.y() - R2, 2 * R2, 2 * R2)
        p.setPen(Qt.NoPen)
        p.setBrush(tema.renk("dikkat", 230))
        p.drawPie(kutu1, 60 * 16, 60 * 16)
        p.setBrush(tema.renk("dur"))
        p.drawPie(kutu2, 60 * 16, 60 * 16)
        p.setBrush(tema.renk("kart_izle"))
        p.drawEllipse(alt, s * .06, s * .06)
    elif tur == "izle":
        p.setBrush(Qt.NoBrush)
        for i, rr in enumerate((.42, .30, .17)):
            p.setPen(_kalem(tema.renk("kart_izle", 110 + i * 60), max(2.0, s * .045)))
            p.drawEllipse(QPointF(cx, cy), s * rr, s * rr)
        p.setPen(_kalem(tema.renk("kart_izle"), max(2.0, s * .05)))
        p.drawLine(QPointF(cx, cy), QPointF(cx + s * .30, cy - s * .30))
        p.setPen(Qt.NoPen)
        p.setBrush(tema.renk("guvenli"))
        for dx, dy in ((.22, -.12), (-.28, -.18), (.08, .30), (-.12, .1)):
            p.drawEllipse(QPointF(cx + s * dx, cy + s * dy), s * .035, s * .035)
    elif tur == "kayit":
        p.setBrush(Qt.NoBrush)
        p.setPen(_kalem(tema.renk("kart_kayit", 120), max(2.0, s * .06)))
        p.drawEllipse(QPointF(cx, cy), s * .40, s * .40)
        p.setPen(Qt.NoPen)
        p.setBrush(tema.renk("kart_kayit"))
        p.drawEllipse(QPointF(cx, cy), s * .24, s * .24)
    elif tur == "oynat":
        p.setBrush(Qt.NoBrush)
        p.setPen(_kalem(tema.renk("kart_oynat", 130), max(2.0, s * .06)))
        p.drawEllipse(QPointF(cx, cy), s * .40, s * .40)
        p.setPen(Qt.NoPen)
        p.setBrush(tema.renk("kart_oynat"))
        p.drawPolygon(QPolygonF([QPointF(cx - s * .12, cy - s * .2), QPointF(cx + s * .22, cy),
                                 QPointF(cx - s * .12, cy + s * .2)]))
    p.restore()


def kart_ikonu(tur, boyut=26):
    """Bölüm simgesinin küçük hali (sayfa başlıkları için)."""
    dpr = 2
    pm = QPixmap(boyut * dpr, boyut * dpr)
    pm.fill(Qt.transparent)
    pm.setDevicePixelRatio(dpr)
    p = QPainter(pm)
    kart_ikonu_ciz(p, QRectF(0, 0, boyut, boyut), tur)
    p.end()
    return pm


def uygulama_ikonu():
    """Pencere simgesi: engel yelpazesi, koyu kare üstünde."""
    boyut = 64
    pm = QPixmap(boyut, boyut)
    pm.fill(Qt.transparent)
    p = QPainter(pm)
    p.setRenderHint(QPainter.Antialiasing)
    p.setPen(Qt.NoPen)
    p.setBrush(QBrush(QColor("#23262d")))
    p.drawRoundedRect(QRectF(2, 2, 60, 60), 12, 12)
    kart_ikonu_ciz(p, QRectF(8, 6, 48, 48), "engel")
    p.end()
    return QIcon(pm)
