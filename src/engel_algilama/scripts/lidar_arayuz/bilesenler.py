"""Sayfalarda ortak kullanılan arayüz parçaları."""

import math

from python_qt_binding.QtCore import QPointF, QRectF, Qt, Signal
from python_qt_binding.QtGui import QBrush, QFont, QPainter, QPainterPath, QPen, QPolygonF
from python_qt_binding.QtWidgets import (
    QAbstractItemView, QAbstractSpinBox, QApplication, QCheckBox, QComboBox, QDoubleSpinBox,
    QFrame, QHBoxLayout, QHeaderView, QLabel, QPlainTextEdit, QPushButton, QSizePolicy, QSlider,
    QSpinBox, QTableWidget, QTableWidgetItem, QTabWidget, QToolButton, QVBoxLayout, QWidget,
)

from . import ikonlar, tema
from .ayar_tanim import varsayilanlar
from .surecler import DURUM_ADLARI, DURUM_RENKLERI, KAPALI
from .yardimci import ETIKET_ADLARI, aciyi_anlat, noktanin_acisi


# -----------------------------------------------------------------------------
# Küçük yardımcılar
# -----------------------------------------------------------------------------
def kart(rol="kart"):
    f = QFrame()
    f.setProperty("rol", rol)
    return f


def etiket(metin="", rol=None, kaydir=False, secilebilir=False):
    e = QLabel(metin)
    if rol:
        e.setProperty("rol", rol)
    if kaydir:
        e.setWordWrap(True)
    if secilebilir:
        e.setTextInteractionFlags(Qt.TextSelectableByMouse)
    return e


def dugme(metin, ikon=None, tur=None, ipucu=None, tiklaninca=None, isaretlenebilir=False,
          ikon_rengi="yazi"):
    d = QPushButton(metin)
    if ikon:
        ikonlar.ata(d, ikon, ikon_rengi)
    if tur:
        d.setProperty("tur", tur)
    if ipucu:
        d.setToolTip(ipucu)
    if isaretlenebilir:
        d.setCheckable(True)
    if tiklaninca:
        d.clicked.connect(tiklaninca)
    d.setCursor(Qt.PointingHandCursor)
    return d


def arac_dugmesi(ikon, ipucu, tiklaninca=None, metin=None, tur="duz", boyut=16):
    d = QToolButton()
    ikonlar.ata(d, ikon, "yazi", boyut)
    d.setToolTip(ipucu)
    d.setProperty("tur", tur)
    if metin:
        d.setText(metin)
        d.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
    if tiklaninca:
        d.clicked.connect(tiklaninca)
    d.setCursor(Qt.PointingHandCursor)
    return d


def ayirac():
    f = QFrame()
    f.setProperty("rol", "ayirac")
    f.setFixedWidth(1)
    return f


def renkli_nokta(renk_adi, metin):
    """'● metin' — nokta rengi temadan."""
    return f"<span style='color:{tema.hex_(renk_adi)}'>●</span>&nbsp;&nbsp;{metin}"


class Cip(QLabel):
    """Üst çubuktaki küçük durum etiketi: '● LiDAR 10.0 Hz'."""

    def __init__(self, metin="", renk_adi="silik", ipucu=None):
        super().__init__()
        self.setProperty("rol", "cip")
        self.setTextFormat(Qt.RichText)
        self._son = None
        if ipucu:
            self.setToolTip(ipucu)
        self.ayarla(metin, renk_adi)

    def ayarla(self, metin, renk_adi="silik"):
        anahtar = (metin, renk_adi, tema.aktif_ad)
        if anahtar != self._son:
            self._son = anahtar
            self.setText(renkli_nokta(renk_adi, metin))


# Tekerlekle kaydırırken (sayfayı aşağı inerken) farenin altındaki çubuğun / kutunun
# yanlışlıkla değişmemesi için: odak yoksa tekerlek olayı sayfaya bırakılır.
class _OdakliTekerlek:
    def wheelEvent(self, olay):
        if self.hasFocus():
            super().wheelEvent(olay)
        else:
            olay.ignore()


class Cubuk(_OdakliTekerlek, QSlider):
    def __init__(self):
        super().__init__(Qt.Horizontal)
        self.setFocusPolicy(Qt.StrongFocus)


class OndalikKutu(_OdakliTekerlek, QDoubleSpinBox):
    def __init__(self):
        super().__init__()
        self.setFocusPolicy(Qt.StrongFocus)


class TamKutu(_OdakliTekerlek, QSpinBox):
    def __init__(self):
        super().__init__()
        self.setFocusPolicy(Qt.StrongFocus)


class Liste(_OdakliTekerlek, QComboBox):
    def __init__(self):
        super().__init__()
        self.setFocusPolicy(Qt.StrongFocus)


# -----------------------------------------------------------------------------
# Tek bir ayar: başlık + yazılabilir değer + kaydırma çubuğu + açıklama
# -----------------------------------------------------------------------------
class AyarSatiri(QFrame):
    degisti = Signal(str)  # kullanıcı değiştirince: ayarın anahtarı

    def __init__(self, ayar):
        super().__init__()
        self.setProperty("rol", "ayar_satiri")
        self.ayar = a = ayar

        self.baslik = etiket(a.baslik)
        self.baslik.setStyleSheet("font-weight: 600;")
        self.sifirla = arac_dugmesi("yenile", "", self.varsayilana, boyut=14)
        self.sifirla.setToolTip(f"Varsayılana döndür ({a.metin(a.varsayilan)})")
        self.sifirla.setVisible(False)

        ust = QHBoxLayout()
        ust.setSpacing(6)
        duzen = QVBoxLayout(self)
        duzen.setContentsMargins(2, 8, 2, 10)
        duzen.setSpacing(4)

        self.cubuk = None
        if a.tip == "mantik":
            self.kutu = QCheckBox()
            self.kutu.toggled.connect(lambda _: self._kullanici())
            ust.addWidget(self.kutu)
            ust.addWidget(self.baslik)
            ust.addStretch()
            ust.addWidget(self.sifirla)
            duzen.addLayout(ust)
        else:
            self.kutu = TamKutu() if a.tip == "tam" else OndalikKutu()
            if a.tip == "ondalik":
                self.kutu.setDecimals(a.basamak)
            self.kutu.setRange(a.alt, a.ust)
            self.kutu.setSingleStep(a.adim)
            bosluk = "" if a.birim == "°" else " "
            self.kutu.setSuffix(bosluk + a.birim)
            self.kutu.setButtonSymbols(QAbstractSpinBox.NoButtons)
            self.kutu.setKeyboardTracking(False)  # yazarken değil, Enter'a basınca uygula
            self.kutu.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self.kutu.setFixedWidth(108)
            self.kutu.setToolTip("Değeri yazıp Enter'a basabilirsiniz")
            self.kutu.valueChanged.connect(self._kutudan)

            self.cubuk = Cubuk()
            self.cubuk.setRange(0, int(round((a.ust - a.alt) / a.adim)))
            self.cubuk.valueChanged.connect(self._cubuktan)

            ust.addWidget(self.baslik, 1)
            ust.addWidget(self.sifirla)
            ust.addWidget(self.kutu)
            duzen.addLayout(ust)

            alt_y = etiket(a.metin(a.alt), "silik")
            ust_y = etiket(a.metin(a.ust), "silik")
            cubuk_satiri = QHBoxLayout()
            cubuk_satiri.setSpacing(8)
            cubuk_satiri.addWidget(alt_y)
            cubuk_satiri.addWidget(self.cubuk, 1)
            cubuk_satiri.addWidget(ust_y)
            duzen.addLayout(cubuk_satiri)

        self.aciklama = etiket(a.aciklama, "aciklama", kaydir=True)
        duzen.addWidget(self.aciklama)
        self.ayarla(a.varsayilan)

    # --- kullanıcı değiştirdi ---
    def _cubuktan(self, v):
        a = self.ayar
        deger = min(a.ust, a.alt + v * a.adim)
        self.kutu.blockSignals(True)
        self.kutu.setValue(deger)
        self.kutu.blockSignals(False)
        self._kullanici()

    def _kutudan(self, v):
        a = self.ayar
        self.cubuk.blockSignals(True)
        self.cubuk.setValue(int(round((v - a.alt) / a.adim)))
        self.cubuk.blockSignals(False)
        self._kullanici()

    def _kullanici(self):
        self._sifirla_guncelle()
        self.degisti.emit(self.ayar.anahtar)

    # --- dışarıdan ---
    def deger(self):
        a = self.ayar
        if a.tip == "mantik":
            return self.kutu.isChecked()
        if a.tip == "tam":
            return int(self.kutu.value())
        return round(float(self.kutu.value()), a.basamak + 2)

    def ayarla(self, v):
        """Değeri gösterir (node'dan okunan, dosyadan yüklenen). Sinyal göndermez."""
        a = self.ayar
        self.kutu.blockSignals(True)
        if a.tip == "mantik":
            self.kutu.setChecked(bool(v))
        else:
            self.kutu.setValue(v)
            self.cubuk.blockSignals(True)
            self.cubuk.setValue(int(round((float(v) - a.alt) / a.adim)))
            self.cubuk.blockSignals(False)
        self.kutu.blockSignals(False)
        self._sifirla_guncelle()

    def varsayilana(self):
        self.ayarla(self.ayar.varsayilan)
        self._kullanici()

    def hatali(self, evet):
        if self.ayar.tip != "mantik":
            tema.ozellik(self.kutu, "hatali", "true" if evet else "false")

    def _sifirla_guncelle(self):
        a = self.ayar
        v = self.deger()
        ayni = (bool(v) == bool(a.varsayilan)) if a.tip == "mantik" else abs(v - a.varsayilan) < 1e-9
        self.sifirla.setVisible(not ayni)


# -----------------------------------------------------------------------------
# Büyük karar kutusu: DUR / DİKKAT / GÜVENLİ + mesafe
# -----------------------------------------------------------------------------
class KararKutusu(QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumHeight(92)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.karar, self.ana, self.alt = None, "Bağlanıyor…", ""

    def ayarla(self, karar, ana, alt=""):
        if (karar, ana, alt) != (self.karar, self.ana, self.alt):
            self.karar, self.ana, self.alt = karar, ana, alt
            self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        r = QRectF(self.rect()).adjusted(1, 1, -1, -1)
        p.setPen(Qt.NoPen)
        p.setBrush(tema.karar_rengi(self.karar))
        p.drawRoundedRect(r, 12, 12)
        yazi = tema.renk("#2a1d00") if self.karar == "DIKKAT" else tema.renk("#ffffff")
        p.setPen(yazi)
        f = QFont(self.font())
        f.setPointSizeF(25 if self.karar else 15)
        f.setBold(True)
        p.setFont(f)
        ust = QRectF(r.x(), r.y() + 6, r.width(), r.height() * (0.62 if self.alt else 1.0) - 6)
        p.drawText(ust, Qt.AlignHCenter | Qt.AlignBottom if self.alt else Qt.AlignCenter, self.ana)
        if self.alt:
            f.setPointSizeF(11)
            f.setBold(False)
            p.setFont(f)
            p.drawText(QRectF(r.x(), r.y() + r.height() * 0.62, r.width(), r.height() * 0.34),
                       Qt.AlignHCenter | Qt.AlignTop, self.alt)


# -----------------------------------------------------------------------------
# Yukarıdan bakış çizimi
# Ekranda YUKARI = LiDAR'ın önü (+x), SOL = LiDAR'ın solu (+y).
# -----------------------------------------------------------------------------
def halka_araligi(metre):
    for a in (0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0):
        if metre / a <= 6:
            return a
    return 100.0


class YelpazeCizimi(QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(240, 220)
        self.ayar = varsayilanlar()
        self.karar = None
        self.engel = None      # (x, y) kararı veren nokta
        self.nesneler = []
        self.yakinlik = 1.0    # 1 = dış sarı çizgi kenara yakın; tekerlekle değişir
        self.setToolTip("Fare tekerleğiyle yakınlaştırıp uzaklaştırın. Çift tıklayınca ilk ölçeğe döner.")

    def guncelle(self, ayar, karar, engel=None, nesneler=()):
        self.ayar, self.karar, self.engel, self.nesneler = ayar, karar, engel, list(nesneler)
        self.update()

    def wheelEvent(self, olay):
        carpan = 1.15 if olay.angleDelta().y() < 0 else 1 / 1.15
        self.yakinlik = min(8.0, max(0.3, self.yakinlik * carpan))
        self.update()

    def mouseDoubleClickEvent(self, _):
        self.yakinlik = 1.0
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        cerceve = QPainterPath()
        cerceve.addRoundedRect(QRectF(0, 0, w, h), 10, 10)
        p.fillPath(cerceve, tema.renk("cizim_zemin"))
        p.setClipPath(cerceve)

        a = self.ayar
        merkez, genislik = a["merkez_aci"], a["gorus_genisligi"]
        dikkat = max(a["dikkat_mesafesi"], 0.1)
        dur = min(a["dur_mesafesi"], dikkat)
        yakin = min(a["min_mesafe"], dur)

        cx, cy = w / 2.0, h / 2.0 + 4
        R = max(40.0, min(w, h) / 2.0 - 30)
        olcek_m = dikkat * self.yakinlik    # R pikselin karşılığı (metre)
        ppm = R / olcek_m

        def ekran(x, y):
            return QPointF(cx - y * ppm, cy - x * ppm)

        kucuk = QFont(self.font())
        kucuk.setPointSizeF(8)
        p.setFont(kucuk)

        # Mesafe halkaları
        aralik = halka_araligi(olcek_m)
        cizgi = tema.renk("cizim_cizgi")
        r_m = aralik
        while r_m * ppm < max(w, h):
            r = r_m * ppm
            p.setPen(QPen(cizgi, 1, Qt.DashLine))
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(QPointF(cx, cy), r, r)
            if r_m <= olcek_m * 1.25:
                p.setPen(tema.renk("cizim_yazi", 150))
                p.drawText(QPointF(cx + r * 0.71 + 3, cy + r * 0.71 + 3), f"{r_m:g} m")
            r_m += aralik
        p.setPen(QPen(cizgi, 1))
        p.drawLine(QPointF(cx, 0), QPointF(cx, h))
        p.drawLine(QPointF(0, cy), QPointF(w, cy))

        # Qt açısı: 0 = saat 3 yönü, saat yönünün tersi. Bizim 0° = yukarı (ön), + = sol.
        def dilim(r_metre, dolgu, kalem, merk=merkez, gen=genislik):
            r = r_metre * ppm
            kutu = QRectF(cx - r, cy - r, 2 * r, 2 * r)
            p.setPen(kalem)
            p.setBrush(QBrush(dolgu) if dolgu is not None else Qt.NoBrush)
            if gen >= 360.0:
                p.drawEllipse(kutu)
            else:
                p.drawPie(kutu, int((90.0 + merk - gen / 2.0) * 16), int(gen * 16))

        # Kutulama alanı (nesne_node) — menzil kadar, mavi kesik çizgi
        kutulama_kalem = QPen(tema.renk("kutulama"), 1.5, Qt.DashLine)
        dilim(a["menzil"], tema.renk("kutulama", 14), kutulama_kalem,
              a["nesne_merkez_aci"], a["nesne_gorus_genisligi"])

        # Uyarı bölgeleri: DİKKAT (sarı), DUR (kırmızı), yok sayılan yakın bölge
        dilim(dikkat, tema.renk("dikkat", 60), QPen(tema.renk("dikkat"), 2))
        dilim(dur, tema.renk("dur", 105), QPen(tema.renk("dur"), 2))
        if yakin > 0:
            dilim(yakin, tema.renk("cizim_zemin"), QPen(tema.renk("cizim_yazi", 90), 1, Qt.DotLine))

        # Gövde kutusu: içindeki noktalar yok sayılır
        if a["govde_x_max"] > a["govde_x_min"] and a["govde_y_max"] > a["govde_y_min"]:
            govde = QRectF(ekran(a["govde_x_max"], a["govde_y_max"]),
                           ekran(a["govde_x_min"], a["govde_y_min"])).normalized()
            p.setPen(QPen(tema.renk("cizim_yazi", 140), 1, Qt.DashLine))
            p.setBrush(QBrush(tema.renk("cizim_yazi", 45), Qt.BDiagPattern))
            p.drawRect(govde)

        # Nesne kutuları (nesne_node)
        for n in self.nesneler:
            renk = tema.renk({"yaya_adayi": "yaya", "yapi": "yapi"}.get(n.etiket, "engel"))
            L, W, yon = n.boyut.x, n.boyut.y, n.yon
            cosy, siny = math.cos(yon), math.sin(yon)
            koseler = []
            for dl, dw in ((L / 2, W / 2), (L / 2, -W / 2), (-L / 2, -W / 2), (-L / 2, W / 2)):
                koseler.append(ekran(n.merkez.x + dl * cosy - dw * siny, n.merkez.y + dl * siny + dw * cosy))
            renk_dolgu = tema.renk(renk.name(), 60)
            p.setPen(QPen(renk, 2.2 if n.uyari_alaninda else 1.3))
            p.setBrush(renk_dolgu)
            p.drawPolygon(QPolygonF(koseler))
            p.setPen(renk)
            ust = min(koseler, key=lambda q: q.y())
            p.drawText(ust + QPointF(4, -4), f"#{n.id}")

        # LiDAR ve önünü gösteren ok
        p.setPen(QPen(tema.renk("#ffffff"), 2))
        p.drawLine(QPointF(cx, cy), QPointF(cx, cy - 24))
        p.drawLine(QPointF(cx, cy - 24), QPointF(cx - 5, cy - 16))
        p.drawLine(QPointF(cx, cy - 24), QPointF(cx + 5, cy - 16))
        p.setBrush(tema.renk("kart_izle"))
        p.drawEllipse(QPointF(cx, cy), 5.5, 5.5)

        # Kararı veren nokta
        if self.engel is not None:
            nokta = ekran(*self.engel)
            renk = tema.karar_rengi(self.karar)
            p.setPen(QPen(renk, 1.2, Qt.DashLine))
            p.drawLine(QPointF(cx, cy), nokta)
            p.setPen(QPen(tema.renk("#ffffff"), 2))
            p.setBrush(renk)
            p.drawEllipse(nokta, 7.5, 7.5)

        # Yön yazıları ve ölçek
        kalin = QFont(self.font())
        kalin.setPointSizeF(8.5)
        kalin.setBold(True)
        p.setFont(kalin)
        p.setPen(tema.renk("cizim_yazi"))
        p.drawText(QRectF(0, 4, w, 16), Qt.AlignHCenter, "ÖN")
        p.drawText(QRectF(0, h - 20, w, 16), Qt.AlignHCenter, "ARKA")
        p.drawText(QRectF(8, cy - 8, 40, 16), Qt.AlignLeft | Qt.AlignVCenter, "SOL")
        p.drawText(QRectF(w - 48, cy - 8, 40, 16), Qt.AlignRight | Qt.AlignVCenter, "SAĞ")
        p.setFont(kucuk)
        p.setPen(tema.renk("cizim_yazi", 170))
        p.drawText(QRectF(10, h - 20, w / 2, 16), Qt.AlignLeft, f"halkalar {aralik:g} m")


# -----------------------------------------------------------------------------
# Nesne tablosu (nesne_node'un /nesne/liste çıktısı)
# -----------------------------------------------------------------------------
class NesneTablosu(QTableWidget):
    SUTUNLAR = ["#", "Etiket", "Mesafe", "Yön", "Hız", "Yaklaşma", "Çarpışma", "Uyarı alanı"]

    def __init__(self):
        super().__init__(0, len(self.SUTUNLAR))
        self.setHorizontalHeaderLabels(self.SUTUNLAR)
        self.verticalHeader().setVisible(False)
        self.verticalHeader().setDefaultSectionSize(24)
        self.setSelectionMode(QAbstractItemView.NoSelection)
        self.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.setFocusPolicy(Qt.NoFocus)
        self.setAlternatingRowColors(True)
        self.setShowGrid(False)
        b = self.horizontalHeader()
        b.setSectionResizeMode(QHeaderView.ResizeToContents)
        b.setSectionResizeMode(1, QHeaderView.Stretch)
        b.setHighlightSections(False)
        self.setToolTip("Çarpışma: nesne aynı hızla yaklaşmaya devam ederse kaç saniye sonra "
                        "LiDAR'a ulaşır. Uyarı alanı: en yakın noktası engel yelpazesinin içinde mi.")

    def _hucre(self, satir, sutun, metin, renk_adi=None, kalin=False):
        oge = self.item(satir, sutun)
        if oge is None:
            oge = QTableWidgetItem()
            self.setItem(satir, sutun, oge)
        oge.setText(metin)
        oge.setForeground(tema.renk(renk_adi) if renk_adi else tema.renk("yazi"))
        if kalin != oge.font().bold():
            f = oge.font()
            f.setBold(kalin)
            oge.setFont(f)

    def guncelle(self, nesneler):
        nesneler = sorted(nesneler, key=lambda n: n.mesafe)[:40]
        self.setRowCount(len(nesneler))
        for i, n in enumerate(nesneler):
            renk = {"yaya_adayi": "yaya", "yapi": "yapi"}.get(n.etiket, "engel")
            hiz = math.hypot(n.hiz.x, n.hiz.y)
            t = n.carpisma_suresi
            t_renk = "hata" if 0 <= t < 2 else "uyari" if 0 <= t < 4 else None
            self._hucre(i, 0, str(n.id), "soluk")
            self._hucre(i, 1, ETIKET_ADLARI.get(n.etiket, n.etiket), renk, True)
            self._hucre(i, 2, f"{n.mesafe:.2f} m", None, True)
            self._hucre(i, 3, aciyi_anlat(noktanin_acisi(n.en_yakin_nokta.x, n.en_yakin_nokta.y)))
            self._hucre(i, 4, f"{hiz:.1f} m/s" if hiz >= 0.3 else "duruyor", None if hiz >= 0.3 else "silik")
            self._hucre(i, 5, f"{n.yaklasma_hizi:+.1f} m/s" if abs(n.yaklasma_hizi) >= 0.1 else "—",
                        "uyari" if n.yaklasma_hizi > 0.3 else "soluk")
            self._hucre(i, 6, f"{t:.1f} s" if t >= 0 else "—", t_renk, t_renk is not None)
            self._hucre(i, 7, "● içinde" if n.uyari_alaninda else "", "dikkat" if n.uyari_alaninda else None)


# -----------------------------------------------------------------------------
# Uyarı / bilgi bandı (sayfanın üstünde)
# -----------------------------------------------------------------------------
class Bant(QFrame):
    RENKLER = {"hata": ("bant_hata", "hata", "uyari"), "uyari": ("bant_uyari", "uyari", "uyari"),
               "bilgi": ("bant_bilgi", "bilgi", "bilgi"), "basari": ("bant_basari", "basari", "tamam")}

    def __init__(self):
        super().__init__()
        self._son = None
        self.duzen = QHBoxLayout(self)
        self.duzen.setContentsMargins(14, 9, 10, 9)
        self.duzen.setSpacing(10)
        self.simge = QLabel()
        self.metin = etiket(kaydir=True)
        self.metin.setTextFormat(Qt.RichText)
        self.duzen.addWidget(self.simge, 0, Qt.AlignTop)
        self.duzen.addWidget(self.metin, 1)
        self.dugmeler = []
        self.hide()

    def goster(self, tur, metin, dugmeler=()):
        """dugmeler: [(metin, fonksiyon), ...]"""
        anahtar = (tur, metin, tuple(d[0] for d in dugmeler), tema.aktif_ad)
        self._fonksiyonlar = [d[1] for d in dugmeler]
        if anahtar == self._son:
            self.show()
            return
        self._son = anahtar
        zemin, cizgi, simge = self.RENKLER.get(tur, self.RENKLER["bilgi"])
        self.setStyleSheet(f"Bant {{ background: {tema.hex_(zemin)}; border: 1px solid "
                           f"{tema.hex_(cizgi)}; border-radius: 9px; }}")
        self.simge.setPixmap(ikonlar.ikon(simge, cizgi, 18).pixmap(18, 18))
        self.metin.setText(metin)
        for d in self.dugmeler:
            self.duzen.removeWidget(d)
            d.setParent(None)
            d.deleteLater()
        self.dugmeler = []
        for i, (yazi, _) in enumerate(dugmeler):
            d = QPushButton(yazi)
            d.setCursor(Qt.PointingHandCursor)
            d.clicked.connect(lambda _=False, i=i: self._fonksiyonlar[i]())
            self.duzen.addWidget(d, 0, Qt.AlignVCenter)
            self.dugmeler.append(d)
        self.show()

    def gizle(self):
        self._son = None
        self.hide()


# -----------------------------------------------------------------------------
# Alt şerit: programların durumu + "Program çıktıları" düğmesi
# -----------------------------------------------------------------------------
class SurecSeridi(QFrame):
    tiklandi = Signal(str)

    def __init__(self):
        super().__init__()
        self.setProperty("rol", "altcubuk")
        self.duzen = QHBoxLayout(self)
        self.duzen.setContentsMargins(10, 4, 10, 4)
        self.duzen.setSpacing(4)
        self.dugmeler = {}
        self._son = {}
        self.duzen.addStretch()
        self.cikti = QToolButton()
        self.cikti.setText("Program çıktıları")
        self.cikti.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.cikti.setCheckable(True)
        self.cikti.setProperty("tur", "duz")
        self.cikti.setCursor(Qt.PointingHandCursor)
        ikonlar.ata(self.cikti, "cikti", "yazi", 16)
        self.duzen.addWidget(self.cikti)

    def ayarla(self, liste):
        """liste: [(anahtar, görünen ad), ...]"""
        if list(liste) == getattr(self, "_liste", None):
            return
        self._liste = list(liste)
        for d in self.dugmeler.values():
            self.duzen.removeWidget(d)
            d.setParent(None)
            d.deleteLater()
        self.dugmeler, self._son = {}, {}
        for i, (anahtar, ad) in enumerate(liste):
            d = QToolButton()
            d.setProperty("tur", "duz")
            d.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
            d.setCursor(Qt.PointingHandCursor)
            d.setToolTip(f"{ad}: çıktısını göster")
            d.clicked.connect(lambda _=False, k=anahtar: self.tiklandi.emit(k))
            d.setProperty("ad", ad)
            self.duzen.insertWidget(i, d)
            self.dugmeler[anahtar] = d

    def guncelle(self, yonetici, ekler=None):
        ekler = ekler or {}
        for anahtar, d in self.dugmeler.items():
            s = yonetici.surecler.get(anahtar)
            durum = s.durum if s else KAPALI
            metin = f"{d.property('ad')}  {ekler.get(anahtar) or DURUM_ADLARI[durum]}"
            anahtar_son = (durum, metin, tema.aktif_ad)
            if self._son.get(anahtar) != anahtar_son:
                self._son[anahtar] = anahtar_son
                d.setIcon(ikonlar.nokta(DURUM_RENKLERI[durum]))
                d.setText(metin)


class CiktiCekmecesi(QFrame):
    """Her programın terminal çıktısı ayrı sekmede."""

    kapat_istendi = Signal()

    def __init__(self, yonetici):
        super().__init__()
        self.setProperty("rol", "kart")
        self.yonetici = yonetici
        self.metinler = {}  # anahtar → [QPlainTextEdit, son satır no]
        duzen = QVBoxLayout(self)
        duzen.setContentsMargins(8, 6, 8, 8)
        duzen.setSpacing(4)
        ust = QHBoxLayout()
        ust.addWidget(etiket("PROGRAM ÇIKTILARI", "bolum"))
        ust.addStretch()
        ust.addWidget(arac_dugmesi("kaydet", "Seçili sekmedeki çıktıyı panoya kopyala", self._kopyala,
                                   metin="Kopyala"))
        ust.addWidget(arac_dugmesi("sil", "Seçili sekmeyi temizle", self._temizle, metin="Temizle"))
        ust.addWidget(arac_dugmesi("kapat", "Kapat", self.kapat_istendi.emit))
        duzen.addLayout(ust)
        self.sekmeler = QTabWidget()
        self.sekmeler.setDocumentMode(True)
        duzen.addWidget(self.sekmeler, 1)

    def ayarla(self, liste):
        """liste: [(anahtar, görünen ad), ...]"""
        istenen = [k for k, _ in liste]
        for k in list(self.metinler):
            if k not in istenen:
                i = self.sekmeler.indexOf(self.metinler[k][0])
                self.sekmeler.removeTab(i)
                self.metinler[k][0].deleteLater()
                del self.metinler[k]
        for anahtar, ad in liste:
            if anahtar not in self.metinler:
                m = QPlainTextEdit()
                m.setReadOnly(True)
                m.setMaximumBlockCount(4000)
                m.setLineWrapMode(QPlainTextEdit.NoWrap)
                self.metinler[anahtar] = [m, 0]
                self.sekmeler.addTab(m, ad)

    def sec(self, anahtar):
        if anahtar in self.metinler:
            self.sekmeler.setCurrentWidget(self.metinler[anahtar][0])

    def yenile(self):
        if not self.isVisible():
            return
        for anahtar, kayit in self.metinler.items():
            s = self.yonetici.surecler.get(anahtar)
            if s is None:
                continue
            satirlar, kayit[1] = s.yeni_satirlar(kayit[1])
            if satirlar:
                m = kayit[0]
                kaydirma = m.verticalScrollBar()
                sonda = kaydirma.value() >= kaydirma.maximum() - 4
                m.appendPlainText("\n".join(satirlar))
                if sonda:
                    kaydirma.setValue(kaydirma.maximum())

    def _secili(self):
        w = self.sekmeler.currentWidget()
        for anahtar, (m, _) in self.metinler.items():
            if m is w:
                return anahtar, m
        return None, None

    def _kopyala(self):
        _, m = self._secili()
        if m is not None:
            QApplication.clipboard().setText(m.toPlainText())

    def _temizle(self):
        anahtar, m = self._secili()
        if m is not None:
            m.clear()


# -----------------------------------------------------------------------------
# Ana ekrandaki durum kartı
# -----------------------------------------------------------------------------
class DurumKarti(QFrame):
    def __init__(self, baslik):
        super().__init__()
        self.setProperty("rol", "kart")
        self.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
        duzen = QVBoxLayout(self)
        duzen.setContentsMargins(14, 11, 14, 12)
        duzen.setSpacing(3)
        self.baslik = etiket(baslik.upper(), "bolum")
        self.deger = etiket("…")
        self.deger.setTextFormat(Qt.RichText)
        self.deger.setStyleSheet("font-size: 11.5pt; font-weight: 600;")
        self.ayrinti = etiket("", "silik", kaydir=True)
        duzen.addWidget(self.baslik)
        duzen.addWidget(self.deger)
        duzen.addWidget(self.ayrinti)
        duzen.addStretch()
        self._son = None

    def ayarla(self, deger, ayrinti="", renk_adi="silik"):
        anahtar = (deger, ayrinti, renk_adi, tema.aktif_ad)
        if anahtar != self._son:
            self._son = anahtar
            self.deger.setText(renkli_nokta(renk_adi, deger))
            self.ayrinti.setText(ayrinti)


def bos_alan(genislik=0, yukseklik=0):
    w = QWidget()
    w.setFixedSize(genislik, yukseklik)
    return w
