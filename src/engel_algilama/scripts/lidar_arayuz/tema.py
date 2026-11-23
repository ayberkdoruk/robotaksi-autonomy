"""
Renkler ve Qt stil sayfası.

İki tema var: "koyu" (varsayılan; gece ve RViz'in yanında göz yormaz) ve "acik"
(güneşli havada okunur). Kendi çizimini yapan parçalar (karar kutusu, yukarıdan
bakış çizimi, simgeler) rengi her çizimde renk() ile buradan alır; böylece tema
değişince sadece yeniden çizilmeleri yeter.
"""

import os

from python_qt_binding.QtCore import QPointF, Qt
from python_qt_binding.QtGui import QColor, QPainter, QPalette, QPixmap, QPolygonF

# Karar ve etiket renkleri iki temada da aynı (RViz'deki renklerle uyumlu)
ORTAK = {
    "dur": "#e5484d", "dikkat": "#f2b01e", "guvenli": "#30a46c", "veri_yok": "#5b6570",
    "yaya": "#4ea4ff", "yapi": "#9aa5b1", "engel": "#ff8a3d", "kutulama": "#64bfff",
    "kart_engel": "#f2b01e", "kart_izle": "#3e8ee8", "kart_kayit": "#e5484d", "kart_oynat": "#30a46c",
}

KOYU = dict(ORTAK, **{
    "zemin": "#12161b", "panel": "#1b2027", "panel2": "#232a33", "panel3": "#2d3540",
    "cizgi": "#2c343e", "yazi": "#e6eaef", "soluk": "#95a1ae", "silik": "#66727f",
    "vurgu": "#3e8ee8", "vurgu_yazi": "#ffffff", "vurgu_zemin": "#1d3a5c",
    "kayit_zemin": "#4a1d20", "kayit_yazi": "#ffb3b5", "kayit_cizgi": "#6b2a2e",
    "basari": "#3fbf86", "uyari": "#f0b44c", "hata": "#f0595e", "bilgi": "#6cb2ff",
    "bant_hata": "#3d1b1e", "bant_uyari": "#3a2e14", "bant_bilgi": "#16304d", "bant_basari": "#163527",
    "cizim_zemin": "#0f1317", "cizim_cizgi": "#2b333d", "cizim_yazi": "#aeb8c3",
})

ACIK = dict(ORTAK, **{
    "zemin": "#eef1f5", "panel": "#ffffff", "panel2": "#f3f5f8", "panel3": "#e2e7ed",
    "cizgi": "#d4dae2", "yazi": "#18202a", "soluk": "#56626f", "silik": "#8a95a2",
    "vurgu": "#2f7bd8", "vurgu_yazi": "#ffffff", "vurgu_zemin": "#d8e7fb",
    "kayit_zemin": "#fde7e7", "kayit_yazi": "#b4232a", "kayit_cizgi": "#f1bfc1",
    "basari": "#1f8f5a", "uyari": "#a8670c", "hata": "#cf2f36", "bilgi": "#2f6fc0",
    "bant_hata": "#fde8e8", "bant_uyari": "#fdf2dc", "bant_bilgi": "#e3eefc", "bant_basari": "#e2f4ea",
    # Yukarıdan bakış çizimi açık temada da koyu zeminde kalır: sarı/kırmızı bölgeler
    # beyaz zeminde seçilmiyor.
    "cizim_zemin": "#1a2027", "cizim_cizgi": "#36404b", "cizim_yazi": "#b8c2cc",
})

TEMALAR = {"koyu": KOYU, "acik": ACIK}

_aktif = dict(KOYU)
aktif_ad = "koyu"

ONBELLEK = os.path.expanduser("~/.cache/lidar_kontrol")


def renk(ad, alfa=None):
    """Aktif temadaki rengi QColor olarak verir. alfa: 0-255."""
    c = QColor(_aktif.get(ad, ad))
    if alfa is not None:
        c.setAlpha(alfa)
    return c


def hex_(ad):
    return _aktif.get(ad, ad)


def karar_rengi(karar):
    return renk({"DUR": "dur", "DIKKAT": "dikkat", "GUVENLI": "guvenli"}.get(karar, "veri_yok"))


def ozellik(widget, ad, deger):
    """Stil sayfasında kullanılan bir özelliği değiştirip görünümü yeniler."""
    if widget.property(ad) == deger:
        return
    widget.setProperty(ad, deger)
    widget.style().unpolish(widget)
    widget.style().polish(widget)
    widget.update()


def _ok_resmi(ad, renk_hex):
    """Açılır listelerin ok simgesi (stil sayfası resim dosyası ister)."""
    os.makedirs(ONBELLEK, exist_ok=True)
    yol = os.path.join(ONBELLEK, f"ok_{ad}.png")
    pm = QPixmap(20, 20)
    pm.fill(Qt.transparent)
    p = QPainter(pm)
    p.setRenderHint(QPainter.Antialiasing)
    p.setPen(Qt.NoPen)
    p.setBrush(QColor(renk_hex))
    p.drawPolygon(QPolygonF([QPointF(5, 7.5), QPointF(15, 7.5), QPointF(10, 13.5)]))
    p.end()
    pm.save(yol)
    return yol


def _stil(r, ok):
    return f"""
    QWidget {{ color: {r['yazi']}; font-size: 10pt; }}
    QMainWindow, QDialog, QWidget#sayfa {{ background: {r['zemin']}; }}
    QToolTip {{ background: {r['panel2']}; color: {r['yazi']}; border: 1px solid {r['cizgi']}; padding: 5px; }}

    QFrame[rol="kart"] {{ background: {r['panel']}; border: 1px solid {r['cizgi']}; border-radius: 10px; }}
    QFrame[rol="ustcubuk"] {{ background: {r['panel']}; border: none; border-bottom: 1px solid {r['cizgi']}; }}
    QFrame[rol="altcubuk"] {{ background: {r['panel']}; border: none; border-top: 1px solid {r['cizgi']}; }}
    QFrame[rol="ayirac"] {{ background: {r['cizgi']}; border: none; max-width: 1px; min-width: 1px; }}
    QFrame[rol="ayar_satiri"] {{ background: transparent; border: none; border-bottom: 1px solid {r['cizgi']}; }}

    QLabel {{ background: transparent; }}
    QLabel[rol="baslik"] {{ font-size: 17pt; font-weight: 700; }}
    QLabel[rol="sayfa_basligi"] {{ font-size: 13pt; font-weight: 700; }}
    QLabel[rol="altbaslik"] {{ font-size: 11pt; font-weight: 600; }}
    QLabel[rol="bolum"] {{ color: {r['soluk']}; font-size: 8.5pt; font-weight: 700; }}
    QLabel[rol="soluk"] {{ color: {r['soluk']}; }}
    QLabel[rol="silik"] {{ color: {r['silik']}; font-size: 9pt; }}
    QLabel[rol="aciklama"] {{ color: {r['soluk']}; font-size: 9pt; }}
    QLabel[rol="buyuk_deger"] {{ font-size: 26pt; font-weight: 800; }}
    QLabel[rol="cip"] {{ background: {r['panel2']}; border: 1px solid {r['cizgi']}; border-radius: 11px;
                         padding: 3px 10px; color: {r['yazi']}; }}
    QLabel[rol="durum"][durum="ok"] {{ color: {r['basari']}; }}
    QLabel[rol="durum"][durum="uyari"] {{ color: {r['uyari']}; }}
    QLabel[rol="durum"][durum="hata"] {{ color: {r['hata']}; }}
    QLabel[rol="durum"][durum="bilgi"] {{ color: {r['soluk']}; }}

    QPushButton, QToolButton {{ background: {r['panel2']}; border: 1px solid {r['cizgi']}; border-radius: 7px;
                                padding: 6px 12px; }}
    QToolButton {{ padding: 5px 8px; }}
    QPushButton:hover, QToolButton:hover {{ background: {r['panel3']}; }}
    QPushButton:pressed, QToolButton:pressed {{ background: {r['cizgi']}; }}
    QPushButton:checked, QToolButton:checked {{ background: {r['vurgu_zemin']}; border-color: {r['vurgu']}; }}
    QPushButton:disabled, QToolButton:disabled {{ color: {r['silik']}; background: {r['panel']}; }}
    QPushButton[tur="birincil"] {{ background: {r['vurgu']}; border-color: {r['vurgu']}; color: {r['vurgu_yazi']};
                                   font-weight: 600; }}
    QPushButton[tur="birincil"]:hover {{ background: {r['bilgi']}; }}
    QPushButton[tur="birincil"]:disabled {{ background: {r['panel3']}; border-color: {r['cizgi']}; color: {r['silik']}; }}
    QPushButton[tur="kayit"], QToolButton[tur="kayit"] {{ background: {r['kayit_zemin']}; border-color: {r['kayit_cizgi']};
                                color: {r['kayit_yazi']}; font-weight: 600; }}
    QPushButton[tur="kayit"]:disabled {{ background: {r['panel']}; border-color: {r['cizgi']}; color: {r['silik']}; }}
    QPushButton[tur="buyuk"] {{ font-size: 12pt; font-weight: 700; padding: 10px 20px; border-radius: 9px; }}
    QPushButton[tur="duz"], QToolButton[tur="duz"] {{ background: transparent; border: 1px solid transparent; }}
    QPushButton[tur="duz"]:hover, QToolButton[tur="duz"]:hover {{ background: {r['panel2']}; border-color: {r['cizgi']}; }}
    QToolButton::menu-indicator {{ image: none; width: 0; }}

    QLineEdit, QComboBox, QAbstractSpinBox {{ background: {r['panel2']}; border: 1px solid {r['cizgi']};
                                              border-radius: 6px; padding: 4px 8px; selection-background-color: {r['vurgu']}; }}
    QLineEdit:focus, QComboBox:focus, QAbstractSpinBox:focus {{ border-color: {r['vurgu']}; }}
    QLineEdit:disabled, QComboBox:disabled, QAbstractSpinBox:disabled {{ color: {r['silik']}; background: {r['panel']}; }}
    QAbstractSpinBox[hatali="true"] {{ border: 1px solid {r['hata']}; color: {r['hata']}; }}
    QAbstractSpinBox {{ font-weight: 700; }}
    QComboBox {{ padding-right: 24px; }}
    QComboBox::drop-down {{ border: none; width: 22px; }}
    QComboBox::down-arrow {{ image: url({ok}); width: 14px; height: 14px; }}
    QComboBox QAbstractItemView {{ background: {r['panel2']}; border: 1px solid {r['cizgi']};
                                   selection-background-color: {r['vurgu']}; selection-color: {r['vurgu_yazi']}; outline: none; }}

    QCheckBox {{ spacing: 8px; background: transparent; }}

    QTabWidget::pane {{ border: none; }}
    QTabBar {{ background: transparent; }}
    QTabBar::tab {{ background: transparent; color: {r['soluk']}; padding: 7px 11px; border: none;
                    border-bottom: 2px solid transparent; }}
    QTabBar::tab:selected {{ color: {r['yazi']}; border-bottom: 2px solid {r['vurgu']}; font-weight: 600; }}
    QTabBar::tab:hover {{ color: {r['yazi']}; }}
    QTabBar::scroller {{ width: 22px; }}

    QSlider::groove:horizontal {{ height: 4px; background: {r['panel3']}; border-radius: 2px; }}
    QSlider::sub-page:horizontal {{ background: {r['vurgu']}; border-radius: 2px; }}
    QSlider::handle:horizontal {{ background: {r['vurgu_yazi']}; border: 2px solid {r['vurgu']}; width: 12px; height: 12px;
                                  margin: -6px 0; border-radius: 8px; }}
    QSlider::sub-page:horizontal:disabled {{ background: {r['silik']}; }}
    QSlider::handle:horizontal:disabled {{ border-color: {r['silik']}; background: {r['panel3']}; }}

    QScrollArea {{ background: transparent; border: none; }}
    QScrollArea > QWidget > QWidget {{ background: transparent; }}
    QScrollBar:vertical {{ background: transparent; width: 10px; margin: 0; }}
    QScrollBar:horizontal {{ background: transparent; height: 10px; margin: 0; }}
    QScrollBar::handle:vertical {{ background: {r['panel3']}; border-radius: 5px; min-height: 30px; }}
    QScrollBar::handle:horizontal {{ background: {r['panel3']}; border-radius: 5px; min-width: 30px; }}
    QScrollBar::add-line, QScrollBar::sub-line {{ width: 0; height: 0; }}
    QScrollBar::add-page, QScrollBar::sub-page {{ background: transparent; }}

    QTableWidget {{ background: {r['panel']}; alternate-background-color: {r['panel2']}; border: none;
                    gridline-color: {r['cizgi']}; selection-background-color: {r['vurgu_zemin']};
                    selection-color: {r['yazi']}; }}
    QHeaderView::section {{ background: {r['panel']}; color: {r['soluk']}; border: none;
                            border-bottom: 1px solid {r['cizgi']}; padding: 5px 6px; font-weight: 600; }}
    QTableCornerButton::section {{ background: {r['panel']}; border: none; }}

    QPlainTextEdit {{ background: {r['cizim_zemin']}; color: {r['cizim_yazi']}; border: none;
                      font-family: "Ubuntu Mono", "DejaVu Sans Mono", monospace; font-size: 9.5pt; }}
    QSplitter::handle {{ background: {r['zemin']}; }}
    QMenu {{ background: {r['panel2']}; border: 1px solid {r['cizgi']}; padding: 4px; }}
    QMenu::item {{ padding: 6px 18px; border-radius: 5px; }}
    QMenu::item:selected {{ background: {r['vurgu']}; color: {r['vurgu_yazi']}; }}
    QMenu::separator {{ height: 1px; background: {r['cizgi']}; margin: 4px 6px; }}
    QGroupBox {{ border: none; }}
    """


def uygula(uygulama, ad):
    """Temayı tüm uygulamaya uygular (Fusion stili + renk paleti + stil sayfası)."""
    global aktif_ad
    ad = ad if ad in TEMALAR else "koyu"
    aktif_ad = ad
    _aktif.clear()
    _aktif.update(TEMALAR[ad])
    r = _aktif

    uygulama.setStyle("Fusion")
    pal = QPalette()
    pal.setColor(QPalette.Window, QColor(r["zemin"]))
    pal.setColor(QPalette.WindowText, QColor(r["yazi"]))
    pal.setColor(QPalette.Base, QColor(r["panel2"]))
    pal.setColor(QPalette.AlternateBase, QColor(r["panel"]))
    pal.setColor(QPalette.Text, QColor(r["yazi"]))
    pal.setColor(QPalette.Button, QColor(r["panel2"]))
    pal.setColor(QPalette.ButtonText, QColor(r["yazi"]))
    pal.setColor(QPalette.Highlight, QColor(r["vurgu"]))
    pal.setColor(QPalette.HighlightedText, QColor(r["vurgu_yazi"]))
    pal.setColor(QPalette.ToolTipBase, QColor(r["panel2"]))
    pal.setColor(QPalette.ToolTipText, QColor(r["yazi"]))
    pal.setColor(QPalette.PlaceholderText, QColor(r["silik"]))
    pal.setColor(QPalette.Link, QColor(r["vurgu"]))
    for grup_rol in (QPalette.WindowText, QPalette.Text, QPalette.ButtonText):
        pal.setColor(QPalette.Disabled, grup_rol, QColor(r["silik"]))
    uygulama.setPalette(pal)
    uygulama.setStyleSheet(_stil(r, _ok_resmi(ad, r["soluk"])))
