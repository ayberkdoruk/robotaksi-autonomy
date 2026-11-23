"""
Algılama görünümü: canlı karar, yukarıdan bakış çizimi, nesne tablosu ve yanında
ayar paneli. Engel Algılama ve Kayıt Oynat sayfaları ile tek başına çalışan ayar
paneli aynı parçayı kullanır.
"""

import time

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QHBoxLayout, QSplitter, QVBoxLayout, QWidget

from . import tema
from .ayar_paneli import AyarPaneli
from .ayar_tanim import ENGEL, NESNE
from .bilesenler import KararKutusu, NesneTablosu, YelpazeCizimi, etiket, kart
from .yardimci import KARAR_ADLARI, aciyi_anlat, noktanin_acisi


class AlgilamaGorunumu(QWidget):
    def __init__(self):
        super().__init__()
        self.karar = KararKutusu()
        self.neden = etiket(kaydir=True)
        self.neden.setTextFormat(Qt.RichText)
        self.neden.setAlignment(Qt.AlignCenter)
        self.neden.setMinimumHeight(38)

        self.cizim = YelpazeCizimi()
        self.bolge = etiket(kaydir=True)
        self.bolge.setTextFormat(Qt.RichText)
        cizim_karti = kart()
        c_duzen = QVBoxLayout(cizim_karti)
        c_duzen.setContentsMargins(8, 8, 8, 8)
        c_duzen.setSpacing(6)
        c_duzen.addWidget(self.cizim, 1)
        c_duzen.addWidget(self.bolge)

        nesne_karti = kart()
        n_duzen = QVBoxLayout(nesne_karti)
        n_duzen.setContentsMargins(10, 8, 10, 8)
        n_duzen.setSpacing(4)
        ust = QHBoxLayout()
        ust.addWidget(etiket("NESNELER", "bolum"))
        ust.addStretch()
        self.nesne_ozeti = etiket("", "silik")
        ust.addWidget(self.nesne_ozeti)
        n_duzen.addLayout(ust)
        self.tablo = NesneTablosu()
        n_duzen.addWidget(self.tablo, 1)

        bolucu = QSplitter(Qt.Vertical)
        bolucu.setChildrenCollapsible(False)
        bolucu.addWidget(cizim_karti)
        bolucu.addWidget(nesne_karti)
        bolucu.setStretchFactor(0, 3)
        bolucu.setStretchFactor(1, 2)

        duzen = QVBoxLayout(self)
        duzen.setContentsMargins(0, 0, 0, 0)
        duzen.setSpacing(6)
        duzen.addWidget(self.karar)
        duzen.addWidget(self.neden)
        duzen.addWidget(bolucu, 1)
        self._son_tablo = 0.0

    def guncelle(self, ros, d, nesne_bagli):
        # --- karar ---
        taze = ros.karar_taze()
        nokta = ros.engel_noktasi if ros.nokta_taze() else None
        if taze:
            alt = f"{ros.mesafe:.2f} m" if ros.mesafe >= 0 else "engel yok"
            if nokta is not None and ros.mesafe >= 0:
                alt += f"  ·  {aciyi_anlat(noktanin_acisi(nokta.x, nokta.y))}"
            self.karar.ayarla(ros.karar, KARAR_ADLARI.get(ros.karar, ros.karar), alt)
        else:
            self.karar.ayarla(None, "VERİ YOK", "LiDAR verisi ya da engel_node bekleniyor")

        # --- kararı veren nokta ---
        if taze and nokta is not None and ros.mesafe >= 0:
            aci = noktanin_acisi(nokta.x, nokta.y)
            metin = (f"Kararı veren nokta: <b>{aciyi_anlat(aci)}</b> ({aci:+.0f}°), "
                     f"<b>{ros.mesafe:.2f} m</b> uzakta, yükseklik <b>{nokta.z:+.2f} m</b>")
            # LiDAR'ın epey altındaysa büyük ihtimalle yer (zemin) engel sanılıyor
            if nokta.z < -0.5:
                metin += (f"<br><span style='color:{tema.hex_('hata')}'>⚠ Bu nokta LiDAR'ın "
                          f"{-nokta.z * 100:.0f} cm ALTINDA; büyük ihtimalle ZEMİN. Yer ise "
                          f"Filtreler → 'Alt sınır (zemin)' değerini yaklaşık <b>{nokta.z + 0.25:.2f}</b> "
                          "yapın.</span>")
            self.neden.setText(metin)
        elif taze:
            self.neden.setText(f"<span style='color:{tema.hex_('basari')}'>Görüş alanında engel yok.</span>")
        else:
            self.neden.setText("")

        # --- çizim ---
        nesneler = ros.nesneler if (nesne_bagli and ros.nesneler_taze()) else []
        self.cizim.guncelle(d, ros.karar if taze else None,
                            (nokta.x, nokta.y) if (nokta is not None and ros.mesafe >= 0) else None, nesneler)
        self.bolge.setText(self._bolge_metni(d))

        # --- nesne tablosu (saniyede 5 kez yeter) ---
        simdi = time.monotonic()
        if simdi - self._son_tablo > 0.2:
            self._son_tablo = simdi
            self.tablo.guncelle(nesneler)
            if not nesne_bagli:
                self.nesne_ozeti.setText("nesne_node kapalı")
            else:
                yaya = sum(1 for n in nesneler if n.etiket == "yaya_adayi")
                alanda = sum(1 for n in nesneler if n.uyari_alaninda)
                self.nesne_ozeti.setText(f"{len(nesneler)} nesne · {yaya} yaya adayı · {alanda} uyarı alanında")

    @staticmethod
    def _bolge_metni(d):
        g = d["gorus_genisligi"]
        if g >= 360:
            yon = "<b>her yön (360°)</b>"
        else:
            yon = (f"<b>{aciyi_anlat(d['merkez_aci'] - g / 2)}</b> ile "
                   f"<b>{aciyi_anlat(d['merkez_aci'] + g / 2)}</b> arası ({g:.0f}°)")
        metin = (f"<span style='color:{tema.hex_('dikkat')}'>■</span> Uyarı alanı: {yon}, "
                 f"<b>{d['min_mesafe']:.2f}–{d['dikkat_mesafesi']:.1f} m</b>, yükseklik "
                 f"<b>{d['z_min']:+.2f} … {d['z_max']:+.2f} m</b>")
        ng = d["nesne_gorus_genisligi"]
        kutulama = "her yön (360°)" if ng >= 360 else (
            f"{aciyi_anlat(d['nesne_merkez_aci'] - ng / 2)} ile {aciyi_anlat(d['nesne_merkez_aci'] + ng / 2)} "
            f"arası ({ng:.0f}°)")
        metin += (f"<br><span style='color:{tema.hex_('kutulama')}'>■</span> Kutulama alanı: {kutulama}, "
                  f"<b>{d['menzil']:.1f} m</b>'ye kadar")
        if d["z_max"] - d["z_min"] < 0.5:
            metin += (f"<br><span style='color:{tema.hex_('hata')}'>⚠ Yükseklik aralığı çok dar "
                      f"({(d['z_max'] - d['z_min']) * 100:.0f} cm); çoğu cisim görülmez!</span>")
        return f"<span style='color:{tema.hex_('soluk')}; font-size:9pt'>{metin}</span>"


class AlgilamaBolumu(QSplitter):
    """Sol: karar + çizim + nesneler. Sağ: ayar paneli. Dar pencerede alt alta dizilir."""

    def __init__(self, ros, genel=None):
        super().__init__(Qt.Horizontal)
        self.ros = ros
        self.setChildrenCollapsible(False)
        self.gorunum = AlgilamaGorunumu()
        self.panel = AyarPaneli(ros, genel)
        self.addWidget(self.gorunum)
        self.addWidget(self.panel)
        self.setStretchFactor(0, 5)
        self.setStretchFactor(1, 6)
        self.panel.degisti.connect(self.guncelle_gorunum)

    def guncelle(self):
        self.panel.guncelle()
        self.guncelle_gorunum()

    def guncelle_gorunum(self):
        self.gorunum.guncelle(self.ros, self.panel.degerler(), self.panel.bagli(NESNE))

    def resizeEvent(self, olay):
        yon = Qt.Horizontal if self.width() >= 820 else Qt.Vertical
        if self.orientation() != yon:
            self.setOrientation(yon)
            toplam = self.width() if yon == Qt.Horizontal else self.height()
            self.setSizes([int(toplam * 0.45), int(toplam * 0.55)])
        super().resizeEvent(olay)

    def node_bagli(self, node=ENGEL):
        return self.panel.bagli(node)

