"""
Bölüm sayfaları: Engel Algılama, LiDAR'ı İzle, Kayıt Al, Kayıt Oynat.

Her sayfa hangi programlara ihtiyacı olduğunu gereken_surecler() ile söyler; ana pencere
çalışanlarla karşılaştırıp gereksizleri kapatır, eksikleri açar, ayarı değişeni (ör. RViz
düzeni, oynatıcının döngü ayarı) yeniden başlatır. Pencere hiç kapanmaz.
"""

import html
import os
import subprocess
import time
from collections import namedtuple

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import (
    QAbstractItemView, QCheckBox, QGridLayout, QHBoxLayout, QHeaderView, QInputDialog, QLabel,
    QLineEdit, QMessageBox, QSizePolicy, QSplitter, QTableWidget, QTableWidgetItem, QVBoxLayout,
    QWidget,
)

from . import ikonlar, komutlar, lidar_bilgi, tema
from .bilesenler import (
    Bant, Cip, CiktiCekmecesi, Cubuk, Liste, SurecSeridi, arac_dugmesi, ayirac, dugme, etiket, kart,
)
from .kayitlar import (
    SIKISTIRMALAR, cop_kutusuna_at, isim_temizle, kayit_oku, kayitlari_listele, yeniden_adlandir,
)
from .surecler import BITTI, CALISIYOR, HATA
from .yardimci import bin_metni, boyut_metni, saat_metni, sure_metni, tarih_metni

# Bir sayfanın ihtiyaç duyduğu program: kimlik değişirse program yeniden başlatılır.
Gerek = namedtuple("Gerek", "kimlik ad komut_fn")

SURE_SECENEKLERI = [("Durdurana kadar", 0), ("30 saniye", 30), ("1 dakika", 60), ("2 dakika", 120),
                    ("5 dakika", 300), ("10 dakika", 600), ("30 dakika", 1800)]


class BolumSayfasi(QWidget):
    anahtar = ""
    baslik = ""
    tur = ""
    veri_kaynagi = "surucu"  # /rslidar_points'i kim yayınlıyor

    def __init__(self, app):
        super().__init__()
        self.app = app
        self.setObjectName("sayfa")
        self.rviz_acik = app.genel["rviz_otomatik"]
        self.nesne_acik = app.genel["nesne_acik"]
        self.rviz_dugme = None
        self.nesne_dugme = None
        self._son_cip = 0.0

        # ---------------- üst çubuk ----------------
        ust = kart("ustcubuk")
        u = QHBoxLayout(ust)
        u.setContentsMargins(10, 8, 12, 8)
        u.setSpacing(8)
        u.addWidget(dugme("Ana ekran", "ev", ipucu="Bu bölümü kapat, ana ekrana dön",
                          tiklaninca=lambda: app.bolum_ac("ana")))
        u.addWidget(ayirac())
        simge = QLabel()
        simge.setPixmap(ikonlar.kart_ikonu(self.tur, 26))
        u.addWidget(simge)
        u.addWidget(etiket(self.baslik, "sayfa_basligi"))
        self.ust_sol = QHBoxLayout()
        self.ust_sol.setSpacing(8)
        u.addSpacing(6)
        u.addLayout(self.ust_sol)
        u.addStretch()
        self.ust_sag = QHBoxLayout()
        self.ust_sag.setSpacing(8)
        u.addLayout(self.ust_sag)
        self.lidar_cip = Cip("LiDAR kapalı", "silik", "/rslidar_points: saniyedeki tur sayısı ve tur başına nokta")
        self.ust_sag.addWidget(self.lidar_cip)

        # ---------------- gövde + çıktı çekmecesi ----------------
        self.bant = Bant()
        self.govde = QWidget()
        self.govde_duzen = QVBoxLayout(self.govde)
        self.govde_duzen.setContentsMargins(0, 0, 0, 0)
        self.govde_duzen.setSpacing(10)
        ic = QWidget()
        ic_duzen = QVBoxLayout(ic)
        ic_duzen.setContentsMargins(12, 10, 12, 8)
        ic_duzen.setSpacing(10)
        ic_duzen.addWidget(self.bant)
        ic_duzen.addWidget(self.govde, 1)

        self.cekmece = CiktiCekmecesi(app.yonetici)
        self.cekmece.hide()
        self.bolucu = QSplitter(Qt.Vertical)
        self.bolucu.setChildrenCollapsible(False)
        self.bolucu.addWidget(ic)
        self.bolucu.addWidget(self.cekmece)
        self.bolucu.setStretchFactor(0, 3)
        self.bolucu.setStretchFactor(1, 1)

        self.serit = SurecSeridi()
        self.serit.cikti.toggled.connect(self._cekmece_goster)
        self.serit.tiklandi.connect(self.cikti_goster)
        self.cekmece.kapat_istendi.connect(lambda: self.serit.cikti.setChecked(False))

        duzen = QVBoxLayout(self)
        duzen.setContentsMargins(0, 0, 0, 0)
        duzen.setSpacing(0)
        duzen.addWidget(ust)
        duzen.addWidget(self.bolucu, 1)
        duzen.addWidget(self.serit)

    # ---------------- ortak düğmeler ----------------
    def rviz_dugmesi(self):
        d = dugme("RViz", "goz", isaretlenebilir=True,
                  ipucu="RViz penceresini aç / kapat (3B nokta bulutu, kutular, yelpaze)")
        d.setChecked(self.rviz_acik)
        d.toggled.connect(self._rviz_degisti)
        self.rviz_dugme = d
        return d

    def _rviz_degisti(self, acik):
        self.rviz_acik = acik
        self.app.esitle()

    def nesne_dugmesi(self):
        d = dugme("Nesne algılama", "kutu", isaretlenebilir=True,
                  ipucu="nesne_node'u aç / kapat (kutular, mesafe, etiket, takip). Kapatınca sadece "
                        "DUR / DİKKAT kararı çalışır, bilgisayar daha az yorulur.")
        d.setChecked(self.nesne_acik)
        d.toggled.connect(self._nesne_degisti)
        self.nesne_dugme = d
        return d

    def _nesne_degisti(self, acik):
        self.nesne_acik = acik
        self.app.genel["nesne_acik"] = acik
        self.app.esitle()

    def _cekmece_goster(self, goster):
        self.cekmece.setVisible(goster)
        if goster:
            self.cekmece.yenile()
            if self.bolucu.sizes()[1] < 120:
                toplam = sum(self.bolucu.sizes())
                self.bolucu.setSizes([int(toplam * 0.68), int(toplam * 0.32)])

    def cikti_goster(self, anahtar):
        self.serit.cikti.setChecked(True)
        self.cekmece.sec(anahtar)

    # ---------------- alt sınıfların dolduracakları ----------------
    def gereken_surecler(self):
        return {}

    def gosterilen(self):
        """Alt şeritte ve çıktı çekmecesinde görünen programlar: [(anahtar, ad), ...]"""
        return []

    def serit_ekleri(self):
        return {}

    def ek_bant(self):
        return None

    def hazirla(self, **secenek):
        pass

    def girildi(self):
        liste = self.gosterilen()
        self.serit.ayarla(liste)
        self.cekmece.ayarla(liste)
        if self.rviz_dugme is not None:
            self.rviz_dugme.blockSignals(True)
            self.rviz_dugme.setChecked(self.rviz_acik)
            self.rviz_dugme.blockSignals(False)

    def ayrildi(self):
        pass

    def surec_degisti(self, anahtar):
        """Ana pencere bu sayfa açıkken bir programın durumu değişince çağırır."""
        s = self.app.yonetici.surecler.get(anahtar)
        # Kullanıcı RViz penceresini kendisi kapattıysa düğme de kapansın
        if anahtar == "rviz" and s and s.durum == BITTI and self.rviz_acik and self.rviz_dugme:
            self.rviz_acik = False
            self.rviz_dugme.blockSignals(True)
            self.rviz_dugme.setChecked(False)
            self.rviz_dugme.blockSignals(False)

    # ---------------- 10 Hz ----------------
    def yenile(self):
        yon = self.app.yonetici
        self.serit.guncelle(yon, self.serit_ekleri())
        self.cekmece.yenile()
        simdi = time.monotonic()
        if simdi - self._son_cip > 0.5:
            self._son_cip = simdi
            self._lidar_cipi()
            durum = self._bant_durumu()
            if durum:
                self.bant.goster(*durum)
            else:
                self.bant.gizle()

    def _lidar_cipi(self):
        sayac = self.app.ros.lidar_sayac
        kaynak = self.app.yonetici.surecler.get(self.veri_kaynagi)
        if sayac.yas() < 1.5:
            self.lidar_cip.ayarla(f"LiDAR {sayac.hz():.1f} Hz · {bin_metni(self.app.ros.nokta_sayisi)} nokta",
                                  "basari")
        elif kaynak and kaynak.acik and kaynak.calisma_suresi() > 4.0:
            self.lidar_cip.ayarla("LiDAR: veri yok", "hata")
        elif kaynak and kaynak.acik:
            self.lidar_cip.ayarla("LiDAR: bekleniyor…", "uyari")
        else:
            self.lidar_cip.ayarla("LiDAR kapalı", "silik")

    def _bant_durumu(self):
        yon = self.app.yonetici
        gereken = self.gereken_surecler()
        for anahtar, _ in self.gosterilen():
            s = yon.surecler.get(anahtar)
            if s and s.durum == HATA and anahtar in gereken:
                son = next((x for x in reversed(s.son_satirlar(12)) if x.strip() and not x.startswith("────")), "")
                metin = (f"<b>{html.escape(s.ad)}</b> beklenmedik şekilde kapandı (çıkış kodu {s.cikis_kodu}).")
                if son:
                    metin += f"<br><span style='font-size:9pt'>Son satır: <i>{html.escape(son[:200])}</i></span>"
                return ("hata", metin, [("Yeniden başlat", lambda k=anahtar: self.app.yeniden_baslat(k)),
                                        ("Çıktıyı göster", lambda k=anahtar: self.cikti_goster(k))])
        ek = self.ek_bant()
        if ek:
            return ek
        if self.app.bekleyen_var():
            return ("bilgi", "Programlar hazırlanıyor… (önceki programların kapanması bekleniyor)", [])
        kaynak = yon.surecler.get("surucu")
        if (self.veri_kaynagi == "surucu" and kaynak and kaynak.durum == CALISIYOR
                and kaynak.calisma_suresi() > 5.0 and self.app.ros.lidar_sayac.yas() > 2.5):
            return ("uyari", "<b>LiDAR'dan veri gelmiyor.</b> Sürücü çalışıyor ama /rslidar_points boş. "
                    "LiDAR'ın gücünü, ethernet kablosunu ve bilgisayarın sabit IP adresini "
                    "(LiDAR'ın veri gönderdiği adres, genelde 192.168.1.102) kontrol edin.",
                    [("Sürücü çıktısı", lambda: self.cikti_goster("surucu"))])
        return None

    # ---------------- yardımcılar ----------------
    def _algilama_gerekleri(self, rviz_turu="tam"):
        g = {"engel": Gerek(("engel",), "engel_node", komutlar.engel_node)}
        if self.nesne_acik:
            g["nesne"] = Gerek(("nesne",), "nesne_node", komutlar.nesne_node)
        if self.rviz_acik:
            g["rviz"] = self._rviz_gerek(rviz_turu)
        return g

    def _rviz_gerek(self, tur):
        return Gerek(("rviz", tur), "RViz", lambda: komutlar.rviz(tur, self.app.rviz_geometrisi()))

    def _surucu_gerek(self):
        return Gerek(("surucu",), "LiDAR sürücüsü", komutlar.surucu)

    def _hz_ekleri(self):
        ros, yon = self.app.ros, self.app.yonetici
        ek = {}
        for anahtar, sayac in (("engel", ros.karar_sayac), ("nesne", ros.nesne_sayac),
                               (self.veri_kaynagi, ros.lidar_sayac)):
            s = yon.surecler.get(anahtar)
            if s and s.durum == CALISIYOR:
                hz = sayac.hz()
                ek[anahtar] = f"{hz:.0f} Hz" if hz > 0 else "veri yok"
        return ek


# -----------------------------------------------------------------------------
# Hızlı kayıt çubuğu (Engel Algılama ve LiDAR'ı İzle sayfalarında)
# -----------------------------------------------------------------------------
class KayitCubugu(QWidget):
    def __init__(self, app, algilama_secenegi=False):
        super().__init__()
        self.app = app
        k = kart()
        d = QHBoxLayout(k)
        d.setContentsMargins(12, 8, 10, 8)
        d.setSpacing(8)
        simge = QLabel()
        simge.setPixmap(ikonlar.ikon("kayit", "kart_kayit", 18).pixmap(18, 18))
        d.addWidget(simge)
        d.addWidget(etiket("Hızlı kayıt", "altbaslik"))
        self.isim = QLineEdit()
        self.isim.setPlaceholderText("kayıt adı (ör. otopark_yaya)")
        self.isim.setMaximumWidth(240)
        d.addWidget(self.isim, 1)
        self.sure = Liste()
        for ad, sn in SURE_SECENEKLERI:
            self.sure.addItem(ad, sn)
        d.addWidget(self.sure)
        self.algilama = QCheckBox("Algılama sonuçlarını da kaydet")
        self.algilama.setToolTip("LiDAR verisinin yanında /engel/* ve /nesne/* çıktıları da kaydedilir: sonradan "
                                 "o an hangi kararın verildiği görülebilir. (Oynatırken sadece LiDAR oynatılır.)")
        self.algilama.setVisible(algilama_secenegi)
        d.addWidget(self.algilama)
        self.baslat = dugme("Kaydı başlat", "kayit", tur="kayit", tiklaninca=self._baslat_durdur,
                            ikon_rengi="kart_kayit")
        d.addWidget(self.baslat)
        self.durum = etiket("", "soluk")
        self.durum.setTextFormat(Qt.RichText)
        d.addWidget(self.durum, 1)
        self.oynat = dugme("Oynat", "oynat", ipucu="Bu kaydı Kayıt Oynat bölümünde aç", tiklaninca=self._oynat)
        self.oynat.hide()
        d.addWidget(self.oynat)
        d.addWidget(arac_dugmesi("kapat", "Gizle", lambda: self.setVisible(False)))
        duzen = QVBoxLayout(self)
        duzen.setContentsMargins(0, 0, 0, 0)
        duzen.addWidget(k)
        self._son_yol = None
        self.hide()
        app.kayit.bitti.connect(self._bitti)

    def _baslat_durdur(self):
        kayit = self.app.kayit
        if kayit.kayitta:
            kayit.durdur()
            return
        hata = kayit.baslat(self.isim.text(), self.sure.currentData(), self.app.genel["sikistirma"],
                            self.algilama.isVisible() and self.algilama.isChecked())
        if hata:
            self.durum.setText(f"<span style='color:{tema.hex_('hata')}'>{html.escape(hata)}</span>")
        else:
            self.oynat.hide()

    def _bitti(self, yol, basarili, mesaj):
        if not self.isVisible():
            return
        renk = tema.hex_("basari" if basarili else "hata")
        k = kayit_oku(yol) if basarili else None
        ek = f" <b>{html.escape(k.ad)}</b>: {sure_metni(k.sure_s)}, {boyut_metni(k.boyut)}" if k else ""
        self.durum.setText(f"<span style='color:{renk}'>{html.escape(mesaj)}</span>{ek}")
        self._son_yol = yol if basarili else None
        self.oynat.setVisible(basarili)

    def _oynat(self):
        if self._son_yol:
            self.app.bolum_ac("oynat", kayit=self._son_yol)

    def yenile(self):
        kayit = self.app.kayit
        if kayit.kayitta:
            self.baslat.setText("Durdur")
            ikonlar.ata(self.baslat, "durdur", "kart_kayit")
            self.baslat.setEnabled(kayit.surec.durum != "duruyor")
            if kayit.surec.durum == "duruyor":
                self.durum.setText("Kayıt kapatılıyor…")
            else:
                self.durum.setText(f"<span style='color:{tema.hex_('kart_kayit')}'>●</span> <b>"
                                   f"{saat_metni(kayit.gecen())}</b> · {boyut_metni(kayit.boyut)} · "
                                   f"{kayit.tur_sayisi()} tur")
            self.isim.setEnabled(False)
            self.sure.setEnabled(False)
            self.algilama.setEnabled(False)
        else:
            self.baslat.setText("Kaydı başlat")
            ikonlar.ata(self.baslat, "kayit", "kart_kayit")
            tamam, neden = kayit.baslatilabilir_mi()
            self.baslat.setEnabled(tamam)
            self.baslat.setToolTip(neden or "Kaydı başlat")
            self.isim.setEnabled(True)
            self.sure.setEnabled(True)
            self.algilama.setEnabled(True)


def kayit_dugmesi(cubuk):
    """Üst çubukta hızlı kayıt çubuğunu açıp kapatan düğme."""
    d = dugme("Kayıt al", "kayit", tur="kayit", ipucu="Bu sayfadan çıkmadan kayıt al",
              ikon_rengi="kart_kayit", tiklaninca=lambda: cubuk.setVisible(not cubuk.isVisible()))
    return d


def kayit_dugmesi_guncelle(d, kayit):
    if kayit.kayitta:
        metin = f"Kayıtta  {saat_metni(kayit.gecen())}"
    else:
        metin = "Kayıt al"
    if d.text() != metin:
        d.setText(metin)


# -----------------------------------------------------------------------------
# Engel Algılama
# -----------------------------------------------------------------------------
class EngelSayfasi(BolumSayfasi):
    anahtar, baslik, tur = "engel", "Engel Algılama", "engel"

    def __init__(self, app):
        super().__init__(app)
        self.kayit_cubugu = KayitCubugu(app, algilama_secenegi=True)
        self.kayit_dugme = kayit_dugmesi(self.kayit_cubugu)
        for w in (self.nesne_dugmesi(), self.rviz_dugmesi(), self.kayit_dugme):
            self.ust_sag.addWidget(w)
        self.govde_duzen.addWidget(self.kayit_cubugu)
        self.yer = QVBoxLayout()
        self.govde_duzen.addLayout(self.yer, 1)

    def gereken_surecler(self):
        g = {"surucu": self._surucu_gerek()}
        g.update(self._algilama_gerekleri("tam"))
        return g

    def gosterilen(self):
        return [("surucu", "LiDAR sürücüsü"), ("engel", "engel_node"), ("nesne", "nesne_node"),
                ("rviz", "RViz"), ("kayit", "Kayıt")]

    def serit_ekleri(self):
        return self._hz_ekleri()

    def hazirla(self, **_):
        self.yer.addWidget(self.app.algilama)  # ortak algılama görünümü bu sayfaya taşınır

    def yenile(self):
        super().yenile()
        self.app.algilama.guncelle()
        self.kayit_cubugu.yenile()
        kayit_dugmesi_guncelle(self.kayit_dugme, self.app.kayit)


# -----------------------------------------------------------------------------
# LiDAR'ı İzle
# -----------------------------------------------------------------------------
class IzleSayfasi(BolumSayfasi):
    anahtar, baslik, tur = "izle", "LiDAR'ı İzle", "izle"

    def __init__(self, app):
        super().__init__(app)
        self.kayit_cubugu = KayitCubugu(app)
        self.kayit_dugme = kayit_dugmesi(self.kayit_cubugu)
        self.ust_sag.addWidget(self.rviz_dugmesi())
        self.ust_sag.addWidget(self.kayit_dugme)
        self.ust_sag.addWidget(dugme("Algılamayı da aç", "oynat", tur="birincil",
                                     ipucu="Sürücüyü kapatmadan Engel Algılama bölümüne geç",
                                     tiklaninca=lambda: app.bolum_ac("engel")))
        self.govde_duzen.addWidget(self.kayit_cubugu)

        # --- Veri kartı ---
        veri = kart()
        v = QVBoxLayout(veri)
        v.setContentsMargins(18, 14, 18, 16)
        v.setSpacing(8)
        v.addWidget(etiket("VERİ", "bolum"))
        self.hz = etiket("—", "buyuk_deger")
        v.addWidget(self.hz)
        self.hz_aciklama = etiket("saniyedeki tur sayısı (Helios 32 için ~10 olmalı)", "silik")
        v.addWidget(self.hz_aciklama)
        self.veri_izgara = QGridLayout()
        self.veri_izgara.setHorizontalSpacing(18)
        self.veri_izgara.setVerticalSpacing(6)
        self.veri_degerleri = {}
        for i, (anahtar, ad) in enumerate((("nokta", "Tur başına nokta"), ("yas", "Son veri"),
                                           ("kopma", "Kopan tur (son 1 dakika)"), ("toplam", "Gelen tur"))):
            self.veri_izgara.addWidget(etiket(ad, "soluk"), i, 0)
            deger = etiket("—")
            deger.setStyleSheet("font-weight: 600;")
            self.veri_izgara.addWidget(deger, i, 1)
            self.veri_degerleri[anahtar] = deger
        v.addLayout(self.veri_izgara)
        v.addStretch()
        v.addWidget(etiket("Kopan tur artıyorsa: ethernet kablosu / anahtar gevşek olabilir ya da bilgisayar "
                           "yetişemiyordur. Hz 0 ise LiDAR'ın gücünü ve IP ayarını kontrol edin.", "silik",
                           kaydir=True))

        # --- Sürücü kartı ---
        surucu = kart()
        s = QVBoxLayout(surucu)
        s.setContentsMargins(18, 14, 18, 16)
        s.setSpacing(8)
        s.addWidget(etiket("SÜRÜCÜ AYARLARI (değiştirilemez)", "bolum"))
        self.surucu_izgara = QGridLayout()
        self.surucu_izgara.setHorizontalSpacing(18)
        self.surucu_izgara.setVerticalSpacing(6)
        s.addLayout(self.surucu_izgara)
        s.addStretch()
        s.addWidget(etiket("Sürücü ayarları rslidar_sdk'nın kendi config.yaml dosyasındadır; LiDAR'ın kendisine "
                           "bu arayüzden hiçbir ayar gönderilmez. Yanlış model ya da port seçilirse hiç veri "
                           "gelmez, bu yüzden burada sadece gösterilir.", "silik", kaydir=True))
        self.klasor_dugme = dugme("Ayar dosyasının klasörünü aç", "klasor", tiklaninca=self._klasoru_ac)
        s.addWidget(self.klasor_dugme, 0, Qt.AlignLeft)

        satir = QHBoxLayout()
        satir.setSpacing(12)
        satir.addWidget(veri, 1)
        satir.addWidget(surucu, 1)
        self.govde_duzen.addLayout(satir, 1)
        self._surucu = {}

    def _klasoru_ac(self):
        if self._surucu.get("dosya"):
            subprocess.Popen(["xdg-open", os.path.dirname(self._surucu["dosya"])])

    def gereken_surecler(self):
        g = {"surucu": self._surucu_gerek()}
        if self.rviz_acik:
            g["rviz"] = self._rviz_gerek("lidar")
        return g

    def gosterilen(self):
        return [("surucu", "LiDAR sürücüsü"), ("rviz", "RViz"), ("kayit", "Kayıt")]

    def serit_ekleri(self):
        return self._hz_ekleri()

    def girildi(self):
        super().girildi()
        self._surucu = lidar_bilgi.surucu_ayarlari()
        while self.surucu_izgara.count():
            w = self.surucu_izgara.takeAt(0).widget()
            if w:
                w.setParent(None)
                w.deleteLater()
        if "hata" in self._surucu:
            satirlar = [("Hata", self._surucu["hata"])]
        else:
            m = self._surucu
            menzil = f"{m['min_mesafe']} – {m['max_mesafe']} m" if m.get("max_mesafe") is not None else "—"
            satirlar = [("Model", f"{m['model_adi']} ({m['model']})"), ("Veri kaynağı", m["kaynak"]),
                        ("Portlar", f"MSOP {m['msop']} · DIFOP {m['difop']}"), ("Menzil", menzil),
                        ("Topic", m["topic"]), ("Frame", m["frame"]), ("Dosya", m["dosya"])]
        for i, (ad, deger) in enumerate(satirlar):
            self.surucu_izgara.addWidget(etiket(ad, "soluk"), i, 0, Qt.AlignTop)
            e = etiket(str(deger), kaydir=True, secilebilir=True)
            e.setStyleSheet("font-weight: 600;")
            self.surucu_izgara.addWidget(e, i, 1)
        self.surucu_izgara.setColumnStretch(1, 1)
        self.klasor_dugme.setEnabled(bool(self._surucu.get("dosya")))

    def yenile(self):
        super().yenile()
        self.kayit_cubugu.yenile()
        kayit_dugmesi_guncelle(self.kayit_dugme, self.app.kayit)
        sayac = self.app.ros.lidar_sayac
        yas = sayac.yas()
        if yas < 1.5:
            hz = sayac.hz()
            self.hz.setText(f"{hz:.1f} Hz")
            self.hz.setStyleSheet(f"color: {tema.hex_('basari' if hz >= 8 else 'uyari')};")
        else:
            self.hz.setText("veri yok" if self.app.yonetici.surecler.get("surucu") else "—")
            self.hz.setStyleSheet(f"color: {tema.hex_('hata')};")
        d = self.veri_degerleri
        d["nokta"].setText(f"{self.app.ros.nokta_sayisi:,}".replace(",", " ") if yas < 1.5 else "—")
        d["yas"].setText(f"{yas:.1f} s önce" if yas != float("inf") else "hiç gelmedi")
        kopma = sayac.kopma_sayisi()
        d["kopma"].setText(str(kopma))
        d["kopma"].setStyleSheet(f"font-weight: 600; color: {tema.hex_('uyari' if kopma else 'yazi')};")
        d["toplam"].setText(f"{sayac.toplam:,}".replace(",", " "))


# -----------------------------------------------------------------------------
# Kayıt listesi (Kayıt Al sayfasında)
# -----------------------------------------------------------------------------
class KayitListesi(QWidget):
    SUTUNLAR = ["İsim", "Tarih", "Süre", "Boyut", "İçerik"]

    def __init__(self, app):
        super().__init__()
        self.app = app
        self.kayitlar = []
        self.tablo = QTableWidget(0, len(self.SUTUNLAR))
        self.tablo.setHorizontalHeaderLabels(self.SUTUNLAR)
        self.tablo.verticalHeader().setVisible(False)
        self.tablo.verticalHeader().setDefaultSectionSize(28)
        self.tablo.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.tablo.setSelectionMode(QAbstractItemView.SingleSelection)
        self.tablo.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.tablo.setAlternatingRowColors(True)
        self.tablo.setShowGrid(False)
        b = self.tablo.horizontalHeader()
        b.setSectionResizeMode(QHeaderView.ResizeToContents)
        b.setSectionResizeMode(0, QHeaderView.Stretch)
        self.tablo.doubleClicked.connect(lambda _: self._oynat())
        self.tablo.itemSelectionChanged.connect(self._secim_degisti)

        self.oynat = dugme("Oynat", "oynat", tur="birincil", tiklaninca=self._oynat)
        self.adlandir = dugme("Adını değiştir", "duzenle", tiklaninca=self._adlandir)
        self.sil = dugme("Sil", "sil", ipucu="Çöp kutusuna taşır (geri alınabilir)", tiklaninca=self._sil)
        klasor = dugme("Klasörü aç", "klasor", tiklaninca=self._klasor)
        yenile = arac_dugmesi("yenile", "Listeyi yenile", self.yenile)
        alt = QHBoxLayout()
        alt.setSpacing(6)
        for w in (self.oynat, self.adlandir, self.sil):
            alt.addWidget(w)
        alt.addStretch()
        alt.addWidget(klasor)
        alt.addWidget(yenile)

        duzen = QVBoxLayout(self)
        duzen.setContentsMargins(0, 0, 0, 0)
        duzen.setSpacing(8)
        duzen.addWidget(self.tablo, 1)
        duzen.addLayout(alt)
        self._secim_degisti()

    def yenile(self):
        secili = self._secili()
        self.kayitlar = kayitlari_listele(self.app.genel.kayit_klasoru)
        self.tablo.setRowCount(len(self.kayitlar))
        for i, k in enumerate(self.kayitlar):
            hucreler = [k.ad, tarih_metni(k.tarih), sure_metni(k.sure_s), boyut_metni(k.boyut),
                        "LiDAR + algılama" if k.algilama_var else "LiDAR"]
            for j, metin in enumerate(hucreler):
                oge = QTableWidgetItem(metin)
                if j == 0:
                    oge.setToolTip(k.yol)
                self.tablo.setItem(i, j, oge)
            if secili and k.yol == secili.yol:
                self.tablo.selectRow(i)
        self._secim_degisti()

    def _secili(self):
        satirlar = self.tablo.selectionModel().selectedRows() if self.tablo.selectionModel() else []
        if not satirlar:
            return None
        i = satirlar[0].row()
        return self.kayitlar[i] if i < len(self.kayitlar) else None

    def _secim_degisti(self):
        var = self._secili() is not None
        for d in (self.oynat, self.adlandir, self.sil):
            d.setEnabled(var)

    def _oynat(self):
        k = self._secili()
        if k:
            self.app.bolum_ac("oynat", kayit=k.yol)

    def _kullanimda(self, k):
        kayit = self.app.kayit
        return kayit.kayitta and kayit.yol == k.yol

    def _adlandir(self):
        k = self._secili()
        if not k or self._kullanimda(k):
            return
        isim, tamam = QInputDialog.getText(self, "Adını değiştir", "Yeni isim:", text=k.ad)
        if not tamam:
            return
        try:
            yeniden_adlandir(k, isim)
        except (OSError, ValueError) as e:
            QMessageBox.warning(self, "Adı değiştirilemedi", str(e))
        self.yenile()

    def _sil(self):
        k = self._secili()
        if not k or self._kullanimda(k):
            return
        if QMessageBox.question(self, "Kaydı sil?",
                                f"'{k.ad}' ({tarih_metni(k.tarih)}, {boyut_metni(k.boyut)}) çöp kutusuna "
                                "taşınsın mı?") != QMessageBox.Yes:
            return
        try:
            cop_kutusuna_at(k.yol)
        except OSError as e:
            QMessageBox.warning(self, "Silinemedi", str(e))
        self.yenile()

    def _klasor(self):
        klasor = self.app.genel.kayit_klasoru
        os.makedirs(klasor, exist_ok=True)
        subprocess.Popen(["xdg-open", klasor])


# -----------------------------------------------------------------------------
# Kayıt Al
# -----------------------------------------------------------------------------
class KayitSayfasi(BolumSayfasi):
    anahtar, baslik, tur = "kayit", "Kayıt Al", "kayit"

    def __init__(self, app):
        super().__init__(app)
        self.ust_sag.addWidget(self.rviz_dugmesi())

        # --- yeni kayıt ---
        form_karti = kart()
        f = QVBoxLayout(form_karti)
        f.setContentsMargins(18, 14, 18, 16)
        f.setSpacing(10)
        f.addWidget(etiket("YENİ KAYIT", "bolum"))
        izgara = QGridLayout()
        izgara.setHorizontalSpacing(12)
        izgara.setVerticalSpacing(10)
        self.isim = QLineEdit()
        self.isim.setPlaceholderText("ne kaydettiğinizi anlatsın (ör. otopark_yaya_testi)")
        self.isim.textChanged.connect(self._onizleme)
        self.onizleme = etiket("", "silik")
        self.sure = Liste()
        for ad, sn in SURE_SECENEKLERI:
            self.sure.addItem(ad, sn)
        self.sikistirma = Liste()
        for kimlik, ad in SIKISTIRMALAR:
            self.sikistirma.addItem(ad, kimlik)
        izgara.addWidget(etiket("İsim", "soluk"), 0, 0)
        izgara.addWidget(self.isim, 0, 1)
        izgara.addWidget(self.onizleme, 1, 1)
        izgara.addWidget(etiket("Süre", "soluk"), 2, 0)
        izgara.addWidget(self.sure, 2, 1)
        izgara.addWidget(etiket("Sıkıştırma", "soluk"), 3, 0)
        izgara.addWidget(self.sikistirma, 3, 1)
        izgara.addWidget(etiket("Kaydedilen", "soluk"), 4, 0)
        izgara.addWidget(etiket("LiDAR nokta bulutu (/rslidar_points)"), 4, 1)
        izgara.setColumnStretch(1, 1)
        f.addLayout(izgara)
        self.disk = etiket("", "silik", kaydir=True)
        f.addWidget(self.disk)
        f.addSpacing(4)
        self.baslat = dugme("KAYDI BAŞLAT", "kayit", tur="kayit", tiklaninca=self._baslat_durdur,
                            ikon_rengi="kart_kayit")
        tema.ozellik(self.baslat, "tur", "kayit")
        self.baslat.setMinimumHeight(52)
        self.baslat.setStyleSheet("font-size: 13pt; font-weight: 700;")
        f.addWidget(self.baslat)
        self.neden = etiket("", kaydir=True)
        self.neden.setTextFormat(Qt.RichText)
        f.addWidget(self.neden)

        # --- kayıt sürüyor ---
        self.sure_yazisi = etiket("00:00", "buyuk_deger")
        self.sure_yazisi.setAlignment(Qt.AlignCenter)
        self.ayrinti = etiket("", "soluk")
        self.ayrinti.setAlignment(Qt.AlignCenter)
        self.ayrinti.setTextFormat(Qt.RichText)
        f.addWidget(self.sure_yazisi)
        f.addWidget(self.ayrinti)
        f.addStretch()

        # --- kayıtlar ---
        liste_karti = kart()
        l_duzen = QVBoxLayout(liste_karti)
        l_duzen.setContentsMargins(14, 14, 14, 14)
        l_duzen.setSpacing(8)
        l_duzen.addWidget(etiket("KAYITLAR", "bolum"))
        self.liste = KayitListesi(app)
        l_duzen.addWidget(self.liste, 1)

        satir = QHBoxLayout()
        satir.setSpacing(12)
        satir.addWidget(form_karti, 4)
        satir.addWidget(liste_karti, 6)
        self.govde_duzen.addLayout(satir, 1)
        app.kayit.bitti.connect(self._bitti)
        self._onizleme()

    def _onizleme(self):
        isim = isim_temizle(self.isim.text()) or "kayit"
        self.onizleme.setText(f"Klasör: {self.app.genel['kayit_klasoru']}/{time.strftime('%Y-%m-%d_%H-%M-%S')}_{isim}")

    def gereken_surecler(self):
        g = {"surucu": self._surucu_gerek()}
        if self.rviz_acik:
            g["rviz"] = self._rviz_gerek("lidar")
        return g

    def gosterilen(self):
        return [("surucu", "LiDAR sürücüsü"), ("kayit", "Kayıt"), ("rviz", "RViz")]

    def serit_ekleri(self):
        return self._hz_ekleri()

    def girildi(self):
        super().girildi()
        i = self.sikistirma.findData(self.app.genel["sikistirma"])
        self.sikistirma.setCurrentIndex(max(0, i))
        self.liste.yenile()

    def _baslat_durdur(self):
        kayit = self.app.kayit
        if kayit.kayitta:
            kayit.durdur()
            return
        hata = kayit.baslat(self.isim.text(), self.sure.currentData(), self.sikistirma.currentData())
        self._sonuc = f"<span style='color:{tema.hex_('hata')}'>{html.escape(hata)}</span>" if hata else ""
        self.neden.setText(self._sonuc)

    def _bitti(self, yol, basarili, mesaj):
        self.liste.yenile()
        if not self.isVisible():
            return
        k = kayit_oku(yol) if basarili else None
        renk = tema.hex_("basari" if basarili else "hata")
        ek = f"<br><b>{html.escape(k.ad)}</b>: {sure_metni(k.sure_s)}, {boyut_metni(k.boyut)}, {k.mesaj} tur" if k else ""
        self._sonuc = f"<span style='color:{renk}'>{html.escape(mesaj)}</span>{ek}"
        self.neden.setText(self._sonuc)

    def yenile(self):
        super().yenile()
        kayit = self.app.kayit
        bos, _ = lidar_bilgi.disk_durumu(self.app.genel.kayit_klasoru)
        profil = self.sikistirma.currentData() or "zstd_fast"
        if bos is not None:
            dakika = lidar_bilgi.kayit_suresi_tahmini(bos, profil)
            renk = "hata" if bos < 5 * 1024 ** 3 else "silik"
            self.disk.setText(f"Diskte {boyut_metni(bos)} boş → yaklaşık {sure_metni(dakika * 60)} kayıt. "
                              f"(Dakikada ~{lidar_bilgi.DAKIKADA_GB.get(profil, 0.6):.1f} GB)")
            self.disk.setStyleSheet(f"color: {tema.hex_(renk)};")
        if kayit.kayitta:
            duruyor = kayit.surec.durum == "duruyor"
            self.baslat.setText("KAPATILIYOR…" if duruyor else "KAYDI DURDUR")
            ikonlar.ata(self.baslat, "durdur", "kart_kayit")
            self.baslat.setEnabled(not duruyor)
            self.sure_yazisi.show()
            self.ayrinti.show()
            hedef = f" / {saat_metni(kayit.sure_s)}" if kayit.sure_s else ""
            self.sure_yazisi.setText(saat_metni(kayit.gecen()) + hedef)
            self.sure_yazisi.setStyleSheet(f"color: {tema.hex_('kart_kayit')};")
            self.ayrinti.setText(f"{boyut_metni(kayit.boyut)} · {kayit.tur_sayisi()} tur · "
                                 f"{html.escape(os.path.basename(kayit.yol or ''))}")
            for w in (self.isim, self.sure, self.sikistirma):
                w.setEnabled(False)
        else:
            self.baslat.setText("KAYDI BAŞLAT")
            ikonlar.ata(self.baslat, "kayit", "kart_kayit")
            tamam, neden = kayit.baslatilabilir_mi()
            self.baslat.setEnabled(tamam)
            self.baslat.setToolTip(neden)
            metin = (f"<span style='color:{tema.hex_('uyari')}'>{html.escape(neden)}</span>"
                     if not tamam else getattr(self, "_sonuc", ""))
            if self.neden.text() != metin:
                self.neden.setText(metin)
            self.sure_yazisi.hide()
            self.ayrinti.hide()
            for w in (self.isim, self.sure, self.sikistirma):
                w.setEnabled(True)
            if int(time.time()) % 2 == 0:
                self._onizleme()


# -----------------------------------------------------------------------------
# Kayıt Oynat
# -----------------------------------------------------------------------------
class OynatSayfasi(BolumSayfasi):
    anahtar, baslik, tur = "oynat", "Kayıt Oynat", "oynat"
    veri_kaynagi = "oynatici"
    HIZLAR = [0.1, 0.25, 0.5, 1.0, 1.5, 2.0]

    def __init__(self, app):
        super().__init__(app)
        self.kayit = None
        self.hiz = 1.0
        self.dongu = True
        self.baslangic_s = 0.0
        self.jeton = 0           # "baştan oynat" için yeniden başlatma sayacı
        self.bitti = False
        self.duraklatildi = False
        self._suruklemede = False
        self.kayitlar = []

        self.secici = Liste()
        self.secici.setMinimumWidth(320)
        self.secici.setToolTip("Oynatılacak kayıt")
        self.secici.currentIndexChanged.connect(self._kayit_secildi)
        self.ust_sol.addWidget(self.secici)
        self.ust_sol.addWidget(arac_dugmesi("yenile", "Kayıt listesini yenile", self._listeyi_doldur))
        self.ust_sag.addWidget(self.nesne_dugmesi())
        self.ust_sag.addWidget(self.rviz_dugmesi())

        # --- oynatma çubuğu ---
        self.oynatma = kart()
        o = QHBoxLayout(self.oynatma)
        o.setContentsMargins(12, 8, 14, 8)
        o.setSpacing(10)
        self.duraklat = dugme("Duraklat", "duraklat", tiklaninca=self._duraklat_devam)
        self.duraklat.setMinimumWidth(110)
        self.sonraki = dugme("1 tur", "ileri", ipucu="Duraklatılmışken tek bir tur ilerlet (sahneyi incelemek için)",
                             tiklaninca=self._sonraki)
        self.zaman = etiket("00:00 / 00:00")
        self.zaman.setStyleSheet("font-weight: 600;")
        self.zaman.setMinimumWidth(110)
        self.ilerleme = Cubuk()
        self.ilerleme.setRange(0, 1000)
        self.ilerleme.setToolTip("Kaydın neresindesiniz. Sürükleyip bırakınca o ana atlar.")
        self.ilerleme.sliderPressed.connect(lambda: setattr(self, "_suruklemede", True))
        self.ilerleme.sliderReleased.connect(self._atla)
        self.hiz_secici = Liste()
        for h in self.HIZLAR:
            self.hiz_secici.addItem(f"{h:g}×", h)
        self.hiz_secici.setCurrentIndex(self.HIZLAR.index(1.0))
        self.hiz_secici.setToolTip("Oynatma hızı (oynarken değişir)")
        self.hiz_secici.currentIndexChanged.connect(self._hiz_degisti)
        self.dongu_kutusu = QCheckBox("Döngü")
        self.dongu_kutusu.setChecked(True)
        self.dongu_kutusu.setToolTip("Kayıt bitince baştan başlasın. Değiştirince oynatıcı bulunduğu yerden "
                                     "yeniden başlatılır.")
        self.dongu_kutusu.toggled.connect(self._dongu_degisti)
        for w in (self.duraklat, self.sonraki, self.zaman):
            o.addWidget(w)
        o.addWidget(self.ilerleme, 1)
        o.addWidget(etiket("Hız", "soluk"))
        o.addWidget(self.hiz_secici)
        o.addWidget(self.dongu_kutusu)
        self.govde_duzen.addWidget(self.oynatma)

        # --- kayıt yoksa ---
        self.bos = kart()
        b = QVBoxLayout(self.bos)
        b.setContentsMargins(30, 30, 30, 30)
        b.addStretch()
        mesaj = etiket("Henüz kayıt yok", "baslik")
        mesaj.setAlignment(Qt.AlignCenter)
        b.addWidget(mesaj)
        alt = etiket("Önce Kayıt Al bölümünden LiDAR verisi kaydedin. Kayıtlar Ayarlar'da seçilen klasörde "
                     "aranır.", "soluk", kaydir=True)
        alt.setAlignment(Qt.AlignCenter)
        b.addWidget(alt)
        git = dugme("Kayıt Al bölümüne git", "kayit", tur="birincil", tiklaninca=lambda: app.bolum_ac("kayit"))
        b.addWidget(git, 0, Qt.AlignCenter)
        b.addStretch()
        self.govde_duzen.addWidget(self.bos, 1)

        self.yer = QVBoxLayout()
        self.govde_duzen.addLayout(self.yer, 1)

    # ---------------- kayıt seçimi ----------------
    def _listeyi_doldur(self, secilecek=None):
        secilecek = secilecek or (self.kayit.yol if self.kayit else None) or self.app.genel["son_kayit"]
        self.kayitlar = kayitlari_listele(self.app.genel.kayit_klasoru)
        self.secici.blockSignals(True)
        self.secici.clear()
        indeks = 0
        for i, k in enumerate(self.kayitlar):
            ek = " · LiDAR + algılama" if k.algilama_var else ""
            self.secici.addItem(f"{k.ad}  ·  {tarih_metni(k.tarih)}  ·  {sure_metni(k.sure_s)}{ek}", k.yol)
            if k.yol == secilecek:
                indeks = i
        if self.kayitlar:
            self.secici.setCurrentIndex(indeks)
        self.secici.blockSignals(False)
        yeni = self.kayitlar[indeks] if self.kayitlar else None
        if (yeni and yeni.yol) != (self.kayit and self.kayit.yol):
            self._kayit_ata(yeni)
        self.bos.setVisible(not self.kayitlar)
        self.oynatma.setVisible(bool(self.kayitlar))
        self.app.algilama.setVisible(bool(self.kayitlar))

    def _kayit_ata(self, kayit):
        self.kayit = kayit
        self.bitti, self.duraklatildi, self.baslangic_s = False, False, 0.0
        if kayit:
            self.app.genel["son_kayit"] = kayit.yol

    def _kayit_secildi(self, i):
        if 0 <= i < len(self.kayitlar):
            self._kayit_ata(self.kayitlar[i])
            self.app.esitle()

    # ---------------- oynatıcı ----------------
    def _konum(self):
        """Kaydın kaçıncı saniyesindeyiz (yaklaşık)."""
        ros = self.app.ros
        if not self.kayit or ros.saat_ns is None or not self.kayit.baslangic_ns:
            return 0.0
        return min(max(0.0, (ros.saat_ns - self.kayit.baslangic_ns) / 1e9), self.kayit.sure_s)

    def _oynatici_komutu(self):
        baslangic, self.baslangic_s = self.baslangic_s, 0.0
        self.duraklatildi = False
        return komutlar.oynatici(self.kayit.yol, self.hiz, self.dongu, baslangic)

    def _duraklat_devam(self):
        if self.app.ros.oynatici("devam" if self.duraklatildi else "duraklat"):
            self.duraklatildi = not self.duraklatildi

    def _sonraki(self):
        self.app.ros.oynatici("sonraki")

    def _hiz_degisti(self, i):
        self.hiz = self.HIZLAR[i]
        self.app.ros.oynatici("hiz", self.hiz)

    def _dongu_degisti(self, acik):
        # Döngü ayarı oynarken değiştirilemiyor: oynatıcı bulunduğu yerden yeniden başlatılır
        self.dongu = acik
        self.baslangic_s = self._konum()
        self.bitti = False
        self.app.esitle()

    def _atla(self):
        self._suruklemede = False
        if not self.kayit:
            return
        hedef_s = self.ilerleme.value() / 1000.0 * self.kayit.sure_s
        if self.bitti:  # oynatıcı kapanmışsa oradan yeniden başlat
            self.bitti, self.baslangic_s = False, hedef_s
            self.jeton += 1
            self.app.esitle()
            return
        self.app.ros.oynatici("atla", self.kayit.baslangic_ns + int(hedef_s * 1e9))

    def _bastan(self):
        self.bitti, self.baslangic_s = False, 0.0
        self.jeton += 1
        self.app.esitle()

    # ---------------- sayfa ----------------
    def gereken_surecler(self):
        if not self.kayit:
            return {}
        g = {}
        if not self.bitti:
            g["oynatici"] = Gerek(("oynatici", self.kayit.yol, self.dongu, self.jeton), "Kayıt oynatıcı",
                                  self._oynatici_komutu)
        g.update(self._algilama_gerekleri("tam"))
        return g

    def gosterilen(self):
        return [("oynatici", "Oynatıcı"), ("engel", "engel_node"), ("nesne", "nesne_node"), ("rviz", "RViz")]

    def serit_ekleri(self):
        return self._hz_ekleri()

    def hazirla(self, kayit=None, **_):
        self.yer.addWidget(self.app.algilama)
        self._listeyi_doldur(secilecek=kayit)

    def surec_degisti(self, anahtar):
        super().surec_degisti(anahtar)
        s = self.app.yonetici.surecler.get(anahtar)
        if anahtar == "oynatici" and s and s.durum == BITTI:
            self.bitti = True

    def ek_bant(self):
        if self.bitti:
            return ("bilgi", "Kayıt sona erdi. Döngü kapalıyken kayıt bir kez oynatılır.",
                    [("Baştan oynat", self._bastan)])
        return None

    def yenile(self):
        super().yenile()
        if not self.kayit:
            return
        self.app.algilama.guncelle()
        hazir = self.app.ros.oynatici_hazir()
        konum = self.kayit.sure_s if self.bitti else self._konum()
        self.zaman.setText(f"{saat_metni(konum)} / {saat_metni(self.kayit.sure_s)}")
        if not self._suruklemede and self.kayit.sure_s > 0:
            self.ilerleme.blockSignals(True)
            self.ilerleme.setValue(int(konum / self.kayit.sure_s * 1000))
            self.ilerleme.blockSignals(False)
        self.duraklat.setEnabled(hazir and not self.bitti)
        self.sonraki.setEnabled(hazir and self.duraklatildi and not self.bitti)
        metin, simge = ("Devam", "oynat") if self.duraklatildi else ("Duraklat", "duraklat")
        if self.duraklat.text() != metin:
            self.duraklat.setText(metin)
            ikonlar.ata(self.duraklat, simge)
