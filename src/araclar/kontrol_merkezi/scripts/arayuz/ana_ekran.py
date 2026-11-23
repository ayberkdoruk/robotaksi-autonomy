"""Açılış ekranı: sistem durumu ve dört bölümün kısayolları. Ayrıca genel ayarlar penceresi."""

import os
import stat
import subprocess
import time

from python_qt_binding.QtCore import QRectF, QSize, Qt
from python_qt_binding.QtGui import QFont, QPainter, QPen
from python_qt_binding.QtWidgets import (
    QAbstractButton, QCheckBox, QDialog, QDialogButtonBox, QFileDialog, QFormLayout, QGridLayout,
    QHBoxLayout, QLabel, QLineEdit, QRadioButton, QSizePolicy, QVBoxLayout, QWidget,
)

from . import ikonlar, komutlar, lidar_bilgi, tema
from .ayar_tanim import KAYIT_KOKU, PROFIL_KLASORU, ayarlar_ayni_mi, dosya_oku, son_ayarlari_oku
from .bilesenler import Bant, DurumKarti, Liste, arac_dugmesi, dugme, etiket
from .kayitlar import SIKISTIRMALAR, kayitlari_listele
from .surecler import disaridaki_surecler, pid_kapat
from .yardimci import sure_metni, tarih_metni


class KartDugmesi(QAbstractButton):
    """Ana ekrandaki büyük bölüm kartı."""

    def __init__(self, tur, baslik, aciklama, alt_bilgi=""):
        super().__init__()
        self.tur, self.baslik, self.aciklama, self.alt_bilgi = tur, baslik, aciklama, alt_bilgi
        self.setCursor(Qt.PointingHandCursor)
        self.setMinimumHeight(150)
        self.setMaximumHeight(230)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setToolTip(f"{baslik} bölümünü aç")

    def alt_bilgi_ayarla(self, metin):
        if metin != self.alt_bilgi:
            self.alt_bilgi = metin
            self.update()

    def sizeHint(self):
        return QSize(380, 180)

    def enterEvent(self, olay):
        self.update()
        super().enterEvent(olay)

    def leaveEvent(self, olay):
        self.update()
        super().leaveEvent(olay)

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        r = QRectF(self.rect()).adjusted(1, 1, -1, -1)
        ustunde = self.underMouse() and self.isEnabled()
        vurgu = tema.renk(f"kart_{self.tur}")
        p.setPen(QPen(tema.renk(f"kart_{self.tur}", 190) if ustunde else tema.renk("cizgi"), 1.5))
        p.setBrush(tema.renk("panel3" if self.isDown() else "panel2" if ustunde else "panel"))
        p.drawRoundedRect(r, 14, 14)
        if not self.isEnabled():
            p.setOpacity(0.45)

        simge = QRectF(r.x() + 22, r.y() + 22, 66, 66)
        p.setPen(Qt.NoPen)
        p.setBrush(tema.renk(f"kart_{self.tur}", 38))
        p.drawEllipse(simge)
        ikonlar.kart_ikonu_ciz(p, simge.adjusted(11, 11, -11, -11), self.tur)

        x = simge.right() + 20
        genislik = r.right() - x - 40
        f = QFont(self.font())
        f.setPointSizeF(15.5)
        f.setBold(True)
        p.setFont(f)
        p.setPen(tema.renk("yazi"))
        p.drawText(QRectF(x, r.y() + 22, genislik, 30), Qt.AlignLeft | Qt.AlignVCenter, self.baslik)
        f.setPointSizeF(10)
        f.setBold(False)
        p.setFont(f)
        p.setPen(tema.renk("soluk"))
        p.drawText(QRectF(x, r.y() + 56, genislik, r.height() - 100), Qt.AlignLeft | Qt.AlignTop | Qt.TextWordWrap,
                   self.aciklama)
        f.setPointSizeF(9)
        p.setFont(f)
        p.setPen(tema.renk("silik"))
        p.drawText(QRectF(r.x() + 22, r.bottom() - 34, r.width() - 44, 22), Qt.AlignLeft | Qt.AlignVCenter,
                   self.alt_bilgi)
        # sağda ok
        p.setPen(QPen(vurgu if ustunde else tema.renk("silik"), 2.2, Qt.SolidLine, Qt.RoundCap))
        cx, cy = r.right() - 26, r.y() + 37
        p.drawLine(int(cx - 5), int(cy - 7), int(cx + 2), int(cy))
        p.drawLine(int(cx + 2), int(cy), int(cx - 5), int(cy + 7))


class AnaEkran(QWidget):
    anahtar = "ana"

    def __init__(self, app):
        super().__init__()
        self.app = app
        self.setObjectName("sayfa")
        self.dinleyici = lidar_bilgi.PaketDinleyici()
        self.surucu = lidar_bilgi.surucu_ayarlari()
        self.kayitlar = []
        self._son_yoklama = self._son_kayit_tarama = self._son_dis_kontrol = 0.0
        self._dis = []

        # ---------------- başlık ----------------
        simge = QLabel()
        simge.setPixmap(ikonlar.uygulama_ikonu().pixmap(46, 46))
        baslik = QVBoxLayout()
        baslik.setSpacing(0)
        baslik.addWidget(etiket("Robotaksi Kontrol Merkezi", "baslik"))
        model = self.surucu.get("model_adi", "LiDAR")
        baslik.addWidget(etiket(f"RoboSense {model} · engel ve nesne algılama", "soluk"))
        self.tema_dugmesi = arac_dugmesi("ay", "Koyu / açık tema", self.app.tema_degistir, boyut=18)
        ayarlar = dugme("Ayarlar", "ayar", ipucu="Kayıt klasörü, RViz yerleşimi, tema, masaüstü kısayolu",
                        tiklaninca=self.app.genel_ayarlar_ac)
        ust = QHBoxLayout()
        ust.setSpacing(12)
        ust.addWidget(simge)
        ust.addLayout(baslik)
        ust.addStretch()
        ust.addWidget(self.tema_dugmesi)
        ust.addWidget(ayarlar)

        self.bant = Bant()

        # ---------------- durum kartları ----------------
        self.lidar_karti = DurumKarti("LiDAR bağlantısı")
        self.disk_karti = DurumKarti("Disk")
        self.profil_karti = DurumKarti("Ayar profili")
        self.kayit_karti = DurumKarti("Kayıtlar")
        durumlar = QHBoxLayout()
        durumlar.setSpacing(12)
        for k in (self.lidar_karti, self.disk_karti, self.profil_karti, self.kayit_karti):
            durumlar.addWidget(k, 1)

        # ---------------- bölüm kartları ----------------
        self.kartlar = {
            "engel": KartDugmesi("engel", "Engel Algılama",
                                 "Canlı LiDAR'dan DUR / DİKKAT / GÜVENLİ kararı, nesne kutuları ve "
                                 "çalışırken değişen tüm ayarlar.",
                                 "Açılır: LiDAR sürücüsü · engel_node · nesne_node · RViz"),
            "izle": KartDugmesi("izle", "LiDAR'ı İzle",
                                "Sadece ham nokta bulutu. LiDAR çalışıyor mu, saniyede kaç tur geliyor, "
                                "ne görüyor?",
                                "Açılır: LiDAR sürücüsü · RViz"),
            "kayit": KartDugmesi("kayit", "Kayıt Al",
                                 "LiDAR verisini kaydedin; sonra masada, LiDAR olmadan tekrar oynatıp "
                                 "ayarları deneyin.",
                                 "Açılır: LiDAR sürücüsü · kayıt"),
            "oynat": KartDugmesi("oynat", "Kayıt Oynat",
                                 "Kayıtlı veride algılamayı çalıştırın: duraklatın, tur tur ilerletin, "
                                 "ayarları gerçek veriyle deneyin.",
                                 ""),
        }
        izgara = QGridLayout()
        izgara.setSpacing(14)
        for i, (ad, k) in enumerate(self.kartlar.items()):
            k.clicked.connect(lambda _=False, ad=ad: self.app.bolum_ac(ad))
            izgara.addWidget(k, i // 2, i % 2)

        alt = etiket("Bir bölüme girince gereken programlar kendiliğinden açılır. "
                     "'Ana ekran' düğmesi o bölümün açtığı her şeyi kapatır.", "silik")
        alt.setAlignment(Qt.AlignCenter)

        duzen = QVBoxLayout(self)
        duzen.setContentsMargins(26, 20, 26, 16)
        duzen.setSpacing(14)
        duzen.addLayout(ust)
        duzen.addWidget(self.bant)
        duzen.addLayout(durumlar)
        duzen.addLayout(izgara, 1)
        duzen.addStretch()
        duzen.addWidget(alt)

    # ---------------- sayfa olayları ----------------
    def hazirla(self, **_):
        pass

    def girildi(self):
        self.surucu = lidar_bilgi.surucu_ayarlari()
        self.dinleyici.baslat(self.surucu.get("msop", 6699))
        self._son_kayit_tarama = self._son_dis_kontrol = 0.0
        self.tema_guncelle()
        self.yenile()

    def ayrildi(self):
        self.dinleyici.durdur()  # sürücü bu portu kullanacak

    def tema_guncelle(self):
        ikonlar.ata(self.tema_dugmesi, "gunes" if tema.aktif_ad == "koyu" else "ay", "yazi", 18)

    # ---------------- 10 Hz ----------------
    def yenile(self):
        simdi = time.monotonic()
        if simdi - self._son_yoklama < 0.5:
            return
        self._son_yoklama = simdi
        self._lidar_durumu()
        self._disk_durumu()
        if simdi - self._son_kayit_tarama > 5.0:
            self._son_kayit_tarama = simdi
            self.kayitlar = kayitlari_listele(self.app.genel.kayit_klasoru)
            self._kayit_durumu()
            self._profil_durumu()
        if simdi - self._son_dis_kontrol > 3.0:
            self._son_dis_kontrol = simdi
            self._dis = disaridaki_surecler(["surucu", "engel", "nesne", "oynatici", "launch"],
                                            self.app.yonetici.bizim_pidler())
            self._bant_guncelle()

    def _bant_guncelle(self):
        eksik = komutlar.eksik_paketler()
        if eksik:
            self.bant.goster("hata", "Bazı programlar kurulu değil: <b>" + ", ".join(eksik) + "</b>. "
                             "Çalışma alanını derleyin: <code>cd ~/robotaksi_ws && colcon build --symlink-install</code>")
        elif self._dis:
            liste = ", ".join(f"<b>{ad}</b> (pid {pid})" for pid, ad, _ in self._dis)
            self.bant.goster("uyari", f"Başka bir yerden (eski kısayol ya da terminal) açılmış programlar "
                             f"çalışıyor: {liste}. Bunlar açıkken bölümlerdeki veriler karışır.",
                             [("Hepsini kapat", self._dislari_kapat)])
        else:
            self.bant.gizle()

    def _dislari_kapat(self):
        from python_qt_binding.QtWidgets import QApplication
        pid_kapat([pid for pid, _, _ in self._dis], olay_isle=QApplication.processEvents)
        self._son_dis_kontrol = 0.0
        self._dis = []
        self._bant_guncelle()
        # sürücü kapandıysa port boşaldı: dinlemeyi yeniden dene
        self.dinleyici.baslat(self.surucu.get("msop", 6699))

    def _lidar_durumu(self):
        durum = self.dinleyici.yokla()
        port = self.surucu.get("msop", 6699)
        if "hata" in self.surucu:
            self.lidar_karti.ayarla("Sürücü ayarı okunamadı", self.surucu["hata"], "uyari")
        elif durum == "geliyor":
            self.lidar_karti.ayarla("Paket geliyor ✓", f"{self.dinleyici.kaynak} adresinden, saniyede "
                                    f"~{self.dinleyici.paket_hizi:.0f} paket (port {port})", "basari")
        elif durum == "bekliyor":
            self.lidar_karti.ayarla("Kontrol ediliyor…", f"Port {port} dinleniyor", "silik")
        elif durum == "gelmiyor":
            self.lidar_karti.ayarla("Paket gelmiyor", "LiDAR'ın gücünü, ethernet kablosunu ve bilgisayarın "
                                    "sabit IP adresini (genelde 192.168.1.102) kontrol edin.", "hata")
        elif durum == "mesgul":
            self.lidar_karti.ayarla("Port kullanımda", f"{port} portunu başka bir program dinliyor; "
                                    "açık bir LiDAR sürücüsü olabilir.", "uyari")
            # sürücü kapandıysa tekrar dene
            self.dinleyici.baslat(port)
        else:
            self.lidar_karti.ayarla("Kontrol edilemedi", self.dinleyici.hata, "uyari")

    def _disk_durumu(self):
        bos, _ = lidar_bilgi.disk_durumu(self.app.genel.kayit_klasoru)
        if bos is None:
            self.disk_karti.ayarla("Bilinmiyor", self.app.genel.kayit_klasoru, "uyari")
            return
        dakika = lidar_bilgi.kayit_suresi_tahmini(bos, self.app.genel["sikistirma"])
        gb = bos / 1024 ** 3
        renk = "basari" if gb > 20 else "uyari" if gb > 5 else "hata"
        self.disk_karti.ayarla(f"{gb:.0f} GB boş", f"≈ {sure_metni(dakika * 60)} kayda yeter · "
                               f"{self.app.genel['kayit_klasoru']}", renk)

    def _kayit_durumu(self):
        n = len(self.kayitlar)
        if not n:
            self.kayit_karti.ayarla("Henüz kayıt yok", "Kayıt Al bölümünden ilk kaydı alın.", "silik")
            self.kartlar["oynat"].alt_bilgi_ayarla("Henüz kayıt yok")
            return
        son = self.kayitlar[0]
        self.kayit_karti.ayarla(f"{n} kayıt", f"Son: {son.ad} · {tarih_metni(son.tarih)} · "
                                f"{sure_metni(son.sure_s)}", "basari")
        self.kartlar["oynat"].alt_bilgi_ayarla(f"{n} kayıt · son: {son.ad}, {tarih_metni(son.tarih)}")

    def _profil_durumu(self):
        ad = self.app.genel["son_profil"]
        yol = os.path.join(PROFIL_KLASORU, ad + ".yaml") if ad else ""
        if not ad or not os.path.exists(yol):
            self.profil_karti.ayarla("Profil seçilmedi", "Node'lar son kullanılan ayarlarla açılır "
                                     "(son_ayarlar.yaml).", "silik")
            return
        profil, _ = dosya_oku(yol)
        if profil is not None and ayarlar_ayni_mi(profil, son_ayarlari_oku()):
            self.profil_karti.ayarla(ad, "Son ayarlar bu profille aynı.", "basari")
        else:
            self.profil_karti.ayarla(ad, "Son ayarlar bu profilden değiştirildi (henüz kaydedilmedi).", "uyari")


# -----------------------------------------------------------------------------
# Genel ayarlar
# -----------------------------------------------------------------------------
def kisayol_olustur():
    """Masaüstüne ve uygulama menüsüne 'Kontrol Merkezi' kısayolu koyar. (mesaj, başarılı)"""
    program = komutlar.paket_programi("kontrol_merkezi", "kontrol_merkezi_baslat.sh")
    if not program:
        return "kontrol_merkezi_baslat.sh bulunamadı (paket derlendi mi?).", False
    try:
        from ament_index_python.packages import get_package_share_directory
        simge = os.path.join(get_package_share_directory("kontrol_merkezi"), "resource", "kontrol_merkezi.svg")
    except Exception:
        simge = "utilities-terminal"
    icerik = ("[Desktop Entry]\nType=Application\nVersion=1.0\nName=Kontrol Merkezi\n"
              "Comment=LiDAR sürücüsü, engel algılama, kayıt alma ve oynatma: hepsi tek pencerede\n"
              f"Exec={program}\nIcon={simge}\nTerminal=false\nCategories=Science;\n"
              "StartupWMClass=kontrol_merkezi\n")
    try:
        masaustu = subprocess.run(["xdg-user-dir", "DESKTOP"], capture_output=True, text=True,
                                  timeout=3).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        masaustu = ""
    masaustu = masaustu or os.path.expanduser("~/Desktop")
    hedefler = [os.path.expanduser("~/.local/share/applications/kontrol-merkezi.desktop")]
    if os.path.isdir(masaustu):
        hedefler.append(os.path.join(masaustu, "kontrol-merkezi.desktop"))
    yazilan = []
    for yol in hedefler:
        try:
            os.makedirs(os.path.dirname(yol), exist_ok=True)
            with open(yol, "w", encoding="utf-8") as f:
                f.write(icerik)
            os.chmod(yol, os.stat(yol).st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
            # GNOME masaüstünde "Başlatmaya izin ver" adımını atla
            subprocess.run(["gio", "set", yol, "metadata::trusted", "true"], capture_output=True, timeout=3)
            yazilan.append(yol)
        except (OSError, subprocess.SubprocessError):
            pass
    if not yazilan:
        return "Kısayol yazılamadı.", False
    return "Kısayol oluşturuldu:\n" + "\n".join(yazilan), True


class GenelAyarlarPenceresi(QDialog):
    def __init__(self, app):
        super().__init__(app)
        self.app = app
        g = app.genel
        self.setWindowTitle("Kontrol Merkezi — Ayarlar")
        self.setMinimumWidth(560)

        self.klasor = QLineEdit(g["kayit_klasoru"])
        sec = dugme("Seç…", "klasor", tiklaninca=self._klasor_sec)
        klasor_satiri = QHBoxLayout()
        klasor_satiri.addWidget(self.klasor, 1)
        klasor_satiri.addWidget(sec)

        self.rviz = QCheckBox("Bir bölüme girince RViz de açılsın")
        self.rviz.setChecked(g["rviz_otomatik"])
        self.yan_yana = QCheckBox("RViz'i bu pencerenin yanına yerleştir (ekranı ikiye böl)")
        self.yan_yana.setChecked(g["yan_yana"])
        self.nesne = QCheckBox("Nesne algılama (kutular, nesne_node) varsayılan olarak açık")
        self.nesne.setChecked(g["nesne_acik"])

        self.koyu = QRadioButton("Koyu (gece, RViz'in yanında)")
        self.acik = QRadioButton("Açık (güneşli havada)")
        (self.koyu if g["tema"] == "koyu" else self.acik).setChecked(True)
        tema_satiri = QHBoxLayout()
        tema_satiri.addWidget(self.koyu)
        tema_satiri.addWidget(self.acik)
        tema_satiri.addStretch()

        self.sikistirma = Liste()
        for kimlik, ad in SIKISTIRMALAR:
            self.sikistirma.addItem(ad, kimlik)
        self.sikistirma.setCurrentIndex(max(0, [k for k, _ in SIKISTIRMALAR].index(g["sikistirma"])
                                            if g["sikistirma"] in dict(SIKISTIRMALAR) else 0))

        kisayol = dugme("Masaüstü kısayolunu oluştur", "ev",
                        ipucu="Masaüstüne ve uygulama menüsüne 'Kontrol Merkezi' simgesi koyar",
                        tiklaninca=self._kisayol)
        self.kisayol_sonucu = etiket("", "silik", kaydir=True)
        ayar_klasoru = dugme("Ayar dosyalarının klasörünü aç", "klasor",
                             tiklaninca=lambda: subprocess.Popen(["xdg-open", KAYIT_KOKU]))

        form = QFormLayout()
        form.setSpacing(12)
        form.setLabelAlignment(Qt.AlignRight | Qt.AlignVCenter)
        form.addRow("Kayıt klasörü", klasor_satiri)
        form.addRow("Varsayılan sıkıştırma", self.sikistirma)
        form.addRow("RViz", self.rviz)
        form.addRow("", self.yan_yana)
        form.addRow("Nesne algılama", self.nesne)
        form.addRow("Tema", tema_satiri)
        form.addRow("Kısayol", kisayol)
        form.addRow("", self.kisayol_sonucu)
        form.addRow("Ayar dosyaları", ayar_klasoru)

        dugmeler = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Cancel)
        dugmeler.button(QDialogButtonBox.Save).setText("Kaydet")
        dugmeler.button(QDialogButtonBox.Cancel).setText("Vazgeç")
        dugmeler.accepted.connect(self._kaydet)
        dugmeler.rejected.connect(self.reject)

        duzen = QVBoxLayout(self)
        duzen.setContentsMargins(20, 18, 20, 16)
        duzen.setSpacing(14)
        duzen.addWidget(etiket("Ayarlar", "sayfa_basligi"))
        duzen.addLayout(form)
        duzen.addWidget(dugmeler)

    def _klasor_sec(self):
        yol = QFileDialog.getExistingDirectory(self, "Kayıt klasörü", os.path.expanduser(self.klasor.text()))
        if yol:
            ev = os.path.expanduser("~")
            self.klasor.setText("~" + yol[len(ev):] if yol.startswith(ev) else yol)

    def _kisayol(self):
        mesaj, _ = kisayol_olustur()
        self.kisayol_sonucu.setText(mesaj)

    def _kaydet(self):
        yeni_tema = "koyu" if self.koyu.isChecked() else "acik"
        self.app.genel.guncelle(
            kayit_klasoru=self.klasor.text().strip() or "~/robotaksi_kayitlar",
            rviz_otomatik=self.rviz.isChecked(), yan_yana=self.yan_yana.isChecked(),
            nesne_acik=self.nesne.isChecked(), sikistirma=self.sikistirma.currentData())
        if yeni_tema != tema.aktif_ad:
            self.app.tema_uygula(yeni_tema)
        self.accept()
