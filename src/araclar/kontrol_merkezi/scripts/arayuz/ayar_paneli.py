"""
Ayar paneli: engel_node ve nesne_node'un çalışırken değişebilen TÜM ayarları,
sekmelere ayrılmış halde, ve kayıtlı ayar profilleri.

- Çubuk oynatılınca / değer yazılınca 150 ms sonra node'a gönderilir (anında uygulanır).
- Node kabul ederse ~/.ros/kontrol_merkezi/son_ayarlar.yaml'a otomatik yazılır; node'lar
  bir sonraki açılışta bu değerlerle başlar.
- Node kapalıysa onun sekmeleri kilitli kalır; node açılınca değerleri ondan okunur.
"""

import os
import re
import time

from python_qt_binding.QtCore import Qt, QTimer, Signal
from python_qt_binding.QtWidgets import (
    QFileDialog, QHBoxLayout, QInputDialog, QMenu, QMessageBox, QScrollArea, QTabWidget,
    QToolButton, QVBoxLayout, QWidget,
)
from rclpy.parameter import Parameter, parameter_value_to_python

from . import ikonlar, tema
from .ayar_tanim import (
    ENGEL, GOVDE, NESNE, NODE_AYARLARI, PROFIL_KLASORU, SEKMELER, SON_AYARLAR, TANIM,
    ayarlar_ayni_mi, dosya_oku, dosya_yaz, kontrol_et, son_ayarlari_oku,
)
from .bilesenler import AyarSatiri, Liste, arac_dugmesi, dugme, etiket, kart

PARAMETRE_TIPI = {"ondalik": Parameter.Type.DOUBLE, "tam": Parameter.Type.INTEGER,
                  "mantik": Parameter.Type.BOOL}


class NodeBaglantisi:
    def __init__(self, node, istemci, hedef):
        self.node, self.istemci, self.hedef = node, istemci, hedef
        self.hazir = False         # parametre servisleri görünüyor mu
        self.yuklendi = False      # node'dan değerler okundu mu
        self.okuma = None
        self.okuma_zamani = 0.0
        self.gonderme = None
        self.son_gonderilen = {}
        self.son_kontrol = 0.0


class AyarPaneli(QWidget):
    degisti = Signal()  # çizimin yenilenmesi için

    def __init__(self, ros, genel=None):
        super().__init__()
        self.ros, self.genel = ros, genel
        self.baglantilar = {node: NodeBaglantisi(node, ros.param[node], ros.hedefler[node])
                            for node in (ENGEL, NESNE)}
        self.satirlar = {}
        self.kapali_uyarilari = {ENGEL: [], NESNE: []}

        # ---------------- Profil çubuğu ----------------
        profil = kart()
        p_duzen = QVBoxLayout(profil)
        p_duzen.setContentsMargins(12, 10, 12, 10)
        p_duzen.setSpacing(6)
        ust = QHBoxLayout()
        ust.addWidget(etiket("AYAR PROFİLİ", "bolum"))
        ust.addStretch()
        self.profil_durumu = etiket("", "silik")
        ust.addWidget(self.profil_durumu)
        p_duzen.addLayout(ust)

        satir = QHBoxLayout()
        satir.setSpacing(6)
        self.profil_listesi = Liste()
        self.profil_listesi.setMinimumWidth(140)
        self.profil_listesi.setToolTip("Kayıtlı ayar profilleri (ör. masa, araç, otopark)")
        satir.addWidget(self.profil_listesi, 1)
        satir.addWidget(dugme("Yükle", "yukle", ipucu="Seçili profildeki ayarları uygula",
                              tiklaninca=self._profil_yukle))
        satir.addWidget(dugme("Kaydet…", "kaydet", ipucu="Şu anki ayarları isim vererek kaydet",
                              tiklaninca=self._profil_kaydet))
        menu = QMenu(self)
        menu.addAction("Seçili profili sil", self._profil_sil)
        menu.addSeparator()
        menu.addAction("Dışa aktar… (başka bilgisayara taşımak için)", self._disa_aktar)
        menu.addAction("İçe aktar… (.yaml ayar dosyası)", self._ice_aktar)
        menu.addSeparator()
        menu.addAction("Bu sekmeyi varsayılana döndür", self._sekme_varsayilan)
        menu.addAction("Tüm ayarları varsayılana döndür", self._hepsi_varsayilan)
        diger = arac_dugmesi("menu", "Diğer işlemler", tur=None)
        diger.setMenu(menu)
        diger.setPopupMode(QToolButton.InstantPopup)
        satir.addWidget(diger)
        p_duzen.addLayout(satir)

        # ---------------- Sekmeler ----------------
        self.sekmeler = QTabWidget()
        self.sekmeler.setDocumentMode(True)
        self.sekmeler.setUsesScrollButtons(True)
        self.sekme_kimlikleri = []
        for kimlik, baslik, node, giris in SEKMELER:
            icerik = QWidget()
            duzen = QVBoxLayout(icerik)
            duzen.setContentsMargins(6, 8, 10, 8)
            duzen.setSpacing(0)
            uyari = etiket("", kaydir=True)
            uyari.setTextFormat(Qt.RichText)
            uyari.setStyleSheet("padding: 6px 2px;")
            self.kapali_uyarilari[node].append(uyari)
            duzen.addWidget(uyari)
            aciklama = etiket(giris, "aciklama", kaydir=True)
            aciklama.setStyleSheet("padding: 2px 2px 8px 2px;")
            duzen.addWidget(aciklama)
            for a in NODE_AYARLARI[node]:
                if a.sekme != kimlik:
                    continue
                s = AyarSatiri(a)
                s.setEnabled(False)
                s.degisti.connect(self._ayar_degisti)
                self.satirlar[a.anahtar] = s
                duzen.addWidget(s)
            if kimlik == "kutulama":
                kapat = dugme("Gövde kutusunu kapat", "kapat",
                              ipucu="Dört sınırı da 0 yapar: gövde kutusu kullanılmaz",
                              tiklaninca=self._govde_kapat)
                self.govde_dugmesi = kapat
                kapat.setEnabled(False)
                h = QHBoxLayout()
                h.setContentsMargins(0, 10, 0, 0)
                h.addWidget(kapat)
                h.addStretch()
                duzen.addLayout(h)
            duzen.addStretch()
            kaydirma = QScrollArea()
            kaydirma.setWidgetResizable(True)
            kaydirma.setWidget(icerik)
            kaydirma.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
            self.sekmeler.addTab(kaydirma, baslik)
            self.sekme_kimlikleri.append((kimlik, node))

        self.durum = etiket("", kaydir=True)
        self.durum.setMinimumHeight(34)
        self.durum.setStyleSheet("padding: 4px 4px;")

        ana = QVBoxLayout(self)
        ana.setContentsMargins(0, 0, 0, 0)
        ana.setSpacing(8)
        ana.addWidget(profil)
        sekme_karti = kart()
        s_duzen = QVBoxLayout(sekme_karti)
        s_duzen.setContentsMargins(8, 4, 4, 4)
        s_duzen.addWidget(self.sekmeler)
        ana.addWidget(sekme_karti, 1)
        ana.addWidget(self.durum)

        # Çubuk oynarken saniyede onlarca istek göndermemek için kısa bekleme
        self.gonder_zamanlayici = QTimer(self)
        self.gonder_zamanlayici.setSingleShot(True)
        self.gonder_zamanlayici.setInterval(150)
        self.gonder_zamanlayici.timeout.connect(self._gonder)

        # Açılışta son kaydedilen değerleri göster (node bağlanınca onun değerleri gelir)
        for anahtar, v in son_ayarlari_oku().items():
            if anahtar in self.satirlar:
                self.satirlar[anahtar].ayarla(v)
        self._profilleri_listele(self.genel["son_profil"] if self.genel else None)
        for node in (ENGEL, NESNE):
            self._kilitle(node, True)
        self._durum_yaz("engel_node / nesne_node bekleniyor…", "soluk")

    # ---------------- dışarıya ----------------
    def degerler(self):
        return {k: s.deger() for k, s in self.satirlar.items()}

    def bagli(self, node):
        return self.baglantilar[node].yuklendi

    def sifirla(self):
        """Node'lar yeniden açılacaksa: bağlantı durumunu baştan kur (değerler tekrar okunur)."""
        for node, b in self.baglantilar.items():
            b.yuklendi = b.hazir = False
            b.okuma = b.gonderme = None
            b.son_kontrol = 0.0
            self._kilitle(node, True)

    # ---------------- yardımcılar ----------------
    def _durum_yaz(self, metin, renk="soluk"):
        self.durum.setText(metin)
        self.durum.setStyleSheet(f"padding: 4px; color: {tema.hex_(renk)};")

    def _kilitle(self, node, kilitli):
        for a in NODE_AYARLARI[node]:
            self.satirlar[a.anahtar].setEnabled(not kilitli)
        if node == NESNE:
            self.govde_dugmesi.setEnabled(not kilitli)
        hedef = self.baglantilar[node].hedef
        for u in self.kapali_uyarilari[node]:
            if kilitli:
                u.setText(f"<span style='color:{tema.hex_('uyari')}'>● <b>{hedef}</b> çalışmıyor.</span> "
                          f"<span style='color:{tema.hex_('soluk')}'>Gösterilen değerler son kaydedilen "
                          "ayarlardır; node açılınca değiştirilebilir.</span>")
                u.show()
            else:
                u.hide()
        for i, (_, n) in enumerate(self.sekme_kimlikleri):
            if n == node:
                self.sekmeler.setTabIcon(i, ikonlar.nokta("uyari", 8) if kilitli else ikonlar.nokta("basari", 8))

    def _yuklu_degerler(self):
        """Sadece bağlanılmış node'ların değerleri."""
        return {k: s.deger() for k, s in self.satirlar.items() if self.bagli(TANIM[k].node)}

    def _otomatik_kaydet(self, ek=None):
        """son_ayarlar.yaml: bağlanılmamış node'un önceki kayıtlı değerleri korunur."""
        degerler = son_ayarlari_oku()
        degerler.update(self._yuklu_degerler())
        if ek:
            degerler.update(ek)
        dosya_yaz(SON_AYARLAR, degerler)
        self._profil_durumunu_guncelle()

    def _hatalari_goster(self):
        hatalar = kontrol_et(self.degerler())
        ilgili = {k for _, anahtarlar in hatalar for k in anahtarlar}
        for k, s in self.satirlar.items():
            s.hatali(k in ilgili)
        return hatalar

    # ---------------- node bağlantısı (ana pencerenin zamanlayıcısından, ~10 Hz) ----------------
    def guncelle(self):
        simdi = time.monotonic()
        for node, b in self.baglantilar.items():
            liste = NODE_AYARLARI[node]
            if simdi - b.son_kontrol > 0.5:
                b.son_kontrol = simdi
                hazir = b.istemci.services_are_ready()
                if b.yuklendi and not hazir:
                    b.yuklendi, b.okuma, b.gonderme = False, None, None
                    self._kilitle(node, True)
                    self._durum_yaz(f"'{b.hedef}' kapandı; ayarları kilitlendi.", "uyari")
                    self.degisti.emit()
                b.hazir = hazir

            # 1) Node'un mevcut değerlerini oku
            if b.hazir and not b.yuklendi and b.okuma is None:
                b.okuma = b.istemci.get_parameters([a.ros_adi for a in liste])
                b.okuma_zamani = simdi
            if b.okuma is not None and b.okuma.done():
                sonuc = b.okuma.result()
                b.okuma = None
                if sonuc is not None and len(sonuc.values) == len(liste):
                    for a, deger in zip(liste, sonuc.values):
                        v = parameter_value_to_python(deger)
                        if v is not None:
                            self.satirlar[a.anahtar].ayarla(v)
                    b.son_gonderilen = {a.anahtar: self.satirlar[a.anahtar].deger() for a in liste}
                    ilk = not b.yuklendi
                    b.yuklendi = True
                    self._kilitle(node, False)
                    if ilk:
                        self._durum_yaz(f"'{b.hedef}' bağlandı. Değişiklikler anında uygulanır ve "
                                        "otomatik kaydedilir.", "basari")
                    self._hatalari_goster()
                    self._profil_durumunu_guncelle()
                    self.degisti.emit()
            elif b.okuma is not None and simdi - b.okuma_zamani > 3.0:
                b.okuma = None  # cevap gelmedi, tekrar denenecek

            # 2) Gönderilen ayarın sonucu
            if b.gonderme is not None and b.gonderme.done():
                sonuc = b.gonderme.result()
                b.gonderme = None
                hatalar = [r.reason for r in (sonuc.results if sonuc else []) if not r.successful]
                if hatalar:
                    self._durum_yaz(f"Reddedildi ({b.hedef}): " + "; ".join(hatalar), "hata")
                    b.yuklendi = False  # node'daki gerçek değerleri tekrar oku
                else:
                    try:
                        self._otomatik_kaydet()
                        self._durum_yaz("Uygulandı ✓  (otomatik kaydedildi)", "basari")
                    except OSError as e:
                        self._durum_yaz(f"Uygulandı ✓ ama otomatik kayıt başarısız: {e}", "uyari")

    # ---------------- değişiklik ----------------
    def _ayar_degisti(self, _anahtar=None):
        self.degisti.emit()
        hatalar = self._hatalari_goster()
        if hatalar:
            self._durum_yaz("⚠ " + " ".join(m for m, _ in hatalar) + "  (Düzeltilene kadar gönderilmiyor.)",
                            "hata")
            self.gonder_zamanlayici.stop()
            return
        self.gonder_zamanlayici.start()

    def _gonder(self):
        d = self.degerler()
        gonderildi = False
        for b in self.baglantilar.values():
            if not b.yuklendi:
                continue
            if b.gonderme is not None:
                self.gonder_zamanlayici.start()  # önceki istek bitmedi, biraz sonra tekrar
                continue
            degisenler = []
            for a in NODE_AYARLARI[b.node]:
                if b.son_gonderilen.get(a.anahtar) == d[a.anahtar]:
                    continue
                degisenler.append(Parameter(a.ros_adi, PARAMETRE_TIPI[a.tip], a.ros_degeri(d[a.anahtar])))
            if degisenler:
                b.son_gonderilen.update({a.anahtar: d[a.anahtar] for a in NODE_AYARLARI[b.node]})
                b.gonderme = b.istemci.set_parameters(degisenler)
                gonderildi = True
        if not gonderildi and not self.gonder_zamanlayici.isActive() and any(
                b.yuklendi for b in self.baglantilar.values()):
            self._durum_yaz("Ayarlar zaten bu değerlerde ✓", "basari")

    def _govde_kapat(self):
        for k in GOVDE:
            self.satirlar[k].ayarla(0.0)
        self._ayar_degisti()

    # ---------------- profiller ----------------
    def _degerleri_uygula(self, degerler, kaynak):
        """Dosyadan okunan değerler: açık node'lara hemen gönderilir, kapalı node'unkiler
        son_ayarlar.yaml'a yazılır (node açılınca o değerlerle başlar)."""
        canli, sonraya = {}, {}
        for k, v in degerler.items():
            (canli if self.bagli(TANIM[k].node) else sonraya)[k] = v
            self.satirlar[k].ayarla(v)
        if sonraya:
            try:
                self._otomatik_kaydet(ek=sonraya)
            except OSError as e:
                self._durum_yaz(f"'{kaynak}' kaydedilemedi: {e}", "hata")
                return
        if canli:
            self._ayar_degisti()
        else:
            self.degisti.emit()
        mesaj = f"'{kaynak}' yüklendi."
        if sonraya:
            kapali = sorted({self.baglantilar[TANIM[k].node].hedef for k in sonraya})
            mesaj += f" {', '.join(kapali)} kapalı; onun ayarları açılınca uygulanacak."
        self._durum_yaz(mesaj, "basari")

    def _profil_yolu(self, ad):
        return os.path.join(PROFIL_KLASORU, ad + ".yaml")

    def _profilleri_listele(self, secili=None):
        self.profil_listesi.clear()
        if os.path.isdir(PROFIL_KLASORU):
            adlar = sorted(f[:-5] for f in os.listdir(PROFIL_KLASORU) if f.endswith(".yaml"))
            self.profil_listesi.addItems(adlar)
        if self.profil_listesi.count() == 0:
            self.profil_listesi.setPlaceholderText("(henüz kayıtlı profil yok)")
        if secili:
            self.profil_listesi.setCurrentText(secili)
        self._profil_durumunu_guncelle()

    def _profil_durumunu_guncelle(self):
        ad = self.genel["son_profil"] if self.genel else ""
        if not ad or not os.path.exists(self._profil_yolu(ad)):
            self.profil_durumu.setText("")
            return
        profil, _ = dosya_oku(self._profil_yolu(ad))
        if profil is None:
            self.profil_durumu.setText("")
        elif ayarlar_ayni_mi(profil, self.degerler()):
            self.profil_durumu.setText(f"'{ad}' ile aynı ✓")
        else:
            self.profil_durumu.setText(f"'{ad}' profilinden değiştirildi")

    def _profil_kaydet(self):
        ad, tamam = QInputDialog.getText(self, "Ayarları kaydet", "Profil adı (ör. masa, arac, otopark):",
                                         text=self.profil_listesi.currentText())
        if not tamam:
            return
        ad = re.sub(r"[^\w\-]+", "_", ad.strip()).strip("_")
        if not ad:
            self._durum_yaz("Geçerli bir isim girin.", "hata")
            return
        yol = self._profil_yolu(ad)
        if os.path.exists(yol) and QMessageBox.question(
                self, "Üzerine yaz?", f"'{ad}' adında bir profil zaten var. Üzerine yazılsın mı?") \
                != QMessageBox.Yes:
            return
        try:
            dosya_yaz(yol, self.degerler())
        except OSError as e:
            self._durum_yaz(f"Kaydedilemedi: {e}", "hata")
            return
        if self.genel:
            self.genel["son_profil"] = ad
        self._profilleri_listele(secili=ad)
        self._durum_yaz(f"'{ad}' kaydedildi ✓", "basari")

    def _profil_yukle(self):
        ad = self.profil_listesi.currentText()
        if not ad:
            self._durum_yaz("Yüklenecek profil yok. Önce 'Kaydet…' ile bir profil oluşturun.", "hata")
            return
        degerler, hata = dosya_oku(self._profil_yolu(ad))
        if hata:
            self._durum_yaz(f"'{ad}' yüklenemedi: {hata}", "hata")
            return
        if self.genel:
            self.genel["son_profil"] = ad
        self._degerleri_uygula(degerler, ad)
        self._profil_durumunu_guncelle()

    def _profil_sil(self):
        ad = self.profil_listesi.currentText()
        if not ad:
            return
        if QMessageBox.question(self, "Sil?", f"'{ad}' profili silinsin mi?") != QMessageBox.Yes:
            return
        try:
            os.remove(self._profil_yolu(ad))
        except OSError as e:
            self._durum_yaz(f"Silinemedi: {e}", "hata")
            return
        if self.genel and self.genel["son_profil"] == ad:
            self.genel["son_profil"] = ""
        self._profilleri_listele()
        self._durum_yaz(f"'{ad}' silindi.", "soluk")

    def _disa_aktar(self):
        yol, _ = QFileDialog.getSaveFileName(self, "Ayarları dışa aktar",
                                             os.path.expanduser("~/engel_ayarlari.yaml"),
                                             "YAML ayar dosyası (*.yaml *.yml)")
        if not yol:
            return
        if not yol.endswith((".yaml", ".yml")):
            yol += ".yaml"
        try:
            dosya_yaz(yol, self.degerler())
        except OSError as e:
            self._durum_yaz(f"Dışa aktarılamadı: {e}", "hata")
            return
        self._durum_yaz(f"Dışa aktarıldı: {yol}", "basari")

    def _ice_aktar(self):
        yol, _ = QFileDialog.getOpenFileName(self, "Ayar dosyası aç", os.path.expanduser("~"),
                                             "YAML ayar dosyası (*.yaml *.yml)")
        if not yol:
            return
        degerler, hata = dosya_oku(yol)
        if hata:
            self._durum_yaz(f"İçe aktarılamadı: {hata}", "hata")
            return
        self._degerleri_uygula(degerler, os.path.basename(yol))

    def _sekme_varsayilan(self):
        kimlik, node = self.sekme_kimlikleri[self.sekmeler.currentIndex()]
        degerler = {a.anahtar: a.varsayilan for a in NODE_AYARLARI[node] if a.sekme == kimlik}
        self._degerleri_uygula(degerler, f"{self.sekmeler.tabText(self.sekmeler.currentIndex())} varsayılanları")

    def _hepsi_varsayilan(self):
        if QMessageBox.question(self, "Varsayılana dön?",
                                "Tüm ayarlar paketin varsayılan değerlerine döndürülsün mü?") != QMessageBox.Yes:
            return
        self._degerleri_uygula({k: TANIM[k].varsayilan for k in self.satirlar}, "Varsayılan ayarlar")
