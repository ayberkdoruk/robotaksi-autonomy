"""
engel_node ve nesne_node ayarlarının tanımları, ayar dosyaları ve arayüzün kendi ayarları.

Kayıt dosyaları (~/.ros/kontrol_merkezi/):
  son_ayarlar.yaml : panelde en son UYGULANAN ayarlar. Her değişiklikte otomatik
                     yazılır; launch dosyası ve Kontrol Merkezi node'ları
                     açarken bunu varsayılanların üstüne yükler.
  profiller/*.yaml : "Kaydet…" ile isim verilerek saklanan ayarlar (ör. masa, arac)
  arayuz.yaml      : Kontrol Merkezi'nin kendi ayarları (tema, kayıt klasörü…)
Ayar dosyaları ROS parametre dosyası biçimindedir, yani
  ros2 run lidar_algilama engel_node --ros-args --params-file DOSYA.yaml
ile de doğrudan kullanılabilir.

Alt/üst sınırlar engel_node.cpp ve nesne_node.cpp'deki aralıklarla AYNI olmalı.
Varsayılan değerler paketin config/*.yaml dosyalarından okunur.
"""

import os
from dataclasses import dataclass

import yaml

KAYIT_KOKU = os.path.expanduser("~/.ros/kontrol_merkezi")
SON_AYARLAR = os.path.join(KAYIT_KOKU, "son_ayarlar.yaml")
PROFIL_KLASORU = os.path.join(KAYIT_KOKU, "profiller")
ARAYUZ_AYARLARI = os.path.join(KAYIT_KOKU, "arayuz.yaml")

ENGEL = "engel_node"
NESNE = "nesne_node"
NODE_ADLARI = {ENGEL: "engel_node (DUR / DİKKAT kararı)", NESNE: "nesne_node (kutular)"}


@dataclass
class Ayar:
    anahtar: str          # arayüz içindeki eşsiz ad
    node: str             # engel_node / nesne_node
    baslik: str
    birim: str
    alt: float
    ust: float
    adim: float
    varsayilan: object
    aciklama: str
    sekme: str
    tip: str = "ondalik"  # "ondalik" | "tam" | "mantik"
    ros_adi: str = ""     # node'daki parametre adı (boşsa anahtar ile aynı)

    def __post_init__(self):
        if not self.ros_adi:
            self.ros_adi = self.anahtar

    def ros_degeri(self, v):
        """Parametreyi ROS'un beklediği tipe çevirir."""
        if self.tip == "tam":
            return int(round(v))
        if self.tip == "mantik":
            return bool(v)
        return round(float(v), 4)

    @property
    def basamak(self):
        if self.tip != "ondalik":
            return 0
        a = self.adim
        return 0 if a >= 1 else 1 if a >= 0.1 else 2 if a >= 0.01 else 3

    def metin(self, v):
        if self.tip == "mantik":
            return "açık" if v else "kapalı"
        bosluk = "" if self.birim in ("°", "") else " "
        return f"{v:.{self.basamak}f}{bosluk}{self.birim}"


# (kimlik, sekme başlığı, node, sekmenin üstündeki açıklama)
SEKMELER = [
    ("uyari", "Uyarı alanı", ENGEL,
     "Engel kararı (DUR / DİKKAT / GÜVENLİ) LiDAR'ın etrafındaki bir yelpazeye bakılarak "
     "verilir. Yelpazenin yönünü, genişliğini ve uyarı mesafelerini buradan seçin."),
    ("filtre", "Filtreler", ENGEL,
     "Yelpazenin içindeki hangi noktaların engel sayılacağı. Zemin ve tavan noktaları "
     "burada ayıklanır; tek tük gürültü noktaları en az nokta şartıyla elenir."),
    ("kutulama", "Kutulama alanı", NESNE,
     "nesne_node'un nesneleri kutu içine aldığı alan. Uyarı yelpazesinden bağımsızdır "
     "(çizimde mavi kesik çizgi). Gövde kutusu, aracın kendi gövdesini yok saydırır."),
    ("zemin", "Zemin", NESNE,
     "Zemin her turda yeniden ölçülüp ayrılır. 'Otomatik zemin bulma' açıkken açılışta kendisi "
     "bulunur; 'Beklenen zemin yüksekliği' (LiDAR'ın yerden yüksekliğinin eksilisi, 1.80 m → "
     "-1.80) o zaman kontrol için kullanılır: bulunan zemin tutmazsa uyarı çıkar."),
    ("kume", "Kümeleme", NESNE,
     "Zemin dışında kalan noktalar birbirine yakınlığına göre nesnelere ayrılır. Yan yana "
     "iki nesne birleşiyorsa mesafeleri küçültün; tek nesne parçalanıyorsa büyütün."),
    ("etiket", "Etiket", NESNE,
     "Kutuların boyutuna bakılarak verilen kaba etiket. 'Yaya adayı' kesin karar değildir, "
     "boy ve taban ölçüsüne göre bir tahmindir."),
    ("takip", "Takip", NESNE,
     "Nesneler kareden kareye eşleştirilip numara alır; hız, yaklaşma hızı ve çarpışma "
     "süresi bu takipten hesaplanır."),
]

A = Ayar
AYARLAR = [
    # ------------------------------ engel_node ------------------------------
    A("merkez_aci", ENGEL, "Bakış yönü", "°", -180.0, 180.0, 1.0, 0.0,
      "Yelpazenin baktığı yön. 0 = LiDAR'ın önü (RViz'deki beyaz ok), +90 = sol, -90 = sağ, "
      "180 = arka.", "uyari"),
    A("gorus_genisligi", ENGEL, "Görüş genişliği", "°", 10.0, 360.0, 1.0, 90.0,
      "Yelpazenin toplam açısı. 90 → bakış yönünün 45° sağından 45° soluna kadar. "
      "360 → her yön.", "uyari"),
    A("dur_mesafesi", ENGEL, "DUR mesafesi", "m", 0.3, 10.0, 0.1, 2.0,
      "Engel bundan yakınsa karar DUR olur (çizimde ve RViz'de kırmızı yay).", "uyari"),
    A("dikkat_mesafesi", ENGEL, "DİKKAT mesafesi", "m", 0.5, 20.0, 0.1, 5.0,
      "Engel bundan yakınsa karar DİKKAT olur. Bundan uzak her şey GÜVENLİ sayılır "
      "(sarı sınır = yelpazenin boyu).", "uyari"),
    A("min_mesafe", ENGEL, "Yakın sınır", "m", 0.0, 3.0, 0.05, 0.3,
      "LiDAR'a bundan yakın noktalar yok sayılır (aracın kendisi, masadaki eşyalar). "
      "Masada test ederken ~1.0 yapın.", "filtre"),
    A("z_min", ENGEL, "Alt sınır (zemin)", "m", -3.0, 0.0, 0.05, -1.2,
      "LiDAR'ın merkezinden bu kadar AŞAĞIDAKİ noktalar yok sayılır. ZEMİNİN biraz ÜSTÜNDE "
      "olmalı, yoksa yer engel sanılır! Masada (yer ≈ -0.85 m) → -0.6; araçta LiDAR yerden "
      "1.3 m yüksekteyse → -1.1.", "filtre"),
    A("z_max", ENGEL, "Üst sınır (tavan)", "m", 0.0, 3.0, 0.05, 1.0,
      "LiDAR'ın merkezinden bu kadar YUKARIDAKİ noktalar yok sayılır (tavan, ağaç dalı, köprü).",
      "filtre"),
    A("min_nokta_sayisi", ENGEL, "En az nokta sayısı", "nokta", 1, 100, 1, 10,
      "Görüş alanında bundan AZ nokta varsa engel yok sayılır; toz, yağmur, yansıma gibi tek tük "
      "noktalar karar verdirmez. Mesafe de en yakın bu kadarıncı noktaya göre ölçülür. Yanlış DUR "
      "alıyorsanız artırın (20-30); küçük cisimleri kaçırıyorsanız azaltın.", "filtre", "tam"),
    A("canli_izleme", ENGEL, "Her turu program çıktısına yaz", "", 0, 1, 1, True,
      "Açıkken engel_node her turda kararı ve mesafeyi yazar (alttaki 'Program çıktıları'nda "
      "görünür).", "filtre", "mantik"),

    # ------------------------------ nesne_node ------------------------------
    A("nesne_merkez_aci", NESNE, "Kutulama yönü", "°", -180.0, 180.0, 1.0, 0.0,
      "Nesnelerin kutu içine alınacağı alanın baktığı yön. 0 = ön, +90 = sol, -90 = sağ, "
      "180 = arka.", "kutulama"),
    A("nesne_gorus_genisligi", NESNE, "Kutulama genişliği", "°", 10.0, 360.0, 1.0, 120.0,
      "Kutulama alanının toplam açısı. 360 → her yöndeki nesneler kutulanır.", "kutulama"),
    A("menzil", NESNE, "Kutulama menzili", "m", 1.0, 50.0, 0.5, 20.0,
      "Bundan uzaktaki nesneler kutulanmaz. Uzaktaki küçük parçalar ekranı kalabalıklaştırıyorsa "
      "azaltın.", "kutulama"),
    A("voxel_boyu", NESNE, "Seyreltme küpü", "m", 0.0, 0.5, 0.01, 0.08,
      "Noktalar bu boyda küplere toplanır, her küpten tek nokta kalır. Büyütmek hızlandırır ama "
      "küçük cisimler kaybolabilir. 0 = kapalı.", "kutulama"),
    A("govde_x_min", NESNE, "Gövde kutusu: arka sınır", "m", -10.0, 10.0, 0.05, 0.0,
      "Gövde kutusunun içindeki noktalar yok sayılır (aracın / masanın kendisi). x = ileri. "
      "Dört sınırın hepsi 0 → kutu kapalı.", "kutulama"),
    A("govde_x_max", NESNE, "Gövde kutusu: ön sınır", "m", -10.0, 10.0, 0.05, 0.0,
      "Gövde kutusunun LiDAR'ın önündeki sınırı (x).", "kutulama"),
    A("govde_y_min", NESNE, "Gövde kutusu: sağ sınır", "m", -10.0, 10.0, 0.05, 0.0,
      "Gövde kutusunun sağ sınırı (y; sağ taraf eksidir).", "kutulama"),
    A("govde_y_max", NESNE, "Gövde kutusu: sol sınır", "m", -10.0, 10.0, 0.05, 0.0,
      "Gövde kutusunun sol sınırı (y).", "kutulama"),
    A("statik_gizle", NESNE, "Duran nesneleri gizle", "", 0, 1, 1, False,
      "Açıkken hiç hareket etmemiş nesneler (duvar, masa, dolap…) kutulanmaz ve listede görünmez; "
      "sadece hareket eden nesneler (yayalar) ve uyarı alanının içindeki duran engeller gösterilir. "
      "Takip arkada sürer: duran bir nesne yürümeye başlarsa hemen görünür; 5 sn yerinde duran "
      "nesne tekrar gizlenir. DUR / DİKKAT kararını (engel_node) etkilemez.",
      "kutulama", "mantik"),
    A("hareket_mesafesi", NESNE, "Hareketli sayılma mesafesi", "m", 0.2, 3.0, 0.05, 0.8,
      "Nesne bundan fazla yer değiştirince hareketli sayılır (5 sn yerinde durursa tekrar duran). "
      "Duran nesneler yanlışlıkla görünüyorsa büyütün, yayalar geç görünüyorsa küçültün.",
      "kutulama"),

    A("zemin_otomatik", NESNE, "Otomatik zemin bulma", "", 0, 1, 1, True,
      "Açıkken zemin açılışta ipucusuz aranır: LiDAR'ın altındaki en alçak geniş yatay yüzey "
      "(masa, tezgâh, kaldırım hep zeminin üstündedir). Kapalıyken sadece beklenen yüksekliğin "
      "± arama bandı içinde aranır.", "zemin", "mantik"),
    A("zemin_z", NESNE, "Beklenen zemin yüksekliği", "m", -4.0, -0.3, 0.01, -0.81,
      "Zeminin LiDAR'a göre yüksekliği: LiDAR'ın yerden yüksekliğinin eksilisi. Otomatik açıkken "
      "sadece kontrol için kullanılır (ve zemin hiç bulunamazsa bu yükseklik varsayılır).", "zemin"),
    A("zemin_tolerans", NESNE, "Zemin kontrolü: yükseklik farkı", "m", 0.02, 1.0, 0.01, 0.10,
      "Bulunan zemin beklenenden bundan fazla farklıysa uyarı çıkar (LiDAR oynamış ya da yanlış "
      "profil yüklü olabilir).", "zemin"),
    A("egim_uyari", NESNE, "Zemin kontrolü: en fazla eğim", "°", 0.5, 30.0, 0.5, 3.0,
      "Bulunan zemin bundan fazla eğikse uyarı çıkar (LiDAR yamuk duruyor olabilir).", "zemin"),
    A("zemin_bant", NESNE, "Zemin arama bandı", "m", 0.05, 1.5, 0.01, 0.30,
      "Zemin her turda bir önceki zeminin ± bu kadar yakınında aranır (otomatik kapalıysa ilk "
      "aramada da beklenen yüksekliğin ± bu kadarı). Yol eğimliyse biraz büyütün.", "zemin"),
    A("zemin_esik", NESNE, "Zemin kalınlığı", "m", 0.02, 0.5, 0.01, 0.10,
      "Zemin düzlemine bundan yakın noktalar zemin sayılır. Büyütürseniz alçak cisimler (kaldırım "
      "taşı) zemine karışabilir.", "zemin"),
    A("zemin_max_egim", NESNE, "En fazla zemin eğimi", "°", 1.0, 30.0, 0.5, 10.0,
      "Bundan dik bir düzlem zemin sayılmaz (duvarı zemin sanmasın diye).", "zemin"),
    A("zemin_min_nokta", NESNE, "Zemin için en az nokta", "nokta", 3, 5000, 1, 30,
      "Zemin düzleminin kabul edilmesi için gereken en az nokta.", "zemin", "tam"),
    A("max_yukseklik", NESNE, "Üst sınır (zeminden)", "m", 0.5, 6.0, 0.1, 2.5,
      "Zeminden bu kadar yukarısı (tavan, dal, köprü) yok sayılır.", "zemin"),

    A("kume_yatay_tol", NESNE, "Yatay birleştirme mesafesi", "m", 0.05, 2.0, 0.01, 0.35,
      "İki nokta yatayda bundan yakınsa aynı nesne sayılır.", "kume"),
    A("kume_dikey_taban", NESNE, "Dikey birleştirme (taban)", "m", 0.05, 2.0, 0.01, 0.25,
      "Dikey aralık eşiği = taban + artış × mesafe. Yakındaki nesneler için taban belirleyicidir.",
      "kume"),
    A("kume_dikey_artis", NESNE, "Dikey birleştirme (artış)", "m/m", 0.0, 0.2, 0.005, 0.04,
      "Uzaktaki nesnelerin LiDAR katmanları birbirinden açık olur; eşik her metrede bu kadar büyür.",
      "kume"),
    A("kume_min_nokta", NESNE, "Nesne için en az nokta", "nokta", 1, 1000, 1, 5,
      "Bundan az noktalı kümeler gürültü sayılır.", "kume", "tam"),
    A("kume_max_nokta", NESNE, "Nesne için en fazla nokta", "nokta", 10, 200000, 10, 20000,
      "Bundan çok noktalı kümeler atılır.", "kume", "tam"),

    A("yaya_min_tepe", NESNE, "Yaya: en kısa boy", "m", 0.3, 3.0, 0.05, 1.0,
      "Yaya adayı: en üst noktası zeminden en az bu kadar yüksek olmalı.", "etiket"),
    A("yaya_max_tepe", NESNE, "Yaya: en uzun boy", "m", 0.5, 3.0, 0.05, 2.1,
      "Yaya adayı: en üst noktası zeminden en fazla bu kadar yüksek olabilir.", "etiket"),
    A("yaya_max_uzunluk", NESNE, "Yaya: en fazla taban uzunluğu", "m", 0.2, 3.0, 0.05, 1.2,
      "Yaya adayı kutusunun taban uzunluğu en fazla bu kadar olabilir.", "etiket"),
    A("yaya_max_genislik", NESNE, "Yaya: en fazla taban genişliği", "m", 0.2, 3.0, 0.05, 1.0,
      "Yaya adayı kutusunun taban genişliği en fazla bu kadar olabilir.", "etiket"),
    A("yapi_min_uzunluk", NESNE, "Yapı: en az uzunluk", "m", 0.5, 20.0, 0.1, 2.5,
      "Yapı (duvar, çit): en az bu kadar uzun ve aşağıdaki kadar ince olmalı.", "etiket"),
    A("yapi_max_genislik", NESNE, "Yapı: en fazla kalınlık", "m", 0.05, 3.0, 0.05, 0.5,
      "Yapı sayılması için kutu en fazla bu kadar kalın olabilir.", "etiket"),
    A("yapi_kesin_uzunluk", NESNE, "Yapı: kesin uzunluk", "m", 1.0, 50.0, 0.5, 6.0,
      "Bundan uzun her şey kalınlığına bakılmadan yapı sayılır.", "etiket"),

    A("takip_kapi", NESNE, "Bir turda en fazla yer değiştirme", "m", 0.1, 5.0, 0.05, 1.0,
      "Bir nesne iki tur arasında bundan fazla yer değiştirirse aynı nesne sayılmaz. Hızlı "
      "nesneleri kaybediyorsanız büyütün.", "takip"),
    A("kutu_yumusatma", NESNE, "Kutu yumuşatma", "", 0.0, 0.95, 0.05, 0.7,
      "Kutunun kareden kareye titremesini azaltır: merkez takipten, boyut ve yön yavaşça "
      "güncellenir. 0 = kapalı (ham kutu), 0.9 = çok yumuşak (değişime geç tepki). Mesafe ve en "
      "yakın nokta yumuşatılmaz, her zaman o turun ölçümüdür.", "takip"),
    A("takip_dogrulama", NESNE, "Numara için gereken tur", "tur", 1, 20, 1, 3,
      "Yeni bir nesne bu kadar tur üst üste görülünce numara alır (tek turluk gürültü numara "
      "almaz).", "takip", "tam"),
    A("takip_kayip", NESNE, "Silinmeden önce bekleme", "tur", 1, 50, 1, 5,
      "Bu kadar tur görülmeyen nesne silinir.", "takip", "tam"),
    A("nesne_canli_izleme", NESNE, "Saniyede bir özet yaz", "", 0, 1, 1, True,
      "Açıkken nesne_node saniyede bir özet satırı yazar (alttaki 'Program çıktıları'nda görünür).",
      "takip", "mantik", ros_adi="canli_izleme"),
]
del A

TANIM = {a.anahtar: a for a in AYARLAR}
NODE_AYARLARI = {ENGEL: [a for a in AYARLAR if a.node == ENGEL],
                 NESNE: [a for a in AYARLAR if a.node == NESNE]}
GOVDE = ("govde_x_min", "govde_x_max", "govde_y_min", "govde_y_max")


def _paket_paylasim():
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory("lidar_algilama")  # varsayılan parametre dosyaları
    except Exception:  # paket kurulu değil / ROS ortamı yok
        return None


def varsayilan_dosyasi(node):
    d = _paket_paylasim()
    if d is None:
        return None
    return os.path.join(d, "config", "engel_params.yaml" if node == ENGEL else "nesne_params.yaml")


def _varsayilanlari_oku():
    """Varsayılanlar paketin config/*.yaml dosyalarından (tek doğru kaynak)."""
    for node in (ENGEL, NESNE):
        yol = varsayilan_dosyasi(node)
        if not yol:
            continue
        try:
            with open(yol, encoding="utf-8") as f:
                bolum = (yaml.safe_load(f) or {})[node]["ros__parameters"]
        except (OSError, yaml.YAMLError, KeyError, TypeError):
            continue
        for a in NODE_AYARLARI[node]:
            if a.ros_adi not in bolum:
                continue
            v = bolum[a.ros_adi]
            if a.tip == "mantik":
                if isinstance(v, bool):
                    a.varsayilan = v
            elif isinstance(v, (int, float)) and a.alt <= v <= a.ust:
                a.varsayilan = a.ros_degeri(v)


_varsayilanlari_oku()


def varsayilanlar():
    return {a.anahtar: a.varsayilan for a in AYARLAR}


def dosya_yaz(yol, degerler):
    """Ayarları ROS parametre dosyası biçiminde yazar (her node kendi bölümünde)."""
    os.makedirs(os.path.dirname(os.path.abspath(yol)), exist_ok=True)
    icerik = {}
    for node in (ENGEL, NESNE):
        for a in NODE_AYARLARI[node]:
            if a.anahtar in degerler:
                icerik.setdefault(node, {"ros__parameters": {}})
                icerik[node]["ros__parameters"][a.ros_adi] = a.ros_degeri(degerler[a.anahtar])
    gecici = yol + ".tmp"
    with open(gecici, "w", encoding="utf-8") as f:
        f.write("# lidar_algilama ayar dosyasi (Kontrol Merkezi / ayar paneli tarafindan yazildi)\n")
        yaml.safe_dump(icerik, f, allow_unicode=True, sort_keys=False)
    os.replace(gecici, yol)  # yarım yazılmış dosya kalmasın diye önce geçiciye yaz


def dosya_oku(yol):
    """
    Ayar dosyasını okur ve kontrol eder. Başarılıysa (degerler, None),
    hatalıysa (None, hata_metni) döndürür. Dosyada olmayan ayarlar sözlükte yer almaz.
    """
    try:
        with open(yol, encoding="utf-8") as f:
            veri = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError) as e:
        return None, f"Dosya okunamadı: {e}"
    if not isinstance(veri, dict):
        return None, "Dosya biçimi tanınmadı."

    # ROS biçimi: {engel_node: {ros__parameters: {...}}, nesne_node: {...}} ya da {/**: {...}}.
    # Bölümsüz düz sözlük de kabul edilir (her iki node'a uygulanır).
    bolumler, genel = {}, {}
    for anahtar, deger in veri.items():
        ad = str(anahtar).lstrip("/")
        if isinstance(deger, dict) and isinstance(deger.get("ros__parameters"), dict):
            if ad == "**":
                genel = deger["ros__parameters"]
            elif ad in (ENGEL, NESNE):
                bolumler[ad] = deger["ros__parameters"]
    if not bolumler and not genel:
        genel = veri

    degerler = {}
    for node in (ENGEL, NESNE):
        kaynak = dict(genel)
        kaynak.update(bolumler.get(node, {}))
        for a in NODE_AYARLARI[node]:
            if a.ros_adi not in kaynak:
                continue
            v = kaynak[a.ros_adi]
            if a.tip == "mantik":
                if not isinstance(v, bool):
                    return None, f"'{a.ros_adi}' true/false olmalı: {v!r}"
                degerler[a.anahtar] = v
                continue
            if isinstance(v, bool) or not isinstance(v, (int, float)):
                return None, f"'{a.ros_adi}' sayı değil: {v!r}"
            if not a.alt <= v <= a.ust:
                return None, f"'{a.ros_adi}' = {v:g} izin verilen aralığın ({a.alt:g} .. {a.ust:g}) dışında."
            degerler[a.anahtar] = a.ros_degeri(v)
    if not degerler:
        return None, "Dosyada engel_node / nesne_node ayarı bulunamadı."
    return degerler, None


def son_ayarlari_oku():
    """son_ayarlar.yaml'daki değerler (yoksa / bozuksa boş sözlük)."""
    if not os.path.exists(SON_AYARLAR):
        return {}
    degerler, _ = dosya_oku(SON_AYARLAR)
    return degerler or {}


# Birbiriyle ilişkili ayarların kuralları: (anahtarlar, kontrol, mesaj).
# engel_node.cpp ve nesne_node.cpp'deki kontrollerin aynısı.
KURALLAR = [
    (("dur_mesafesi", "dikkat_mesafesi"), lambda d: d["dur_mesafesi"] < d["dikkat_mesafesi"],
     "DUR mesafesi, DİKKAT mesafesinden küçük olmalı."),
    (("min_mesafe", "dur_mesafesi"), lambda d: d["min_mesafe"] < d["dur_mesafesi"],
     "Yakın sınır, DUR mesafesinden küçük olmalı."),
    (("z_min", "z_max"), lambda d: d["z_min"] < d["z_max"],
     "Alt sınır (zemin), üst sınırdan (tavan) küçük olmalı."),
    (("govde_x_min", "govde_x_max"), lambda d: d["govde_x_min"] <= d["govde_x_max"],
     "Gövde kutusunun arka sınırı, ön sınırından büyük olamaz."),
    (("govde_y_min", "govde_y_max"), lambda d: d["govde_y_min"] <= d["govde_y_max"],
     "Gövde kutusunun sağ sınırı, sol sınırından büyük olamaz."),
    (("kume_min_nokta", "kume_max_nokta"), lambda d: d["kume_min_nokta"] <= d["kume_max_nokta"],
     "Nesne için en az nokta, en fazla noktadan büyük olamaz."),
    (("yaya_min_tepe", "yaya_max_tepe"), lambda d: d["yaya_min_tepe"] < d["yaya_max_tepe"],
     "Yaya en kısa boyu, en uzun boyundan küçük olmalı."),
    (("yapi_min_uzunluk", "yapi_kesin_uzunluk"), lambda d: d["yapi_min_uzunluk"] < d["yapi_kesin_uzunluk"],
     "Yapı en az uzunluğu, kesin uzunluktan küçük olmalı."),
]


def kontrol_et(d):
    """Kurala uymayanlar: [(mesaj, (anahtarlar...)), ...]. Hepsi uyuyorsa boş liste."""
    hatalar = []
    for anahtarlar, kural, mesaj in KURALLAR:
        if all(k in d for k in anahtarlar) and not kural(d):
            hatalar.append((mesaj, anahtarlar))
    return hatalar


def ayarlar_ayni_mi(a, b):
    """İki ayar sözlüğü, ortak anahtarlarda aynı değerde mi? (profil 'değişti' kontrolü)"""
    for k, v in a.items():
        if k in b:
            if TANIM[k].tip == "mantik":
                if bool(v) != bool(b[k]):
                    return False
            elif abs(float(v) - float(b[k])) > 1e-6:
                return False
    return True


# -----------------------------------------------------------------------------
# Kontrol Merkezi'nin kendi ayarları
# -----------------------------------------------------------------------------
GENEL_VARSAYILAN = {
    "kayit_klasoru": "~/robotaksi_kayitlar",
    "rviz_otomatik": True,      # bölüme girince RViz de açılsın
    "yan_yana": True,           # RViz'i arayüzün yanına yerleştir (ekranı ikiye böl)
    "nesne_acik": True,         # nesne_node (kutular) varsayılan olarak açık
    "tema": "koyu",
    "sikistirma": "zstd_fast",  # kayıt sıkıştırması
    "son_profil": "",           # en son yüklenen / kaydedilen profil
    "son_kayit": "",            # en son oynatılan kayıt
}


class GenelAyarlar:
    def __init__(self):
        self.d = dict(GENEL_VARSAYILAN)
        try:
            with open(ARAYUZ_AYARLARI, encoding="utf-8") as f:
                veri = yaml.safe_load(f) or {}
            if isinstance(veri, dict):
                for k, v in veri.items():
                    if k in self.d and type(v) is type(self.d[k]):
                        self.d[k] = v
        except (OSError, yaml.YAMLError):
            pass

    def __getitem__(self, k):
        return self.d[k]

    def __setitem__(self, k, v):
        self.d[k] = v
        self.kaydet()

    def guncelle(self, **degerler):
        self.d.update(degerler)
        self.kaydet()

    @property
    def kayit_klasoru(self):
        return os.path.expanduser(self.d["kayit_klasoru"])

    def kaydet(self):
        try:
            os.makedirs(KAYIT_KOKU, exist_ok=True)
            gecici = ARAYUZ_AYARLARI + ".tmp"
            with open(gecici, "w", encoding="utf-8") as f:
                f.write("# Kontrol Merkezi ayarlari\n")
                yaml.safe_dump(self.d, f, allow_unicode=True, sort_keys=False)
            os.replace(gecici, ARAYUZ_AYARLARI)
        except OSError:
            pass
