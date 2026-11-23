"""Robotaksi Kontrol Merkezi ve engel ayar paneli için ortak arayüz paketi.

Modüller:
  tema         renkler ve Qt stil sayfası (koyu / açık)
  ikonlar      kodla çizilen düğme ve kart simgeleri
  yardimci     açı, süre, boyut ve tarih metinleri
  ayar_tanim   engel_node / nesne_node ayar tanımları, ayar dosyaları, genel arayüz ayarları
  ros_baglanti arayüzün ROS bağlantısı (abonelikler, parametre ve oynatıcı istemcileri)
  surecler     arka planda çalışan programları (sürücü, node'lar, RViz, kayıt) yönetir
  lidar_bilgi  sürücü ayar dosyası, LiDAR paket kontrolü, disk durumu
  kayitlar     rosbag kayıtlarını listeleme ve kayıt alma
  bilesenler   ortak arayüz parçaları
  ayar_paneli  parametre sekmeleri + kayıtlı ayar profilleri
  algilama     karar kutusu, yukarıdan bakış çizimi, nesne tablosu
  ana_ekran    açılış ekranı
  bolumler     Engel Algılama / LiDAR'ı İzle / Kayıt Al / Kayıt Oynat sayfaları
  uygulama     Kontrol Merkezi ana penceresi
  panel        tek başına çalışan engel ayar paneli (lidar_sistem.launch.py açar)
"""
