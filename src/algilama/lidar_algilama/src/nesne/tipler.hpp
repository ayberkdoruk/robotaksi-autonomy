// =============================================================================
// tipler.hpp — nesne algılama modüllerinin ortak tipleri
//
// Bu klasördeki (src/nesne/) kodların hiçbiri ROS'a bağlı değildir: sadece
// PCL nokta bulutu alır, sonuç döndürür. ROS tarafı (parametreler, topic'ler,
// RViz) nesne_node.cpp'de. Böylece algoritmalar tek başına okunabilir kalır.
//
// Eksenler (LiDAR'a göre): x = ileri, y = sol, z = yukarı, birim metre.
// =============================================================================
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace nesne
{

using Nokta = pcl::PointXYZ;
using Bulut = pcl::PointCloud<Nokta>;

// Zemin düzlemi:  a*x + b*y + c*z + d = 0
// (a, b, c) birim uzunlukta ve yukarı bakar (c > 0). Bu sayede bir noktayı
// denkleme koyunca çıkan sayı, o noktanın zeminden yüksekliğidir (metre).
struct Duzlem
{
  float a = 0.0f;
  float b = 0.0f;
  float c = 1.0f;
  float d = 0.0f;
  bool gecerli = false;

  float yukseklik(const Nokta & p) const {return a * p.x + b * p.y + c * p.z + d;}
  // Düzlemin LiDAR'ın tam altındaki yüksekliği (x = y = 0 iken z)
  float merkezZ() const {return -d / c;}
};

// Bir nesneyi saran, z ekseni etrafında dönük kutu ve ölçümleri
struct Kutu
{
  // Kutu merkezi ve boyutları (uzunluk >= genişlik)
  float cx = 0.0f, cy = 0.0f, cz = 0.0f;
  float uzunluk = 0.0f, genislik = 0.0f, yukseklik = 0.0f;
  float yon = 0.0f;  // uzunluk kenarının x ekseniyle açısı (radyan, -90°..+90°)

  // LiDAR'a en yakın nokta ve onun yatay mesafesi (engel_node'daki "r" ile aynı tanım)
  float yakin_x = 0.0f, yakin_y = 0.0f, yakin_z = 0.0f;
  float mesafe = 0.0f;

  float tepe_yuksekligi = 0.0f;  // en üst noktanın ZEMİNDEN yüksekliği
  float alt_z = 0.0f, ust_z = 0.0f;  // kutunun alt ve üst yüzü (LiDAR frame'inde z)
  int nokta_sayisi = 0;
};

}  // namespace nesne
