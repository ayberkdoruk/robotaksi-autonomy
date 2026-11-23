// =============================================================================
// kutu.hpp — Kümeyi saran dönük kutu, mesafe ve kaba etiket
//
// Kutu z ekseni etrafında dönebilir: çapraz duran bir duvar veya araç, eksene
// hizalı bir kutuya konsaydı gerçekte olduğundan çok daha büyük görünürdü.
// =============================================================================
#pragma once

#include <string>
#include <vector>

#include "nesne/tipler.hpp"

namespace nesne
{

// Kümedeki noktalardan kutu ve ölçümleri hesaplar.
//   indeks: kümenin buluttaki nokta indeksleri (boş olmamalı)
//   zemin : tepe noktasının zeminden yüksekliğini bulmak için
Kutu kutuHesapla(const Bulut & bulut, const std::vector<int> & indeks, const Duzlem & zemin);

struct EtiketAyari
{
  float yaya_min_tepe = 1.0f;       // yaya adayı: tepe noktası zeminden en az ...
  float yaya_max_tepe = 2.1f;       // ... en fazla bu kadar yüksek
  float yaya_max_uzunluk = 1.2f;    // ve kutu bu kadar uzun ...
  float yaya_max_genislik = 1.0f;   // ... ve bu kadar geniş olabilir
  float yapi_min_uzunluk = 2.5f;    // yapı: bu kadar uzun ...
  float yapi_max_genislik = 0.5f;   // ... ve bu kadar ince (duvarın tek yüzü)
  float yapi_kesin_uzunluk = 6.0f;  // ya da bundan uzunsa genişliğe bakmadan yapı
};

// Etiketler. Bu sadece kutu şekline göre bir TAHMİN; kesin kararı ileride kamera verecek.
inline const char * const ETIKET_YAYA = "yaya_adayi";
inline const char * const ETIKET_YAPI = "yapi";
inline const char * const ETIKET_ENGEL = "engel";

std::string etiketVer(const Kutu & k, const EtiketAyari & a);

}  // namespace nesne
