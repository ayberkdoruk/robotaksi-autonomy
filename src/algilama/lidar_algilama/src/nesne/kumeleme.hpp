// =============================================================================
// kumeleme.hpp — Zemin dışı noktaları nesnelere (kümelere) ayırır
//
// Fikir: birbirine yeterince yakın noktalar aynı nesnedir. Bir noktadan başla,
// yakınındaki noktaları ekle, onların yakınındakileri ekle... ("bölge büyütme").
//
// Helios 32'nin bir özelliği: katmanlar (ring'ler) uzaklaştıkça dikeyde birbirinden
// açılır. 10 m'deki bir insanın üst üste iki katmanı arasında ~0.3-0.4 m boşluk olur.
// Sabit bir "yakınlık" eşiği ya uzaktaki insanı parçalar ya da yakındaki iki
// nesneyi birleştirir. Bu yüzden:
//   - yatay eşik SABİT  (yan yana duran iki yaya birleşmesin)
//   - dikey eşik mesafeyle BÜYÜR  (tek yayanın katmanları birleşsin)
// =============================================================================
#pragma once

#include <vector>

#include "nesne/tipler.hpp"

namespace nesne
{

struct KumeAyari
{
  float yatay_tol = 0.35f;    // iki nokta arasındaki en fazla yatay mesafe (metre)
  float dikey_taban = 0.25f;  // dikey eşik = dikey_taban + dikey_artis * r
  float dikey_artis = 0.04f;  // (r: noktanın LiDAR'a yatay mesafesi)
  int min_nokta = 5;          // bundan az noktalı küme gürültü sayılır
  int max_nokta = 20000;      // bundan çok noktalı küme atılır
};

// Her küme, buluttaki nokta indekslerinin listesidir.
std::vector<std::vector<int>> kumele(const Bulut::ConstPtr & bulut, const KumeAyari & a);

}  // namespace nesne
