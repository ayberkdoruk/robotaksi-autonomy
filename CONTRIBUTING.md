# Katkı rehberi

Bu çalışma alanı bir ekip tarafından kullanılır: **ortak zemin** herkesin, **sürüş yöntemleri** her
geliştiricinin kendisinindir. Aşağıdaki kurallar, birinin değişikliğinin başkasının yöntemini
bozmaması içindir.

## Nereye ne konur?

| Değişiklik | Yer | Not |
|---|---|---|
| Kendi sürüş yönteminiz | `src/surus/<yontem>_surus/` | Paket adı yöntemi anlatsın (`lidar_surus`), kişiyi değil |
| Yeni algılama paketi | `src/algilama/` | Sonuçlarını topic olarak yayınlar, araca komut vermez |
| Ortak mesaj | `src/ortak/robotaksi_msgs/` | Herkesi etkiler — önce ekiple konuşun |
| Sensör sürücüsü | `src/surucular/` | Dış depo ise `robotaksi.repos`'a sabit sürümle eklenir, değişiklik `yamalar/` altında yama olarak tutulur |
| Araç komut katmanı | `src/arac/` | Herkesi etkiler — önce ekiple konuşun |
| Geliştirme aracı | `src/araclar/` | |

Bağımlılık yönü tek taraflıdır: araçlar → algılama → ortak. Algılama paketleri arayüzleri bilmez.

## Çalışma düzeni

1. `main`'den bir dal açın: `git switch -c nesne-takip-iyilestirme`.
2. Değişikliği yapın ve derleyin:
   ```bash
   colcon build --symlink-install --packages-up-to <paket>
   ```
3. **Gerçek veriyle deneyin**: canlı LiDAR ya da sahada alınmış bir kayıt
   (Kontrol Merkezi → *Kayıt Oynat*). Uydurma / sentetik veriyle doğrulama yapılmaz.
4. Pull request açın; ne değiştiğini ve hangi kayıtla denediğinizi yazın. CI derlemesi geçmelidir.

## Kod düzeni

- İsimler, yorumlar ve dokümantasyon **Türkçe**dir (ROS / kütüphane adları hariç).
- C++17. Algoritmalar ROS'tan bağımsız dosyalarda (`src/nesne/` gibi), ROS tarafı ince bir katmanda.
- Her ayar bir ROS parametresidir; varsayılanı `config/*.yaml`'da, açıklaması paket README'sindeki
  tabloda durur. Yeni parametre eklerken ikisini de güncelleyin.
- Konumlar sensör frame'inde: **x = ileri, y = sol, z = yukarı**, metre; açılar radyan.

## Commit mesajları

Kısa, Türkçe, ne değiştiğini anlatan bir satır; gerekirse altında ayrıntı:

```
nesne_node: duvara yapışık yaya ayrı kümelenir
```
