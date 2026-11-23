cmake_minimum_required(VERSION 3.8)
project(engel_algilama)

# C++17 standardını kullan
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Derleyici uyarılarını aç (hataları erken görmek için faydalı)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Python arayüzleri (derleme gerekmez):
#   engel_ayar_paneli : tek başına ayar paneli (launch dosyaları açar)
#   lidar_kontrol     : LiDAR Kontrol Merkezi — her şey tek pencerede (masaüstü simgesi)
# İkisi de lidar_arayuz/ paketini kullanır; paket programların yanına kurulur.
install(PROGRAMS scripts/engel_ayar_paneli.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME engel_ayar_paneli
)
install(PROGRAMS scripts/lidar_kontrol.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME lidar_kontrol
)
install(PROGRAMS scripts/lidar_kontrol_baslat.sh
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY scripts/lidar_arayuz
  DESTINATION lib/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

# config, launch, rviz ve simge klasörlerini share/<paket_adı> altına kur
install(DIRECTORY launch rviz resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
