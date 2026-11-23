cmake_minimum_required(VERSION 3.8)
project(engel_algilama)

# C++17 standardını kullan
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Derleyici uyarılarını aç (hataları erken görmek için faydalı)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Bağımlılıkları bul (package.xml'deki <depend> satırlarıyla aynı olmalı)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
# nesne_node için: nokta bulutu işleme kütüphanesi ve kendi mesajlarımız
# NOT: PCL derlenirken "CMP0144 / FLANN_ROOT" uyarısı verir; PCL'in kendi dosyasından gelir, zararsızdır.
find_package(PCL REQUIRED COMPONENTS common filters kdtree search sample_consensus segmentation)
find_package(pcl_conversions REQUIRED)
find_package(engel_algilama_msgs REQUIRED)

# engel_node adında çalıştırılabilir dosya oluştur
add_executable(engel_node src/engel_node.cpp)

# Node'u bağımlılıklara bağla (başlık dosyaları + kütüphaneler)
ament_target_dependencies(engel_node
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
  rcl_interfaces
  geometry_msgs
)

# nesne_node: zemin ayırma + kümeleme + kutu + etiket + takip
# Algoritmalar src/nesne/ altında (ROS'tan bağımsız), ROS tarafı nesne_node.cpp'de.
add_executable(nesne_node
  src/nesne_node.cpp
  src/nesne/zemin.cpp
  src/nesne/kumeleme.cpp
  src/nesne/kutu.cpp
  src/nesne/takip.cpp
)
target_include_directories(nesne_node PRIVATE src)
# PCL başlıklarını "sistem" başlığı say: PCL'in kendi içindeki uyarılar bizim
# uyarılarımızın arasında kaybolmasın
target_include_directories(nesne_node SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(nesne_node PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(nesne_node ${PCL_LIBRARIES})
# Derleme tipi verilmese bile optimize et (43 bin nokta x 10 Hz)
target_compile_options(nesne_node PRIVATE -O2)
ament_target_dependencies(nesne_node
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
  rcl_interfaces
  geometry_msgs
  pcl_conversions
  engel_algilama_msgs
)

# Çalıştırılabilir dosyayı "ros2 run" bulabilsin diye lib/<paket_adı> altına kur
install(TARGETS engel_node nesne_node
  DESTINATION lib/${PROJECT_NAME}
)

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
install(FILES scripts/ros_ortami.sh
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY scripts/lidar_arayuz
  DESTINATION lib/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

# config, launch, rviz ve simge klasörlerini share/<paket_adı> altına kur
install(DIRECTORY config launch rviz resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
