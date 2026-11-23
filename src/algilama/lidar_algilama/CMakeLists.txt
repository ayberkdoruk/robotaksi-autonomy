cmake_minimum_required(VERSION 3.8)
project(lidar_algilama)

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
find_package(robotaksi_msgs REQUIRED)

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
  robotaksi_msgs
)

# Çalıştırılabilir dosyayı "ros2 run" bulabilsin diye lib/<paket_adı> altına kur
install(TARGETS engel_node nesne_node
  DESTINATION lib/${PROJECT_NAME}
)

# Parametre dosyaları ve launch dosyası share/<paket_adı> altına
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
