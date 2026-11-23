cmake_minimum_required(VERSION 3.8)
project(robotaksi_baslat)

# Derlenecek kod yok: ekibin ortak ROS ortamı ve ayarları.
#   scripts/ros_ortami.sh            : masaüstü kısayollarıyla açılan betiklerin yüklediği ortam
#   config/fastdds_buyuk_mesaj.xml   : büyük mesajlar (nokta bulutu) için Fast DDS ayarı
find_package(ament_cmake REQUIRED)

install(DIRECTORY config scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
