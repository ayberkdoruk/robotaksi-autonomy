cmake_minimum_required(VERSION 3.8)
project(kontrol_merkezi)

# Derlenecek kod yok: Python arayüzleri ve betikler kurulur.
find_package(ament_cmake REQUIRED)

# Python arayüzleri:
#   kontrol_merkezi : Robotaksi Kontrol Merkezi — her şey tek pencerede (masaüstü simgesi)
#   ayar_paneli     : tek başına engel ayar paneli (lidar_sistem.launch.py açar)
# İkisi de arayuz/ paketini kullanır; paket programların yanına kurulur.
install(PROGRAMS scripts/kontrol_merkezi.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME kontrol_merkezi
)
install(PROGRAMS scripts/ayar_paneli.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ayar_paneli
)
install(DIRECTORY scripts/arayuz
  DESTINATION lib/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

# Betikler (masaüstü kısayolları bunları çalıştırır)
install(PROGRAMS
  scripts/kontrol_merkezi_baslat.sh
  scripts/engel_sistemi_baslat.sh
  scripts/lidar_kayit.sh
  scripts/lidar_oynat.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
