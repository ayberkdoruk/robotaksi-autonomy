cmake_minimum_required(VERSION 3.8)
project(arac_haberlesme)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robotaksi_msgs REQUIRED)

# serial_comm_node: Teensy'ye yazan TEK düğüm (/arac/komut + /gamepad/state -> 5 byte paket)
add_executable(serial_comm_node src/serial_comm_node.cpp)
ament_target_dependencies(serial_comm_node rclcpp std_msgs robotaksi_msgs)

install(TARGETS serial_comm_node
  DESTINATION lib/${PROJECT_NAME}
)

# gamepad_node: Logitech F710'u okur, /gamepad/state yayınlar (seri porta yazmaz)
install(PROGRAMS scripts/gamepad_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gamepad_node
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
