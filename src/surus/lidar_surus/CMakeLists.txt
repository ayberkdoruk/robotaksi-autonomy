cmake_minimum_required(VERSION 3.8)
project(lidar_surus)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robotaksi_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# lidar_surus_node: düz git, engel / yaya görünce dur (/engel/durum + /nesne/liste -> /arac/komut)
add_executable(lidar_surus_node src/lidar_surus_node.cpp)
ament_target_dependencies(lidar_surus_node rclcpp std_msgs robotaksi_msgs nav_msgs sensor_msgs)

install(TARGETS lidar_surus_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
