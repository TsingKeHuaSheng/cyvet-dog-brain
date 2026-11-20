cmake_minimum_required(VERSION 3.8)
project(livox_pc2_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

add_executable(livox_pc2_bridge_node src/main.cpp)
ament_target_dependencies(livox_pc2_bridge_node rclcpp sensor_msgs livox_ros_driver2)

install(TARGETS livox_pc2_bridge_node DESTINATION lib/${PROJECT_NAME})
ament_package()
