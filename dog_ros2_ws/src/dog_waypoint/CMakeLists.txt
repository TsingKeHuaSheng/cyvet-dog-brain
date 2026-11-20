cmake_minimum_required(VERSION 3.8)
project(dog_waypoint)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(uniubi_motion_bridge REQUIRED)

# ---- 航点跟随（核心）----
add_executable(waypoint_follower src/waypoint_follower.cpp)
ament_target_dependencies(waypoint_follower
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_srvs
  uniubi_motion_bridge)

# ---- 航点录制（教一次）----
add_executable(waypoint_recorder src/waypoint_recorder.cpp)
ament_target_dependencies(waypoint_recorder
  rclcpp
  nav_msgs)

# ---- /odom -> TF（上 Nav2 / RViz 时必需）----
add_executable(odom_to_tf src/odom_to_tf.cpp)
ament_target_dependencies(odom_to_tf
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros)

install(TARGETS waypoint_follower waypoint_recorder odom_to_tf
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY paths
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
