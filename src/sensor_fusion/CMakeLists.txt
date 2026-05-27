cmake_minimum_required(VERSION 3.8)
project(sensor_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

add_library(multi_rate_ekf SHARED src/multi_rate_ekf.cpp)
ament_target_dependencies(multi_rate_ekf rclcpp sensor_msgs nav_msgs geometry_msgs)
target_link_libraries(multi_rate_ekf Eigen3::Eigen)

add_library(multi_rate_ekf_2d SHARED src/multi_rate_ekf_2d.cpp)
ament_target_dependencies(multi_rate_ekf_2d rclcpp)
target_link_libraries(multi_rate_ekf_2d Eigen3::Eigen)

add_library(imu_prefilter SHARED src/imu_prefilter.cpp)
ament_target_dependencies(imu_prefilter rclcpp)
target_link_libraries(imu_prefilter Eigen3::Eigen)

add_library(lidar_module SHARED src/lidar_module.cpp)
ament_target_dependencies(lidar_module rclcpp)
target_link_libraries(lidar_module Eigen3::Eigen)

add_library(vision_module SHARED src/vision_module.cpp)
ament_target_dependencies(vision_module rclcpp)
target_link_libraries(vision_module Eigen3::Eigen ${OpenCV_LIBS})

add_library(graph_optimizer SHARED src/graph_optimizer.cpp)
ament_target_dependencies(graph_optimizer rclcpp)
target_link_libraries(graph_optimizer Eigen3::Eigen)

add_executable(fusion_node src/fusion_node.cpp)
ament_target_dependencies(fusion_node rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs)
target_link_libraries(fusion_node multi_rate_ekf imu_prefilter lidar_module vision_module graph_optimizer Eigen3::Eigen ${OpenCV_LIBS})

add_executable(sensor_fusion_node src/sensor_fusion_node.cpp)
ament_target_dependencies(sensor_fusion_node rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs)
target_link_libraries(sensor_fusion_node multi_rate_ekf_2d imu_prefilter Eigen3::Eigen)

install(TARGETS
  multi_rate_ekf
  multi_rate_ekf_2d
  imu_prefilter
  lidar_module
  vision_module
  graph_optimizer
  fusion_node
  sensor_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
