cmake_minimum_required(VERSION 3.8)
project(sentinel_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(imu_prefilter_node src/imu_prefilter/imu_prefilter_node.cpp)
ament_target_dependencies(imu_prefilter_node
  rclcpp
  rclcpp_components
  sensor_msgs
)
target_compile_features(imu_prefilter_node PUBLIC c_std_99 cxx_std_17)

add_executable(point_lio_wrapper_node src/lidar_ekf/point_lio_wrapper_node.cpp)
ament_target_dependencies(point_lio_wrapper_node
  rclcpp
  nav_msgs
  sensor_msgs
)
target_compile_features(point_lio_wrapper_node PUBLIC c_std_99 cxx_std_17)

add_executable(vision_ekf_node src/vision_ekf/vision_ekf_node.cpp)
ament_target_dependencies(vision_ekf_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(vision_ekf_node Eigen3::Eigen)
target_compile_features(vision_ekf_node PUBLIC c_std_99 cxx_std_17)

add_executable(graph_optimizer_node src/graph_optimizer/graph_optimizer_node.cpp)
ament_target_dependencies(graph_optimizer_node
  rclcpp
  nav_msgs
  sensor_msgs
)
target_link_libraries(graph_optimizer_node Eigen3::Eigen)
target_compile_features(graph_optimizer_node PUBLIC c_std_99 cxx_std_17)

install(TARGETS
  imu_prefilter_node
  point_lio_wrapper_node
  vision_ekf_node
  graph_optimizer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
