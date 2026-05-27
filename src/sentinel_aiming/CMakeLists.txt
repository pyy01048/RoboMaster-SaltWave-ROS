cmake_minimum_required(VERSION 3.8)
project(sentinel_aiming)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sentinel_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(yolov8_detector_node src/detector/yolov8_detector_node.cpp)
ament_target_dependencies(yolov8_detector_node
  rclcpp
  sensor_msgs
  sentinel_msgs
  cv_bridge
  OpenCV
)
target_compile_features(yolov8_detector_node PUBLIC c_std_99 cxx_std_17)

add_executable(bytetrack_node src/tracker/bytetrack_node.cpp)
ament_target_dependencies(bytetrack_node
  rclcpp
  sentinel_msgs
  geometry_msgs
)
target_compile_features(bytetrack_node PUBLIC c_std_99 cxx_std_17)

add_executable(ballistics_node src/ballistics/ballistics_node.cpp)
ament_target_dependencies(ballistics_node
  rclcpp
  sentinel_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)
target_compile_features(ballistics_node PUBLIC c_std_99 cxx_std_17)

add_executable(servo_controller_node src/servo_controller/servo_controller_node.cpp)
ament_target_dependencies(servo_controller_node
  rclcpp
  sentinel_msgs
  geometry_msgs
)
target_compile_features(servo_controller_node PUBLIC c_std_99 cxx_std_17)

install(TARGETS
  yolov8_detector_node
  bytetrack_node
  ballistics_node
  servo_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
