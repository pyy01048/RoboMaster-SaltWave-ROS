cmake_minimum_required(VERSION 3.8)
project(sentinel_decision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sentinel_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

add_library(${PROJECT_NAME}_bt_nodes SHARED
  bt_nodes/ActionActivateAiming.cpp
  bt_nodes/ActionNavigate.cpp
  bt_nodes/ActionSetMode.cpp
  bt_nodes/ConditionTargetVisible.cpp
)

ament_target_dependencies(${PROJECT_NAME}_bt_nodes
  rclcpp
  geometry_msgs
  sentinel_msgs
  behaviortree_cpp_v3
)

target_compile_features(${PROJECT_NAME}_bt_nodes PUBLIC c_std_99 cxx_std_17)

add_executable(state_manager_node src/state_manager/state_manager_node.cpp)
ament_target_dependencies(state_manager_node
  rclcpp
  rclcpp_components
  geometry_msgs
  sentinel_msgs
)
target_compile_features(state_manager_node PUBLIC c_std_99 cxx_std_17)

install(TARGETS
  ${PROJECT_NAME}_bt_nodes
  state_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
