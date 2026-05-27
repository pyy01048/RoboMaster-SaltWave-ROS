cmake_minimum_required(VERSION 3.8)
project(decision_making)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sentinel_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

include_directories(include)

add_library(bt_nodes SHARED
  src/bt_nodes.cpp
)
ament_target_dependencies(bt_nodes
  rclcpp
  geometry_msgs
  sentinel_msgs
  behaviortree_cpp_v3
)

add_executable(decision_node src/decision_node.cpp)
ament_target_dependencies(decision_node
  rclcpp
  geometry_msgs
  sentinel_msgs
  behaviortree_cpp_v3
)
target_link_libraries(decision_node bt_nodes)

install(TARGETS
  bt_nodes
  decision_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  behavior_trees
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
