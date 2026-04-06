cmake_minimum_required(VERSION 3.8)
project(capra_udp_bridge)

find_package(rclcpp REQUIRED)

add_executable(udp_bridge_node
  src/udp_bridge_node.cpp
  src/udp_bridge.cpp
)

target_include_directories(udp_bridge_node PUBLIC
  include
)

ament_target_dependencies(udp_bridge_node rclcpp)

install(TARGETS udp_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()