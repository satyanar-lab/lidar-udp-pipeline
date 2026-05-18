cmake_minimum_required(VERSION 3.8)
project(rplidar_ros)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Reuse the parser source UNCHANGED, from the main repo
set(PARSER_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../..)

add_executable(rplidar_node
  src/rplidar_node.cpp
  ${PARSER_ROOT}/src/rplidar_parser.cpp
)
target_include_directories(rplidar_node PRIVATE ${PARSER_ROOT}/include)
target_compile_features(rplidar_node PRIVATE cxx_std_17)

ament_target_dependencies(rplidar_node rclcpp sensor_msgs)

install(TARGETS rplidar_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()