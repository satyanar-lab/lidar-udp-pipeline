cmake_minimum_required(VERSION 3.16)
project(lidar_udp_pipeline CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -O2)

add_executable(udp_capture src/udp_capture.cpp)

add_library(rplidar_parser STATIC src/rplidar_parser.cpp)
target_include_directories(rplidar_parser PUBLIC include)

add_executable(parse_file src/parse_file.cpp)
target_link_libraries(parse_file PRIVATE rplidar_parser)