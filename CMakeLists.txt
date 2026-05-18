cmake_minimum_required(VERSION 3.16)
project(lidar_udp_pipeline CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -O2)

add_executable(udp_capture src/udp_capture.cpp)