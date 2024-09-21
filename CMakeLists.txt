cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
enable_testing()

project(PidProject VERSION 1.0)

add_subdirectory(src)
add_subdirectory(test)

set(CMAKE_BINARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/build")