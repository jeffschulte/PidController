project(pidProjectLib)

add_library(${PROJECT_NAME} STATIC pidProjectLib.cpp)
include_directories(${CMAKE_SOURCE_DIR}/include)
message(STATUS "CMAKE_SOURCE_DIR is: ${CMAKE_SOURCE_DIR}")