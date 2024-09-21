project(pidProject)

add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)
include_directories(${CMAKE_SOURCE_DIR}/include)

add_subdirectory(pidProjectLib)
target_link_libraries(${PROJECT_NAME} pidProjectLib)
