project(pidProject)

add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)

add_subdirectory(pidProjectLib)
target_link_libraries(${PROJECT_NAME} pidProjectLib)

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/USBXpressHostSDK/CP2112/include)
include_directories(${CMAKE_SOURCE_DIR}/USBXpressHostSDK/CP2112/srcpkg/slabhiddevice/slabhiddevice/src)

