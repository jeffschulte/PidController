project(pidProject)

# Set RPATH for the executable
set(CMAKE_INSTALL_RPATH "${CMAKE_BINARY_DIR}/src")
set(CMAKE_BUILD_RPATH "${CMAKE_BINARY_DIR}/src")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_SKIP_RPATH FALSE)
set(CMAKE_SKIP_INSTALL_RPATH FALSE)

add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)

add_subdirectory(pidProjectLib)
target_link_libraries(${PROJECT_NAME} pidProjectLib)

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/USBXpressHostSDK/CP2112/include)
include_directories(${CMAKE_SOURCE_DIR}/USBXpressHostSDK/CP2112/srcpkg/slabhiddevice/slabhiddevice/src)

