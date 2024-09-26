project(pidProjectLib)

add_library(${PROJECT_NAME} STATIC pidProjectLib.cpp CP2112.cpp Max31889.cpp)

# find and link to labjack shared lib
target_link_libraries(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/libs/liblabjackusb.so.2.7.0)
target_link_libraries(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/libs/libu3.a)

# Find libraries
find_library(SLABHIDDEVICE_LIBRARY slabhiddevice PATHS ${CMAKE_SOURCE_DIR}/USBXpressHostSDK/CP2112/lib/x86_64)
find_library(SLABHIDTOSMBUS_LIBRARY slabhidtosmbus PATHS ${CMAKE_SOURCE_DIR}/USBXpressHostSDK/CP2112/lib/x86_64)
target_link_libraries(${PROJECT_NAME} ${SLABHIDDEVICE_LIBRARY} ${SLABHIDTOSMBUS_LIBRARY})

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/USBXpressHostSDK/CP2112/include)
include_directories(${CMAKE_SOURCE_DIR}/USBXpressHostSDK/CP2112/srcpkg/slabhiddevice/slabhiddevice/src)
