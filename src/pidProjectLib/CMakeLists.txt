project(pidProjectLib)

add_library(${PROJECT_NAME} STATIC pidProjectLib.cpp CP2112.cpp Max31889.cpp)

# find and link to labjack shared lib
target_link_libraries(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/libs/liblabjackusb.so.2.7.0)
target_link_libraries(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/libs/libu3.a)

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/USBXpressHostSDK/CP2112/include)
include_directories(${CMAKE_SOURCE_DIR}/USBXpressHostSDK/CP2112/srcpkg/slabhiddevice/slabhiddevice/src)

# Find libraries
find_library(SLABHIDDEVICE_LIBRARY slabhiddevice PATHS ${CMAKE_SOURCE_DIR}/USBXpressHostSDK/CP2112/lib/x86_64)
find_library(SLABHIDTOSMBUS_LIBRARY slabhidtosmbus PATHS ${CMAKE_SOURCE_DIR}/USBXpressHostSDK/CP2112/lib/x86_64)
target_link_libraries(${PROJECT_NAME} ${SLABHIDDEVICE_LIBRARY} ${SLABHIDTOSMBUS_LIBRARY})

# Post-build action: Copy files to the executable directory
add_custom_command(TARGET ${PROJECT_NAME}
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/USBXpressHostSDK/CP2112/lib/x86_64/libslabhiddevice.so.1"
        "${CMAKE_BINARY_DIR}/src"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/USBXpressHostSDK/CP2112/lib/x86_64/libslabhidtosmbus.so.1"
        "${CMAKE_BINARY_DIR}/src"
    COMMENT "Copying shared libraries to executable directory... ${CMAKE_SOURCE_DIR}/USBXpressHostSDK/CP2112/lib/x86_64/libslabhidtosmbus.so.1 ${CMAKE_BINARY_DIR}/src"
)
