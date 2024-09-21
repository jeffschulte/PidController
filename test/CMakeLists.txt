project(pidProjectTest)

add_executable(${PROJECT_NAME} test.cpp)
target_link_libraries(${PROJECT_NAME} pidProjectLib)
include_directories(${CMAKE_SOURCE_DIR}/include)

add_test(NAME PidProjectTest1 COMMAND "pidProjectTest" "test1")
add_test(NAME PidProjectTest2 COMMAND "pidProjectTest" "test2")