cmake_minimum_required(VERSION 3.0)
project(KalmanFilterUnittest)


find_package(Eigen3 Catch2 CONFIG REQUIRED)
find_package(Catch2 CONFIG REQUIRED)

include_directories(../include)
add_executable(kalmanFilter.test main.cpp kalmanFilter.Test.cpp)

set(UNIT_TEST kalmanFilter.test)
add_custom_command(
     TARGET ${UNIT_TEST}
     COMMENT "Run tests ${UNIT_TEST}"
     POST_BUILD 
     COMMAND ${UNIT_TEST}
)

target_link_libraries(kalmanFilter.test PRIVATE Eigen3::Eigen Catch2::Catch2)