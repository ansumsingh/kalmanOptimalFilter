cmake_minimum_required(VERSION 2.7)
project(KalmanFilterUnittest)

find_package(Eigen3 REQUIRED)

    

include_directories(${Eigen3_INCLUDE_DIRS})

add_executable(kalmanFilter.test kalmanFilter.hpp main.cpp kalmanFilterTest.cpp)


target_link_libraries(kalmanFilter.test PRIVATE Eigen3::Eigen3 Catch2::Catch2)

