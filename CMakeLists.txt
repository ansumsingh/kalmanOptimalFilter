cmake_minimum_required(VERSION 2.7)
project(KalmanFilterLibrary)

find_package(Eigen3 REQUIRED)

    

include_directories(${Eigen3_INCLUDE_DIRS})

add_executable(kalmanFilter kalmanFilter.hpp kalmanFilterTest.cpp)


target_link_libraries(kalmanFilter ${Eigen3_LIBRARIES})

