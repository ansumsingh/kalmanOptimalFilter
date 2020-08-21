cmake_minimum_required(VERSION 3.0)

project(KalmanFilterUnittest)


find_package(Eigen3 CONFIG REQUIRED)
find_package(Catch2 CONFIG REQUIRED)

# get the parent directory
get_filename_component(PARENT_DIR ${CMAKE_SOURCE_DIR} DIRECTORY)

include_directories(${PARENT_DIR}/include)

add_executable(kalmanFilter_unittests main.cpp kalmanFilter.Test.cpp)

set_property(TARGET kalmanFilter_unittests PROPERTY CXX_STANDARD 17)
set_property(TARGET kalmanFilter_unittests PROPERTY CXX_STANDARD_REQUIRED ON)

set(UNIT_TEST kalmanFilter_unittests)

target_link_libraries(kalmanFilter_unittests PRIVATE Eigen3::Eigen Catch2::Catch2)

#add the unittests at the end.
add_custom_command(
     TARGET ${UNIT_TEST}
     COMMENT "Run tests ${UNIT_TEST}"
     POST_BUILD 
     COMMAND ${UNIT_TEST}
)