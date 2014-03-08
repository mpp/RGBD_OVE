cmake_minimum_required(VERSION 2.8.3)
project(RGBD_OVE)

SET(CMAKE_BUILD_TYPE Debug)

ADD_DEFINITIONS(
    -std=c++11 # Or -std=c++0x
    -Wno-deprecated
)

find_package(PCL 1.3 REQUIRED COMPONENTS common visualization io filters sample_consensus segmentation registration)

include_directories(/usr/include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(
    ${PROJECT_NAME}
    src/io_test.cpp
)

target_link_libraries(
    ${PROJECT_NAME}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_SAMPLE_CONSENSUS_LIBRARIES}
    ${PCL_SEGMENTATION_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${PCL_FILTERS_LIBRARIES}
    ${PCL_VISUALIZATION_LIBRARIES}
    ${PCL_REGISTRATION_LIBRARIES}
)
