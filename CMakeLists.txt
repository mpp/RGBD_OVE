cmake_minimum_required(VERSION 2.8.3)
project(RGBD_OVE)

SET(CMAKE_BUILD_TYPE Debug)

ADD_DEFINITIONS(
    -std=c++11 # Or -std=c++0x
    -Wno-deprecated
)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR})
find_package( OpenNI2 REQUIRED )

find_package(PCL 1.3 REQUIRED COMPONENTS common visualization io filters sample_consensus segmentation registration)

include_directories(/usr/include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

FIND_PACKAGE(VTK)
INCLUDE(${VTK_USE_FILE})

set(QT_QMAKE_EXECUTABLE /usr/bin/qmake-qt4)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})
include_directories(src)

file(GLOB UI_FILES ui/*.ui)
file(GLOB QT_WRAP src/*.h)
file(GLOB CXX_FILES *.cpp)

QT4_WRAP_UI(UISrcs ${UI_FILES})
QT4_WRAP_CPP(MOCSrcs ${QT_WRAP})

find_package( OpenCV REQUIRED )

add_executable(
    ${PROJECT_NAME}
    src/io_test.cpp
    src/visualizerthread.cpp
    src/cloudsgrabber.cpp
    src/workerthread.cpp
)

add_executable(
    qttest
    ${CXX_FILES}
    src/qttest.cpp
    src/pclwindow.cpp
    src/cloudinteraction.cpp
    src/simplecloudgrabber.cpp
    ${UISrcs} ${MOCSrcs}
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
#    ${OPENNI2_LIBRARIES}
)

target_link_libraries(
    qttest QVTK ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${OPENNI2_LIBRARIES}
)
