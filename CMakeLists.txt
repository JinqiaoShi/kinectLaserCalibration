cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "bin/")
rosbuild_init()
rosbuild_add_boost_directories()
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#SET(laser_SOURCES src/main.cpp src/laserWindow.cpp)
#SET(laser_HEADERS src/laserWindow.h)
#SET(laser_FORMS src/laserWindow.ui)

FIND_PACKAGE(Qt4 REQUIRED)
SET(QT_USE_QTVTK TRUE)
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})
SET(my_FORMS src/dialog.ui src/laser.ui)
QT4_WRAP_UI(my_FORMS_HEADERS ${my_FORMS})
SET(my_HEADERS src/laserWindow.h src/laser.h src/spinner.h)
QT4_WRAP_CPP(my_HEADERS_MOC ${my_HEADERS})
INCLUDE(${QT_USE_FILE})
#ADD_DEFINITIONS(${QT_DEFINITIONS})

rosbuild_add_executable(laserFlatter src/main.cpp src/laserWindow.cpp src/laser.cpp src/spinner.cpp
    ${my_FORMS_HEADERS} ${my_HEADERS_MOC})

TARGET_LINK_LIBRARIES(laserFlatter ${QT_LIBRARIES} ${VTK_LIBRARIES} QVTK)
rosbuild_link_boost(laserFlatter thread signals system)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
