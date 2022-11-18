cmake_minimum_required(VERSION 3.15)

project(robot_detection)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++17 -Os -pthread -g")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++17 -pthread")

find_package(OpenCV 4 REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} main.cpp
            ${PROJECT_SOURCE_DIR}/robot_status/robot_status.cpp
            ${PROJECT_SOURCE_DIR}/robot_status/robot_status.h
            ${PROJECT_SOURCE_DIR}/number_DNN/number_DNN.cpp
            ${PROJECT_SOURCE_DIR}/number_DNN/number_DNN.h 
            ${PROJECT_SOURCE_DIR}/armor_detection/armor_detection.cpp
            ${PROJECT_SOURCE_DIR}/armor_detection/armor_detection.hpp
            ${PROJECT_SOURCE_DIR}/armor_track/armor_track.cpp
            ${PROJECT_SOURCE_DIR}/armor_track/armor_track.h            
            ${PROJECT_SOURCE_DIR}/armor_prediction/armor_prediction.cpp
            ${PROJECT_SOURCE_DIR}/armor_prediction/armor_prediction.h
            ${PROJECT_SOURCE_DIR}/gimbal_control/gimbal_control.cpp
            ${PROJECT_SOURCE_DIR}/gimbal_control/gimbal_control.h
)

# 包含海康相机动态链接库
# find_library(MVS_LIB libMvCameraControl.so /opt/MVS/lib/64/)
# target_link_libraries(Infantry_Vision ${MVS_LIB})


