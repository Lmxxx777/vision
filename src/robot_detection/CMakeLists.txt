cmake_minimum_required(VERSION 3.0.2)
project(robot_detection)

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(autoaim robot_detection_node.cpp)

add_subdirectory(robot_status)
add_subdirectory(number_DNN)
add_subdirectory(armor_detection)
add_subdirectory(armor_track)
add_subdirectory(armor_prediction)
add_subdirectory(gimbal_control)



target_link_libraries(autoaim
    gimbal_control
    armor_prediction
    armor_track
    armor_detection
    number_DNN
    robot_status
  )

