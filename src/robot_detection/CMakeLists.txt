cmake_minimum_required(VERSION 3.15)

project(robot_detection)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++17 -Os -pthread -g")
# set(EIGEN_DIR /usr/include/eigen3)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++17 -pthread")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rosconsole
  sensor_msgs
  cv_bridge
  image_geometry
  image_proc
  image_transport
  message_filters
  camera_info_manager
  geometry_msgs
  tf

  robot_msgs
)

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(Ceres REQUIRED)
# find_package(matplotlib_cpp REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES armor_detection
 CATKIN_DEPENDS roscpp rospy std_msgs rosconsole sensor_msgs cv_bridge image_geometry image_proc image_transport message_filters camera_info_manager geometry_msgs tf robot_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/robot_status
  ${PROJECT_SOURCE_DIR}/number_DNN
  ${PROJECT_SOURCE_DIR}/armor_detection
  ${PROJECT_SOURCE_DIR}/armor_track            
  ${PROJECT_SOURCE_DIR}/armor_prediction
  ${PROJECT_SOURCE_DIR}/gimbal_control
  ${PROJECT_SOURCE_DIR}/buff_solution
)


add_library(robot_detection_lib
            ${PROJECT_SOURCE_DIR}/number_DNN/number_DNN.cpp
            ${PROJECT_SOURCE_DIR}/number_DNN/number_DNN.h 
            ${PROJECT_SOURCE_DIR}/armor_detection/armor_detection.cpp
            ${PROJECT_SOURCE_DIR}/armor_detection/armor_detection.h
            ${PROJECT_SOURCE_DIR}/armor_track/armor_track.cpp
            ${PROJECT_SOURCE_DIR}/armor_track/armor_track.h            
            ${PROJECT_SOURCE_DIR}/armor_prediction/armor_prediction.cpp
            ${PROJECT_SOURCE_DIR}/armor_prediction/armor_prediction.h
            ${PROJECT_SOURCE_DIR}/armor_prediction/singer_prediction.cpp
            ${PROJECT_SOURCE_DIR}/armor_prediction/singer_prediction.h
            ${PROJECT_SOURCE_DIR}/gimbal_control/gimbal_control.cpp
            ${PROJECT_SOURCE_DIR}/gimbal_control/gimbal_control.h
            ${PROJECT_SOURCE_DIR}/robot_status/robot_status.cpp
            ${PROJECT_SOURCE_DIR}/robot_status/robot_status.h
            ${PROJECT_SOURCE_DIR}/buff_solution/buff_detection.cpp
            ${PROJECT_SOURCE_DIR}/buff_solution/buff_detection.h
            ${PROJECT_SOURCE_DIR}/buff_solution/buff_prediction.cpp
            ${PROJECT_SOURCE_DIR}/buff_solution/buff_prediction.h
)

# for auto aim
add_executable(robot_detection robot_detection.cpp)
# for buff aim
# add_executable(robot_detection buff_solution.cpp)
# for debug buff
# add_executable(robot_detection buff_debug.cpp)

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_detection
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Eigen3_LIBRARIES}
  ${Boost_INCLUDE_DIRS}
  # Ceres::ceres
  robot_detection_lib
)











