cmake_minimum_required(VERSION 3.15)

project(send_message)

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
  message_filters
  camera_info_manager
  
  robot_msgs
)

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
# find_package(matplotlib_cpp REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES armor_detection
 CATKIN_DEPENDS roscpp rospy std_msgs rosconsole sensor_msgs message_filters robot_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(send_pub
  send.cpp
)
add_executable(receive_sub
  receive.cpp
)

target_link_libraries(send_pub
  ${catkin_LIBRARIES}
)
target_link_libraries(receive_sub
  ${catkin_LIBRARIES}
)

# # add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(robot_detection
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBS}
#   ${Eigen3_LIBRARIES}
#   ${Boost_INCLUDE_DIRS}
# )











