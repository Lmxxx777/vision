add_library(robot_status "")
target_sources(robot_status
  PRIVATE
      robot_status.cpp
  PUBLIC
      ${CMAKE_CURRENT_LIST_DIR}/robot_status.h
  )
target_include_directories(robot_status
  PUBLIC
      ${CMAKE_CURRENT_LIST_DIR}
  )