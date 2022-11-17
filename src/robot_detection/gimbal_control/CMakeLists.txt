add_library(gimbal_control "")
target_sources(gimbal_control
  PRIVATE
      gimbal_control.cpp
  PUBLIC
      ${CMAKE_CURRENT_LIST_DIR}/gimbal_control.h
  )
target_include_directories(gimbal_control
  PUBLIC
      ${CMAKE_CURRENT_LIST_DIR}
  )