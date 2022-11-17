add_library(armor_detection "")
target_sources(armor_detection
  PRIVATE
      armor_detection.cpp
  PUBLIC
      ${CMAKE_CURRENT_LIST_DIR}/armor_detection.hpp
  )
target_include_directories(armor_detection
  PUBLIC
      ${CMAKE_CURRENT_LIST_DIR}
  )