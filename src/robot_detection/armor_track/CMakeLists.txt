add_library(armor_track "")
target_sources(armor_track
  PRIVATE
      armor_track.cpp
  PUBLIC
      ${CMAKE_CURRENT_LIST_DIR}/armor_track.h
  )
target_include_directories(armor_track
  PUBLIC
      ${CMAKE_CURRENT_LIST_DIR}
  )