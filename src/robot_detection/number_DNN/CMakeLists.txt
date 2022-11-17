add_library(number_DNN "")
target_sources(number_DNN
  PRIVATE
      number_DNN.cpp
  PUBLIC
      ${CMAKE_CURRENT_LIST_DIR}/number_DNN.h
  )
target_include_directories(number_DNN
  PUBLIC
      ${CMAKE_CURRENT_LIST_DIR}
  )