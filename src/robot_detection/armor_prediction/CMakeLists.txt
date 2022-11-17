
find_package(Eigen3 3.3 REQUIRED CONFIG)
if(TARGET Eigen3::Eigen)
  message(STATUS "Eigen3 v${EIGEN3_VERSION_STRING} found in ${EIGEN3_INCLUDE_DIR}")
endif()


add_library(armor_prediction "")
target_sources(armor_prediction
  PRIVATE
  armor_prediction.cpp
  PUBLIC
      ${CMAKE_CURRENT_LIST_DIR}/armor_prediction.h
  )
target_include_directories(armor_prediction
  PUBLIC
      ${CMAKE_CURRENT_LIST_DIR}
  )