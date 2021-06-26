cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME qr_feature_extractor)
project(${PROJECT_NAME})

add_definitions(-std=c++11)

## ================= Set project variables ==================

set(QR_FEATURE_EXTRACTOR_SOURCE_DIR src/source)

set(QR_FEATURE_EXTRACTOR_INCLUDE_DIR src/include)

set(QR_FEATURE_EXTRACTOR_SOURCE_FILES
  ${QR_FEATURE_EXTRACTOR_SOURCE_DIR}/qr_feature_extractor.cpp
  ${QR_FEATURE_EXTRACTOR_SOURCE_DIR}/qr_feature_extractor.cpp
)

set(QR_FEATURE_EXTRACTOR_HEADER_FILES
  ${QR_FEATURE_EXTRACTOR_INCLUDE_DIR}/qr_feature_extractor.h
)

## ================= Find catkin macros and libraries =================

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  droneMsgsROS
  drone_process
  robot_process
  sensor_msgs
  aerostack_msgs
  cv_bridge
  image_transport
)
## ================= Catkin specific configuration =================

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  robot_process
  droneMsgsROS
  aerostack_msgs
)

## ================= Build =================

include_directories(${QR_FEATURE_EXTRACTOR_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})

add_library(

  qr_feature_extractor_lib
  ${QR_FEATURE_EXTRACTOR_SOURCE_FILES}
  ${QR_FEATURE_EXTRACTOR_HEADER_FILES}
)
add_dependencies(qr_feature_extractor_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(qr_feature_extractor_lib zbar ${catkin_LIBRARIES})
add_executable(
  qr_feature_extractor
  ${QR_FEATURE_EXTRACTOR_SOURCE_DIR}/qr_feature_extractor_main.cpp
)
add_dependencies(qr_feature_extractor ${catkin_EXPORTED_TARGETS})
target_link_libraries(qr_feature_extractor qr_feature_extractor_lib)
target_link_libraries(qr_feature_extractor ${catkin_LIBRARIES})
