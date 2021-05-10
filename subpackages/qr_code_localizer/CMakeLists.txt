cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME qr_code_localizer)
project(${PROJECT_NAME})

add_definitions(-std=c++11)

## ================= Set project variables ==================

set(QR_CODE_LOCALIZER_SOURCE_DIR src/source)

set(QR_CODE_LOCALIZER_INCLUDE_DIR src/include)

set(QR_CODE_LOCALIZER_SOURCE_FILES
  ${QR_CODE_LOCALIZER_SOURCE_DIR}/qr_code_localizer.cpp
  ${QR_CODE_LOCALIZER_SOURCE_DIR}/qr_code_localizer.cpp
)

set(QR_CODE_LOCALIZER_HEADER_FILES
  ${QR_CODE_LOCALIZER_INCLUDE_DIR}/qr_code_localizer.h
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

include_directories(${QR_CODE_LOCALIZER_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})

add_library(

  qr_code_localizer_lib
  ${QR_CODE_LOCALIZER_SOURCE_FILES}
  ${QR_CODE_LOCALIZER_HEADER_FILES}
)
add_dependencies(qr_code_localizer_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(qr_code_localizer_lib zbar ${catkin_LIBRARIES})
add_executable(
  qr_code_localizer
  ${QR_CODE_LOCALIZER_SOURCE_DIR}/qr_code_localizer_main.cpp
)
add_dependencies(qr_code_localizer ${catkin_EXPORTED_TARGETS})
target_link_libraries(qr_code_localizer qr_code_localizer_lib)
target_link_libraries(qr_code_localizer ${catkin_LIBRARIES})
