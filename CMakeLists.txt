cmake_minimum_required(VERSION 3.0.2)
project(lidar_ohyejun)

## [권장] Eigen과 최신 라이브러리 호환을 위해 C++14 권장 (C++11도 되긴 함)
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  vision_msgs
  pcl_ros
  pcl_conversions
  # [★추가됨 1] 내 차 속도(Odometry)와 시각화(Marker)를 위해 필수
  nav_msgs
  visualization_msgs
)

## System dependencies
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}  # [수정] 대소문자 주의 (보통 Eigen3_INCLUDE_DIRS 사용)
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare this project as a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    geometry_msgs
    vision_msgs
    pcl_ros
    pcl_conversions
    # [★추가됨 2] 다른 패키지에서 참조할 때 필요
    nav_msgs
    visualization_msgs
  DEPENDS
    Eigen3
    PCL
    OpenCV
)

###########
## Build ##
###########

## 1. Clustering Node
add_executable(gigacha_lidar_clustering
  src/lidar.cpp
)

target_link_libraries(gigacha_lidar_clustering
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(gigacha_lidar_clustering
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## 2. Tracking Node (EKF)
# [주의] 파일명이 src/tracking.cpp 인지 src/tracking2.cpp 인지 꼭 확인하세요!
add_executable(gigacha_lidar_tracking
  src/tracking2.cpp
)

target_link_libraries(gigacha_lidar_tracking
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  # Eigen은 보통 헤더 온리라 링크 안 해도 되지만, 문제 생기면 아래 주석 해제
  # ${Eigen3_LIBRARIES} 
)

add_dependencies(gigacha_lidar_tracking
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

install(TARGETS
  gigacha_lidar_clustering
  gigacha_lidar_tracking
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)