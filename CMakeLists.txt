cmake_minimum_required(VERSION 3.0.2)
project(lidar_ohyejun)

## [권장] C++14 표준 사용
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  vision_msgs
  pcl_ros
  pcl_conversions
  nav_msgs            # [필수] 내 차 속도
  visualization_msgs  # [필수] 마커 시각화
  patchworkpp         # [★추가됨] 여기에 추가해야 합니다
)

## System dependencies
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
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
    nav_msgs
    visualization_msgs
    patchworkpp      # [★추가됨] 의존성 명시
  DEPENDS
    PCL
    OpenCV
)

###########
## Build ##
###########

## ===============================================
## 1. Clustering Node (Patchwork++ 적용 대상)
## ===============================================
add_executable(gigacha_lidar_clustering
  src/lidar.cpp 
)

target_link_libraries(gigacha_lidar_clustering
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  patchworkpp::patchworkpp  # [★추가됨] 라이브러리 링크 필수!
)

add_dependencies(gigacha_lidar_clustering
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## ===============================================
## 2. Tracking Node (기존 유지)
## ===============================================
add_executable(gigacha_lidar_tracking
  src/tracking2.cpp
)

target_link_libraries(gigacha_lidar_tracking
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(gigacha_lidar_tracking
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## ===============================================
## 3. Cone Mission Node (기존 유지)
## ===============================================
add_executable(gigacha_lidar_cone
  src/lidar_cone.cpp
)

target_link_libraries(gigacha_lidar_cone
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(gigacha_lidar_cone
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

install(TARGETS
  gigacha_lidar_clustering
  gigacha_lidar_tracking
  gigacha_lidar_cone
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)