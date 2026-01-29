cmake_minimum_required(VERSION 3.0.2)
project(lidar_ohyejun)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  vision_msgs
  pcl_ros
  pcl_conversions
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
# [★추가됨 1] OpenCV 패키지 찾기
find_package(OpenCV REQUIRED)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  # [★추가됨 2] OpenCV 헤더 경로 포함
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
  DEPENDS
    Eigen3
    PCL
    # [★추가됨 3] 패키지 의존성에 OpenCV 추가 (선택사항이나 권장)
    OpenCV
)

###########
## Build ##
###########

## Declare a C++ executable (Clustering)
add_executable(gigacha_lidar_clustering
  src/lidar.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(gigacha_lidar_clustering
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  # [★추가됨 4] 클러스터링 노드에 OpenCV 라이브러리 링크 (필수!)
  ${OpenCV_LIBRARIES}
)

## Add cmake target dependencies of the executable
add_dependencies(gigacha_lidar_clustering
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Declare a C++ executable for tracking
add_executable(gigacha_lidar_tracking
  src/tracking2.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(gigacha_lidar_tracking
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  # [선택] 트래킹 쪽도 나중에 OpenCV 쓸 수 있으니 미리 넣어두면 좋음
  ${OpenCV_LIBRARIES}
)

## Add cmake target dependencies of the executable
add_dependencies(gigacha_lidar_tracking
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS
  gigacha_lidar_clustering
  gigacha_lidar_tracking
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)