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

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
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
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(gigacha_lidar_clustering
  src/lidar.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(gigacha_lidar_clustering
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
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

###########
## Test ##
###########

## Add gtest based C++ test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lidar_ohyejun.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
