cmake_minimum_required(VERSION 3.0.2)
project(imu_orientation_estimator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11 -O0 -Wall")


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  nav_msgs
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
)
catkin_package(CATKIN_DEPENDS  roscpp sensor_msgs  geometry_msgs  nav_msgs  tf  tf2  tf2_ros  tf2_geometry_msgs  std_msgs)


include_directories(
include
  ${catkin_INCLUDE_DIRS}
  "/usr/include/Eigen3"
)

add_library(${PROJECT_NAME} SHARED 
    src/imu_orientation_estimator.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )   #Eigen没有库文件

# find_package(OpenCV REQUIRED)
# include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PROJECT_NAME})