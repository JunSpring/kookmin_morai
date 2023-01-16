cmake_minimum_required(VERSION 3.0.2)
project(lidar_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  cv_bridge
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  lidar0.msg
  lidar1.msg
  lidar2.msg
  lidarALL.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lidar_detection
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs 
)


file(GLOB_RECURSE project_HEADERS
  "*.h"
  "*.hpp"
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME} 
  src/Scan2PCL.cpp
  src/Cluster.cpp
  src/main.cpp
)

set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.7 /usr/include/eigen3/)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)
