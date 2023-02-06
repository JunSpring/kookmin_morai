cmake_minimum_required(VERSION 3.0.2)
project(rubber_cone)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  rubber_cone_msg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rubber_cone
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS system_lib
)

file(GLOB_RECURSE project_HEADERS
  "*.h"
  "*.hpp"
)

add_executable(
  ${PROJECT_NAME} 
  src/rubber_cone.cpp
  src/main.cpp
)

include_directories( include
  ${catkin_INCLUDE_DIRS}
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)