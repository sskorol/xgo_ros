cmake_minimum_required(VERSION 3.0.2)
project(xgo_ros)

list(APPEND CMAKE_PREFIX_PATH "/usr/local")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  message_runtime
  tf
)
find_library(
  SERIAL_LIBRARY
  NAMES serial
  HINTS ${CMAKE_PREFIX_PATH}/lib
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES xgo_ros
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${CMAKE_PREFIX_PATH}/include
)

add_library(${PROJECT_NAME}
  src/xgo.cpp
)

add_dependencies(
${PROJECT_NAME}
${${PROJECT_NAME}_EXPORTED_TARGETS}
${catkin_EXPORTED_TARGETS}
)

add_executable(${PROJECT_NAME}_node 
src/xgo_control_node.cpp
src/xgo.cpp
)

target_link_libraries(${PROJECT_NAME}_node PUBLIC
  ${SERIAL_LIBRARY}
  ${catkin_LIBRARIES}
)