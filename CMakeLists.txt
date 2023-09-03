cmake_minimum_required(VERSION 3.5)
project(xgo2_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# list(APPEND CMAKE_PREFIX_PATH "/usr/local")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(serial REQUIRED)

# find_library(
#   SERIAL_LIBRARY
#   NAMES serial
#   HINTS ${CMAKE_PREFIX_PATH}/lib
# )

include_directories(
  include
  /usr/include/jsoncpp
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  # ${CMAKE_PREFIX_PATH}/include
)

add_library(${PROJECT_NAME} src/xgo.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
  serial
  nav_msgs
  jsoncpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)

add_executable(${PROJECT_NAME}_node src/xgo_control_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${SERIAL_LIBRARY}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
  jsoncpp
)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  sensor_msgs
)

install(TARGETS 
  ${PROJECT_NAME}
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  config meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  launch/xgo_control_launch.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp std_msgs sensor_msgs geometry_msgs)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
